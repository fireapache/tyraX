/*
# Modified by TyraX - grow-only package pools (no new[] per submit) and
# object-space AABB frustum classification in checkFrustum (see the comment
# on that function).
# Based on the original by Sandro Sobczynski (h4570/tyra), Apache License 2.0.
*/

#include <tamtypes.h>
#include "debug/hardware_trace.hpp"
#include <math.h>
#include "debug/debug.hpp"
#include "renderer/3d/pipeline/static/core/bag/packaging/stapip_bag_packager.hpp"
#include "renderer/3d/pipeline/static/core/stapip_probes.hpp"

// Modified by TyraX: integer ceiling division avoids software double math on EE.
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"

namespace Tyra {

// Modified by TyraX: the boxes, from a float bag's StaPipBagPackagesBBox or
// from a compact stream's own (six floats a part / a coarse group - the
// stream's coarse groups are 24 parts, the 8 full packages the old coarse
// level covers).
void StaPipBagPackager::mergedMinMax(u32 index, u32 parts, Vec4* mn,
                                     Vec4* mx) const {
  if (renderBBox != nullptr) {
    renderBBox->getMergedMinMax(index, static_cast<u16>(parts), mn, mx);
    return;
  }
  const u32 count = renderStream->partCount();
  if (index >= count) index = count - 1;
  if (index + parts > count) parts = count - index;
  const float* b = renderStream->partBoxes() + index * 6;
  mn->set(b[0], b[1], b[2], 1.0F);
  mx->set(b[3], b[4], b[5], 1.0F);
  for (u32 i = 1; i < parts; ++i) {
    const float* q = b + i * 6;
    if (q[0] < mn->x) mn->x = q[0];
    if (q[1] < mn->y) mn->y = q[1];
    if (q[2] < mn->z) mn->z = q[2];
    if (q[3] > mx->x) mx->x = q[3];
    if (q[4] > mx->y) mx->y = q[4];
    if (q[5] > mx->z) mx->z = q[5];
  }
}

void StaPipBagPackager::coarseMinMax(u32 group, Vec4* mn, Vec4* mx) const {
  if (renderBBox != nullptr) {
    *mn = renderBBox->coarseMin(group);
    *mx = renderBBox->coarseMax(group);
    return;
  }
  const u32 groups = renderStream->coarseCount();
  const float* b =
      renderStream->coarseBoxes() + (group < groups ? group : groups - 1) * 6;
  mn->set(b[0], b[1], b[2], 1.0F);
  mx->set(b[3], b[4], b[5], 1.0F);
}

#if TYRA_STAPIP_PROBE_ACCEPT_ALL
// Probe A1 (stapip_probes.hpp). The test ran; its rejection is thrown away.
// PARTIALLY_IN_FRUSTUM is deliberately untouched - the clip route must keep
// every package that straddles a plane, or the cull programs, which do not
// clip, draw wedges.
static inline CoreBBoxFrustum probeAcceptAll(CoreBBoxFrustum result) {
  return result == OUTSIDE_FRUSTUM ? IN_FRUSTUM : result;
}
#endif

#if TYRA_STAPIP_ATTRIB
// Added by TyraX: the attribution pass' own clock (stapip_attrib.hpp). The
// classification is the one bracket in the dispatch split that is measured
// EXCLUSIVELY rather than derived as a residual, which is the whole point of
// putting it here rather than around create().
static inline u32 readPackagerAttribTicks() {
  u32 ticks;
  asm volatile("mfc0 %0, $9" : "=r"(ticks));
  return ticks;
}
#endif

StaPipBagPackager::StaPipBagPackager() {}

StaPipBagPackager::~StaPipBagPackager() {}

void StaPipBagPackager::init(Renderer3DFrustumPlanes* t_frustumPlanes) {
  frustumPlanes = t_frustumPlanes;
}

/**
 * @brief Create render packages from provided render data
 *
 * @param size Max maxVertCount verts (VU1 buffer size)
 */
StaPipBagPackage* StaPipBagPackager::create(u16* o_size, StaPipBag* data,
                                            u16 size) {
  HardwareTrace::Scope trace("Package_create");
  TYRA_ASSERT(size <= maxVertCount, "StaPipBagPackage can have max ",
              maxVertCount, " verts. Provided \"", size, "\"");

  *o_size = (data->count + size - 1) / size;
#if TYRA_STAPIP_ATTRIB
  stats.packages += *o_size;
#endif
  // Modified by TyraX: grow-only pool instead of new[] per submit.
  // Pool entries are reused, so pointers absent from this bag must be
  // reset - a stale sts/colors/normals from a previous bag would otherwise
  // leak into the fill path.
  if (bagPackagesPool.size() < *o_size) bagPackagesPool.resize(*o_size);
  StaPipBagPackage* result = bagPackagesPool.data();

  CoreBBoxFrustum coarseRoute = PARTIALLY_IN_FRUSTUM;
  const bool coarse = size == maxVertCount && hasBoxes() && objectSpacePlanes;
#if TYRA_STAPIP_PROBE_COARSE_CLASSIFY
  // Probe A2 (stapip_probes.hpp): the group's mask and guard-band answer,
  // derived once beside its verdict and copied to all eight packages.
  u8 coarseMask = 0;
  bool coarseGuardBandOnly = false;
#endif
  for (u16 i = 0; i < *o_size; i++) {
    if (coarse && (i & 7) == 0) {
      Vec4 coarseMin, coarseMax;
      coarseMinMax(i / 8, &coarseMin, &coarseMax);
      coarseRoute = CoreBBox::frustumCheckAABB(objectSpacePlanes, coarseMin,
                                               coarseMax, nullptr);
#if TYRA_STAPIP_PROBE_COARSE_CLASSIFY
      // Exactly the second pass checkFrustum runs, on the group box. Only a
      // partial box pays it, same as per package.
      coarseMask = 0;
      coarseGuardBandOnly = false;
      if (capturePlaneMasks && clipObjectSpacePlanes != nullptr) {
        const u8 mask =
            coarseRoute == PARTIALLY_IN_FRUSTUM
                ? CoreBBox::activePlaneMaskAABB(clipObjectSpacePlanes,
                                                coarseMin, coarseMax, 8)
                : 0;
        coarseMask = static_cast<u8>(mask & 0x3F);
        coarseGuardBandOnly = coarseRoute == PARTIALLY_IN_FRUSTUM && mask == 0;
      }
#endif
#if TYRA_STAPIP_PROBE_ACCEPT_ALL
      // A1: the coarse level rejects eight packages on one test, so disabling
      // rejection has to cover it too - otherwise most of the frame's
      // rejections survive and the arm measures almost nothing. Mapping the
      // VERDICT rather than forcing the branch keeps the EE cost identical to
      // the control: `coarseRoute != PARTIALLY_IN_FRUSTUM` still decides the
      // same way, so the same packages skip the per-package test.
      coarseRoute = probeAcceptAll(coarseRoute);
#endif
    }

    result[i].bag = data;
    // Modified by TyraX: the first vertex, and pointers into whichever float
    // arrays the bag has - a bag drawn from its compact stream may have none.
    result[i].first = i * size;
    result[i].vertices =
        data->vertices ? &data->vertices[i * size] : nullptr;

    result[i].sts = data->texture && data->texture->coordinates
                        ? &data->texture->coordinates[i * size]
                        : nullptr;

    result[i].colors =
        data->color->many
            ? reinterpret_cast<const Vec4*>(&data->color->many[i * size])
            : nullptr;

    result[i].normals = data->lighting && data->lighting->normals
                            ? &data->lighting->normals[i * size]
                            : nullptr;

    result[i].indexOf1By3BBox = (i * size) / (maxVertCount / 3);

    if (i == *o_size - 1) {
      result[i].size = data->count - i * size;
    } else {
      result[i].size = size;
    }

    // Modified by TyraX: last 1/3 bbox the package overlaps.
    result[i].endIndexOf1By3BBox =
        (i * size + result[i].size - 1) / (maxVertCount / 3);

    result[i].clipPlaneMask = 0;
    result[i].guardBandOnly = false;
#if TYRA_STAPIP_PROBE_COARSE_CLASSIFY
    // A2: the group's answer stands for every package in it, PARTIAL groups
    // included, so checkFrustum never runs on the coarse path at all.
    if (coarse) {
      result[i].isInFrustum = coarseRoute;
      result[i].clipPlaneMask = coarseMask;
      result[i].guardBandOnly = coarseGuardBandOnly;
      continue;
    }
    result[i].isInFrustum = checkFrustum(
        result[i], capturePlaneMasks ? &result[i].clipPlaneMask : nullptr,
        &result[i].guardBandOnly);
#else
    // A wholly inside/outside coarse box proves the same for every child.
    // Partial groups retain the exact per-package and guard-band tests.
    result[i].isInFrustum = coarse && coarseRoute != PARTIALLY_IN_FRUSTUM
        ? coarseRoute : checkFrustum(
        result[i], capturePlaneMasks ? &result[i].clipPlaneMask : nullptr,
        &result[i].guardBandOnly);
#endif
  }

  return result;
}

/**
 * @brief Split render package to smaller packages
 *
 * @param size Max maxVertCount verts (VU1 buffer size)
 */
StaPipBagPackage* StaPipBagPackager::create(u16* o_count,
                                            const StaPipBagPackage& pkg,
                                            u16 size) {
  HardwareTrace::Scope trace("Package_create");
  TYRA_ASSERT(size <= maxVertCount, "StaPipBagPackage can have max ",
              maxVertCount, " verts. Provided \"", size, "\"");

  *o_count = (pkg.size + size - 1) / size;
#if TYRA_STAPIP_ATTRIB
  stats.packages += *o_count;
#endif
  // Modified by TyraX: grow-only pool instead of new[] per split (see
  // the bag-level overload above; separate pool - the parent package array
  // is still alive during a split).
  if (splitPackagesPool.size() < *o_count) splitPackagesPool.resize(*o_count);
  StaPipBagPackage* result = splitPackagesPool.data();

  for (u16 i = 0; i < *o_count; i++) {
    result[i].bag = pkg.bag;
    result[i].first = pkg.first + i * size;  // Modified by TyraX: see create()
    result[i].vertices = pkg.vertices ? &pkg.vertices[i * size] : nullptr;

    result[i].sts = pkg.sts ? &pkg.sts[i * size] : nullptr;

    result[i].colors = pkg.colors ? &pkg.colors[i * size] : nullptr;

    result[i].normals = pkg.normals ? &pkg.normals[i * size] : nullptr;

    result[i].indexOf1By3BBox =
        pkg.indexOf1By3BBox + ((i * size) / (maxVertCount / 3));

    if (i == *o_count - 1) {
      result[i].size = pkg.size - i * size;
    } else {
      result[i].size = size;
    }

    // Modified by TyraX: last 1/3 bbox the subpackage overlaps (parent
    // packages always start on a 1/3 boundary).
    result[i].endIndexOf1By3BBox =
        pkg.indexOf1By3BBox +
        ((i * size + result[i].size - 1) / (maxVertCount / 3));

    result[i].clipPlaneMask = 0;
    result[i].guardBandOnly = false;
    result[i].isInFrustum = checkFrustum(
        result[i], capturePlaneMasks ? &result[i].clipPlaneMask : nullptr,
        &result[i].guardBandOnly);
  }

  return result;
}

// Modified by TyraX: classification runs against object-space planes
// (set once per bag by StaPipCore) with the p-vertex/n-vertex AABB test.
// The previous shape of this function transformed up to 8 corners of a
// (possibly freshly merged) bbox per package and dotted each corner against
// each world plane - the dominant EE cost of partially-visible geometry
// after the clipper itself.
CoreBBoxFrustum StaPipBagPackager::checkFrustum(const StaPipBagPackage& pkg,
                                                u8* crossingMask,
                                                bool* o_guardBandOnly) {
  HardwareTrace::Scope trace("Package_classify");
#if TYRA_STAPIP_ATTRIB
  const u32 attribStart = readPackagerAttribTicks();
  struct AttribClose {
    StaPipBagPackager::Stats* s;
    u32 start;
    ~AttribClose() { s->classifyTicks += readPackagerAttribTicks() - start; }
  } attribClose{&stats, attribStart};
#endif
  if (o_guardBandOnly) *o_guardBandOnly = false;
  if (!hasBoxes() || !objectSpacePlanes)
    return CoreBBoxFrustum::OUTSIDE_FRUSTUM;

  Vec4 min, max;

#if TYRA_STAPIP_ATTRIB
  // The merge loop's trip count, so "is the classification the WALK or the
  // arithmetic" stops being a guess. Counted here rather than inside
  // getMergedMinMax to keep that hot function untouched.
  stats.mergeParts +=
      pkg.size <= (maxVertCount / 3)
          ? (pkg.endIndexOf1By3BBox - pkg.indexOf1By3BBox + 1)
          : ((pkg.size + maxVertCount / 3 - 1) / (maxVertCount / 3));
#endif
  if (pkg.size <= (maxVertCount / 3)) {  // Is subpackage
    // A subpackage smaller than maxVertCount / 3 (VU1 clipping mode) can
    // straddle a 1/3 bbox boundary - classify it against the merged bbox
    // of every part it overlaps.
    mergedMinMax(pkg.indexOf1By3BBox,
                 pkg.endIndexOf1By3BBox - pkg.indexOf1By3BBox + 1, &min, &max);
  } else {  // Is package
    const auto& indexOfPart = pkg.indexOf1By3BBox;
    auto partSize = (pkg.size + maxVertCount / 3 - 1) / (maxVertCount / 3);
    mergedMinMax(indexOfPart, partSize, &min, &max);
  }

  // VU1 uses guard-band/near-margin clip planes which are not always the same
  // as the view frustum (env-map passes deliberately widen that frustum).
  // Classify against the view planes, but derive the functional mask from the
  // exact VU equations transformed to object space by StaPipCore.
  u8* frustumMask = clipObjectSpacePlanes == nullptr ? crossingMask : nullptr;
  const CoreBBoxFrustum result =
      CoreBBox::frustumCheckAABB(objectSpacePlanes, min, max, frustumMask);
  if (crossingMask != nullptr && clipObjectSpacePlanes != nullptr) {
    // Modified by TyraX: eight planes, not six. Bits 0..5 are the mask VU1
    // gets; bits 6..7 are the exact near/far pair, and a package that is
    // inside all eight needs no clipping (StaPipCore::isGuardBandOnly).
    // Both are answered by one pass over the same box.
#if TYRA_STAPIP_ATTRIB
    if (result == PARTIALLY_IN_FRUSTUM) ++stats.maskCalls;
#endif
    const u8 mask =
        result == PARTIALLY_IN_FRUSTUM
            ? CoreBBox::activePlaneMaskAABB(clipObjectSpacePlanes, min, max, 8)
            : 0;
    *crossingMask = static_cast<u8>(mask & 0x3F);
    if (o_guardBandOnly)
      *o_guardBandOnly = result == PARTIALLY_IN_FRUSTUM && mask == 0;
  }
#if TYRA_STAPIP_PROBE_ACCEPT_ALL
  // A1: every test above ran and every output it wrote stands. Only the
  // verdict's OUTSIDE case is discarded, which sends the package to the cull
  // route with a zero mask - the same route a wholly-inside package takes.
  return probeAcceptAll(result);
#else
  return result;
#endif
}

void StaPipBagPackager::setMaxVertCount(const u32& count) {
  maxVertCount = count;
}

}  // namespace Tyra
