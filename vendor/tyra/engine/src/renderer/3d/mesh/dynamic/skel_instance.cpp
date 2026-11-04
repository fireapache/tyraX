/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: per-object skeletal playback; pose evaluation on
# the EE, vertex skinning on VU0 in macro mode (COP2 inline asm).
# The sampling/pose math mirrors the editor's src/glbparser.cpp (the
# viewport preview and the stage-1 baker) - keep the formulas in sync so
# what the editor shows is what the console computes. Two parts do NOT keep
# bit-parity, both for VU0 rounding (~1 ulp against the EE FPU): the vertex
# loop, verified by screenshot parity against the EE loop, and (TyraX fork)
# the pose's matrix products (mulM4Vu0), verified by the golden-vertex gyms.
*/

#include "renderer/3d/mesh/dynamic/skel_instance.hpp"

#include <float.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <unordered_map>
#include <string>
#include <new>
#include "debug/debug.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "renderer/core/paths/path1/vif1_queue.hpp"

#if TYRA_SKEL_PROFILE
static unsigned skelTicks() { unsigned t; asm volatile("mfc0 %0, $9" : "=r"(t)); return t; }
#endif

#include "loaders/3d/builder/mesh_builder_data.hpp"
#if TYRA_MEMORY_MANAGER
#include "memory/manager.hpp"
#endif

namespace Tyra {

namespace {

/** Column-major r = a * b on VU0 in macro mode, for 16-byte aligned operands
 *  (r may alias a or b - every load lands before a store). A pose does 78 of
 *  these (41 nodes + 37 palette slots on the combatent): as a scalar triple
 *  loop on the EE they were ~60% of evalPose, about 2 us each - 0.18 of a
 *  0.29 ms pose (docs/animated-models.md, "What a pose costs"). The sequence
 *  is M4x4::cross's, which is private. */
void mulM4Vu0(float* r, const float* a, const float* b) {
  asm volatile(
      "lqc2         $vf1, 0x00(%1) \n\t"
      "lqc2         $vf2, 0x10(%1) \n\t"
      "lqc2         $vf3, 0x20(%1) \n\t"
      "lqc2         $vf4, 0x30(%1) \n\t"
      "lqc2         $vf5, 0x00(%2) \n\t"
      "lqc2         $vf6, 0x10(%2) \n\t"
      "lqc2         $vf7, 0x20(%2) \n\t"
      "lqc2         $vf8, 0x30(%2) \n\t"
      "vmulax.xyzw  $ACC, $vf5, $vf1 \n\t"
      "vmadday.xyzw $ACC, $vf6, $vf1 \n\t"
      "vmaddaz.xyzw $ACC, $vf7, $vf1 \n\t"
      "vmaddw.xyzw  $vf1, $vf8, $vf1 \n\t"
      "vmulax.xyzw  $ACC, $vf5, $vf2 \n\t"
      "vmadday.xyzw $ACC, $vf6, $vf2 \n\t"
      "vmaddaz.xyzw $ACC, $vf7, $vf2 \n\t"
      "vmaddw.xyzw  $vf2, $vf8, $vf2 \n\t"
      "vmulax.xyzw  $ACC, $vf5, $vf3 \n\t"
      "vmadday.xyzw $ACC, $vf6, $vf3 \n\t"
      "vmaddaz.xyzw $ACC, $vf7, $vf3 \n\t"
      "vmaddw.xyzw  $vf3, $vf8, $vf3 \n\t"
      "vmulax.xyzw  $ACC, $vf5, $vf4 \n\t"
      "vmadday.xyzw $ACC, $vf6, $vf4 \n\t"
      "vmaddaz.xyzw $ACC, $vf7, $vf4 \n\t"
      "vmaddw.xyzw  $vf4, $vf8, $vf4 \n\t"
      "sqc2         $vf1, 0x00(%0) \n\t"
      "sqc2         $vf2, 0x10(%0) \n\t"
      "sqc2         $vf3, 0x20(%0) \n\t"
      "sqc2         $vf4, 0x30(%0) \n\t"
      :
      : "r"(r), "r"(b), "r"(a)
      : "memory");
}

/** T * R * S from translation / quaternion / scale (glTF conventions). */
void fromTrs(float* m, const float* t, const float* q, const float* s) {
  const float x = q[0], y = q[1], z = q[2], w = q[3];
  const float x2 = x + x, y2 = y + y, z2 = z + z;
  const float xx = x * x2, xy = x * y2, xz = x * z2;
  const float yy = y * y2, yz = y * z2, zz = z * z2;
  const float wx = w * x2, wy = w * y2, wz = w * z2;
  m[0] = (1.0F - (yy + zz)) * s[0];
  m[1] = (xy + wz) * s[0];
  m[2] = (xz - wy) * s[0];
  m[3] = 0.0F;
  m[4] = (xy - wz) * s[1];
  m[5] = (1.0F - (xx + zz)) * s[1];
  m[6] = (yz + wx) * s[1];
  m[7] = 0.0F;
  m[8] = (xz + wy) * s[2];
  m[9] = (yz - wx) * s[2];
  m[10] = (1.0F - (xx + yy)) * s[2];
  m[11] = 0.0F;
  m[12] = t[0];
  m[13] = t[1];
  m[14] = t[2];
  m[15] = 1.0F;
}

void slerp(const float* a, const float* bIn, float t, float* out) {
  float b[4] = {bIn[0], bIn[1], bIn[2], bIn[3]};
  float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0.0F) {  // take the short arc
    dot = -dot;
    for (int i = 0; i < 4; ++i) b[i] = -b[i];
  }
  float wa, wb;
  if (dot > 0.9995F) {  // nearly parallel - lerp avoids a degenerate sin
    wa = 1.0F - t;
    wb = t;
  } else {
    const float theta = acosf(dot > 1.0F ? 1.0F : dot);
    const float sinTheta = sinf(theta);
    wa = sinf((1.0F - t) * theta) / sinTheta;
    wb = sinf(t * theta) / sinTheta;
  }
  for (int i = 0; i < 4; ++i) out[i] = wa * a[i] + wb * b[i];
  const float len = sqrtf(out[0] * out[0] + out[1] * out[1] +
                          out[2] * out[2] + out[3] * out[3]);
  if (len > 1e-6F)
    for (int i = 0; i < 4; ++i) out[i] /= len;
  else
    out[3] = 1.0F;
}

/** First key index with times[hi] >= t; the cursor amortizes the scan to
 * O(1) since playback time only moves forward between resets. */
u32 findKey(const std::vector<float>& times, float t, u32* cursor) {
  const u32 n = times.size();
  u32 hi = *cursor;
  if (hi > n) hi = n;
  while (hi > 0 && times[hi - 1] >= t) --hi;  // rewind (restart safety)
  while (hi < n && times[hi] < t) ++hi;
  *cursor = hi;
  return hi;
}

/** Samples one channel at time t into out[3|4] - same rules as the editor:
 * clamp outside the key range, STEP holds the left key, rotations slerp. */
void sampleChannel(const SkelChannel& ch, float t, u32* cursor, float* out) {
  const u32 n = ch.times.size();
  const u32 hi = findKey(ch.times, t, cursor);

  if (ch.path == 1) {
    float a[4], b[4];
    const u32 lo = hi == 0 ? 0 : hi - 1;
    const u32 hiC = hi >= n ? n - 1 : hi;
    for (int c = 0; c < 4; ++c) {
      a[c] = ch.quat[(size_t)lo * 4 + c] * (1.0F / 32767.0F);
      b[c] = ch.quat[(size_t)hiC * 4 + c] * (1.0F / 32767.0F);
    }
    if (hi == 0 || hi >= n || lo == hiC) {
      memcpy(out, hi == 0 ? a : b, 4 * sizeof(float));
      // still normalize the dequantized endpoint
      const float len = sqrtf(out[0] * out[0] + out[1] * out[1] +
                              out[2] * out[2] + out[3] * out[3]);
      if (len > 1e-6F)
        for (int i = 0; i < 4; ++i) out[i] /= len;
      return;
    }
    const float t0 = ch.times[lo], t1 = ch.times[hi];
    float f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0F;
    if (ch.step) f = 0.0F;
    slerp(a, b, f, out);
    return;
  }

  if (hi == 0) {
    memcpy(out, &ch.vec[0], 3 * sizeof(float));
    return;
  }
  if (hi >= n) {
    memcpy(out, &ch.vec[(size_t)(n - 1) * 3], 3 * sizeof(float));
    return;
  }
  const u32 lo = hi - 1;
  const float t0 = ch.times[lo], t1 = ch.times[hi];
  float f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0F;
  if (ch.step) f = 0.0F;
  const float* a = &ch.vec[(size_t)lo * 3];
  const float* b = &ch.vec[(size_t)hi * 3];
  for (int c = 0; c < 3; ++c) out[c] = a[c] + (b[c] - a[c]) * f;
}

}  // namespace

namespace {

/** Repacks one mesh variant's bind data for the VU0 loop (see the header
 * comment on SkelBindLod): aligned qwords, normalized weights, joints sorted
 * by descending weight, influence counts for the dispatch. */
void repackBind(const float* positions, const float* normals, const u8* joints,
                const u8* weights, u32 count, SkelBindLod& pl) {
  pl.count = count;
  pl.bindPositions.resize(count);
  pl.bindNormals.resize(count);
  pl.skinWeights.resize(count);
  pl.sortedJoints.resize((size_t)count * 4);
  pl.influences.resize(count);
  pl.skinSource.resize(count);
  // Exact bind attributes only: UV seams can share skinning, hard normals
  // and different bone weights cannot. This temporary table dies at load.
  std::unordered_map<std::string, u32> firstCorner;
  firstCorner.reserve(count);
  for (u32 v = 0; v < count; v++) {
    pl.bindPositions[v].set(positions[(size_t)v * 3],
                            positions[(size_t)v * 3 + 1],
                            positions[(size_t)v * 3 + 2], 1.0F);
    pl.bindNormals[v].set(normals[(size_t)v * 3], normals[(size_t)v * 3 + 1],
                          normals[(size_t)v * 3 + 2], 0.0F);
    const u8* jj = &joints[(size_t)v * 4];
    const u8* ww = &weights[(size_t)v * 4];
    u8 idx[4] = {0, 1, 2, 3};  // slot order by descending weight
    for (int a = 1; a < 4; a++)
      for (int b = a; b > 0 && ww[idx[b]] > ww[idx[b - 1]]; b--) {
        const u8 t = idx[b];
        idx[b] = idx[b - 1];
        idx[b - 1] = t;
      }
    const u32 wsum = (u32)ww[0] + ww[1] + ww[2] + ww[3];
    const float inv = wsum > 0 ? 1.0F / (float)wsum : 0.0F;
    float wf[4];
    u8 n = 0;
    for (int k = 0; k < 4; k++) {
      wf[k] = (float)ww[idx[k]] * inv;
      pl.sortedJoints[(size_t)v * 4 + k] = jj[idx[k]];
      if (ww[idx[k]] > 0) n++;
    }
    pl.skinWeights[v].set(wf[0], wf[1], wf[2], wf[3]);
    pl.influences[v] = n;
    char key[32];
    memcpy(key, positions + (size_t)v * 3, 12);
    memcpy(key + 12, normals + (size_t)v * 3, 12);
    memcpy(key + 24, jj, 4);
    memcpy(key + 28, ww, 4);
    const auto inserted = firstCorner.emplace(std::string(key, sizeof(key)), v);
    pl.skinSource[v] = inserted.first->second;
  }
}

/** The influences of a level's corners (Fixed::prepCorner), in corner order. */
std::vector<SkelSkin::Fixed::CornerInfl> prepInfl(const u8* joints, const u8* weights,
                                                  u32 count) {
  std::vector<SkelSkin::Fixed::CornerInfl> out(count);
  for (u32 v = 0; v < count; v++)
    out[v] = SkelSkin::Fixed::prepCorner(joints + 4 * (size_t)v, weights + 4 * (size_t)v);
  return out;
}

/** A level's unique corners as a level of their own (SkelBindLod::uniq): each corner whose
 * skinSource is itself, in corner order, with its bind data copied and the skin source the
 * identity, so the VU0 corner loop blends the uniques alone. */
void buildUniqueLod(SkelBindLod& bl) {
  auto u = std::make_shared<SkelBindLod>();
  std::vector<u32> corner;
  for (u32 v = 0; v < bl.count; v++) {
    if (bl.skinSource[v] != v) continue;
    u->bindPositions.push_back(bl.bindPositions[v]);
    u->bindNormals.push_back(bl.bindNormals[v]);
    u->skinWeights.push_back(bl.skinWeights[v]);
    u->sortedJoints.insert(u->sortedJoints.end(), bl.sortedJoints.begin() + 4 * (size_t)v,
                           bl.sortedJoints.begin() + 4 * (size_t)v + 4);
    u->influences.push_back(bl.influences[v]);
    corner.push_back(v);
  }
  u->count = (u32)u->bindPositions.size();
  u->skinSource.resize(u->count);
  for (u32 i = 0; i < u->count; i++) u->skinSource[i] = i;
  u->uniqCorner = std::move(corner);
  bl.uniq = std::move(u);
}

/** Every part's chain, built once per model: level 0 = the full mesh, then
 * the baked (generated or authored) LOD variants. */
std::shared_ptr<const SkelBindCache> buildBindCache(const SkelModel& model) {
  auto cache = std::make_shared<SkelBindCache>();
  cache->parts.resize(model.parts.size());
  for (size_t pi = 0; pi < model.parts.size(); pi++) {
    const SkelPart& part = model.parts[pi];
    auto& chain = cache->parts[pi];
    chain.resize(1 + part.lods.size());
    // The bind data is built from the compact meshes, decoded once here.
    std::vector<float> pos((size_t)part.vertexCount * 3);
    std::vector<float> nrm((size_t)part.vertexCount * 3);
    decodeMesh(model.box, part, pos.data(), nrm.data(), nullptr);
    repackBind(pos.data(), nrm.data(), part.joints.data(), part.weights.data(),
               part.vertexCount, chain[0]);
    chain[0].infl = prepInfl(part.joints.data(), part.weights.data(), part.vertexCount);
    SkelSkin::bindExtent(part.pos.data(), part.joints.data(), part.weights.data(),
                         part.vertexCount, chain[0].extent);
    buildUniqueLod(chain[0]);
    for (size_t l = 0; l < part.lods.size(); l++) {
      const SkelLod& src = part.lods[l];
      SkelBindLod& bl = chain[1 + l];
      std::vector<float> lpos((size_t)src.vertexCount * 3);
      std::vector<float> lnrm((size_t)src.vertexCount * 3);
      std::vector<float> luv(part.texturePath.empty() ? 0 : (size_t)src.vertexCount * 2);
      decodeMesh(model.box, src, lpos.data(), lnrm.data(),
                 luv.empty() ? nullptr : luv.data());
      repackBind(lpos.data(), lnrm.data(), src.joints.data(), src.weights.data(),
                 src.vertexCount, bl);
      bl.infl = prepInfl(src.joints.data(), src.weights.data(), src.vertexCount);
      SkelSkin::bindExtent(src.pos.data(), src.joints.data(), src.weights.data(),
                           src.vertexCount, bl.extent);
      buildUniqueLod(bl);
      if (!luv.empty()) {
        bl.uvs.resize(src.vertexCount);
        for (u32 v = 0; v < src.vertexCount; v++)
          bl.uvs[v].set(luv[(size_t)v * 2], luv[(size_t)v * 2 + 1], 1.0F, 0.0F);
      }
    }
  }
  return cache;
}

/** A level > 0 gets its skin output the first time it is skinned - an
 * instance never seen at that distance never pays for the buffers. They are
 * taken with new (std::nothrow): false, with nothing changed, when the heap
 * has no room for them - the caller then draws a level the instance holds
 * (heldLevel) and asks again the next time it skins. */
bool ensureOutput(SkelInstance::PartLod& pl) {
  if (pl.outV) return true;
#if TYRA_MEMORY_MANAGER
  // Added by TyraX: a level's skin output is the Skinning budget's, as the
  // buffers the constructor makes are (docs/memory-manager.md). Skinning
  // refuses when full: the buffers are null and the part draws a level it
  // holds.
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  std::unique_ptr<Vec4[]> vertices(new (std::nothrow) Vec4[pl.count]);
  std::unique_ptr<Vec4[]> normals(new (std::nothrow) Vec4[pl.count]);
  if (!vertices || !normals) return false;
  for (u32 v = 0; v < pl.count; v++) {  // bind pose until the first skin
    vertices[v] = pl.bind->bindPositions[v];
    normals[v] = pl.bind->bindNormals[v];
  }
  pl.ownVertices = std::move(vertices);
  pl.ownNormals = std::move(normals);
  pl.outV = pl.ownVertices.get();
  pl.outN = pl.ownNormals.get();
  return true;
}

/** The level a part draws for a wanted `lod` (clamped to its chain): that
 * level when it holds a skin output - made now when `make` - else the
 * cheapest level it holds one for. Level 0's output is the mesh's own, so
 * there always is one. Skinning (make) and reading (not) answer the same
 * level, since only skinning makes outputs. */
u8 heldLevel(std::vector<SkelInstance::PartLod>& chain, u8 lod, bool make) {
  if (lod >= chain.size()) lod = (u8)(chain.size() - 1);
  if (chain[lod].outV != nullptr || (make && ensureOutput(chain[lod])))
    return lod;
  for (u8 l = (u8)(chain.size() - 1); l > 0; l--)
    if (chain[l].outV != nullptr) return l;
  return 0;
}

}  // namespace

SkelInstance::SkelInstance(const SkelModel* t_model) : model(t_model) {
#if TYRA_MEMORY_MANAGER
  // Added by TyraX: what an instance allocates here - its mesh, its skin
  // output and every LOD's buffers - is the Skinning budget's
  // (docs/memory-manager.md).
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  // Single-frame builder data in bind pose; the DynamicMesh takes ownership
  // of the arrays, we keep the pointers and overwrite them every update.
  MeshBuilderData data;
  data.loadNormals = true;
  data.loadLightmap = false;
  if (!model->bindCache) model->bindCache = buildBindCache(*model);
  const SkelBindCache& cache = *model->bindCache;
  partLods.resize(model->parts.size());
  partLevel.assign(model->parts.size(), 0);
  for (size_t pi = 0; pi < model->parts.size(); pi++) {
    const SkelPart& part = model->parts[pi];
    auto* material = new MeshBuilderMaterialData();
    data.materials.push_back(material);
    material->name = part.name;
    if (!part.texturePath.empty()) material->texturePath = part.texturePath;
    // the single color the VU1 programs modulate with (128 = 1.0)
    material->ambient.set(part.color[0] * 128.0F, part.color[1] * 128.0F,
                          part.color[2] * 128.0F, 128.0F);

    auto* frame = new MeshBuilderMaterialFrameData();
    material->frames.push_back(frame);
    frame->count = part.vertexCount;
    frame->vertices = new Vec4[part.vertexCount];
    frame->normals = new Vec4[part.vertexCount];
    // the bind pose, decoded from the compact mesh (the model's box)
    std::vector<float> pos((size_t)part.vertexCount * 3);
    std::vector<float> nrm((size_t)part.vertexCount * 3);
    std::vector<float> uv(part.texturePath.empty() ? 0 : (size_t)part.vertexCount * 2);
    decodeMesh(model->box, part, pos.data(), nrm.data(),
               uv.empty() ? nullptr : uv.data());
    for (u32 v = 0; v < part.vertexCount; v++) {
      frame->vertices[v].set(pos[(size_t)v * 3], pos[(size_t)v * 3 + 1],
                             pos[(size_t)v * 3 + 2], 1.0F);
      frame->normals[v].set(nrm[(size_t)v * 3], nrm[(size_t)v * 3 + 1],
                            nrm[(size_t)v * 3 + 2], 1.0F);
    }
    if (!part.texturePath.empty()) {
      frame->textureCoords = new Vec4[part.vertexCount];
      for (u32 v = 0; v < part.vertexCount; v++)
        frame->textureCoords[v].set(uv[(size_t)v * 2], uv[(size_t)v * 2 + 1],
                                    1.0F, 0.0F);
    }

    // level 0 = the full mesh, skinning straight into the frame arrays
    auto& chain = partLods[pi];
    const auto& shared = cache.parts[pi];
    chain.resize(shared.size());
    for (size_t l = 0; l < shared.size(); l++) {
      chain[l].bind = &shared[l];
      chain[l].count = shared[l].count;
    }
    chain[0].outV = frame->vertices;
    chain[0].outN = frame->normals;
    chain[0].uvPtr = frame->textureCoords;  // nullptr when untextured

    // deeper levels: baked decimated variants - their own skin buffers
    // (ensureOutput, on first use), the shared packed UVs
    for (size_t l = 1; l < chain.size(); l++)
      if (!part.texturePath.empty() && !shared[l].uvs.empty())
        chain[l].uvPtr = const_cast<Vec4*>(shared[l].uvs.data());
    if (chain.size() > maxLodLevels) maxLodLevels = (u8)chain.size();
  }
  mesh = std::make_unique<DynamicMesh>(&data);

  localsCur.resize(model->nodes.size() * 10);
  localsPrev.resize(model->nodes.size() * 10);
  animatedCur.resize(model->nodes.size());
  animatedPrev.resize(model->nodes.size());
  globals.resize(model->nodes.size());
  palette.resize(model->palette.size());
  // Blend-space sample layers switch clips at runtime: their cursor arrays
  // are reserved for the longest clip now, so a switch never allocates.
  size_t maxChannels = 0, maxSamples = 0;
  for (const SkelClip& c : model->clips)
    if (c.channels.size() > maxChannels) maxChannels = c.channels.size();
  for (const SkelBlendSpace& b : model->blendSpaces)
    if (b.samples.size() > maxSamples) maxSamples = b.samples.size();
  if (maxSamples > 0) {
    for (Layer& l : bs) l.cursors.reserve(maxChannels);
    // the frozen pose a mode or blend-LOD switch fades from (~1.7 KB for 41
    // nodes): sized now, so no switch allocates mid-game
    localsFrom.resize(localsCur.size());
    animatedFrom.resize(animatedCur.size());
  }
  bsOwnClock.assign(maxSamples, 0.0F);

  play(0, true, 0.0F);
}

SkelInstance::~SkelInstance() {}

size_t SkelInstance::bytesFor(const SkelModel& model) {
  // Per allocation, a heap header and alignment slack; per part, the builder
  // material and frame, the mesh's material, frame and boxes, the names.
  const size_t kPerPart = 1024;
  const size_t kFixed = 8 * 1024;  // the mesh, its frame, a log line, vectors
  size_t bytes = kFixed;
  size_t largestLevel = 0;  // the most vertices one level of one part has
  for (const SkelPart& part : model.parts) {
    const size_t arrays = part.texturePath.empty() ? 2 : 3;
    bytes += (size_t)part.vertexCount * sizeof(Vec4) * arrays + kPerPart;
    if (!model.bindCache) {
      // The repacked bind data of every level (SkelBindLod): positions,
      // normals and weights as Vec4s, joints, influences, the skin source
      // index - and packed UVs for the levels past the full mesh.
      size_t vertices = part.vertexCount;
      for (const SkelLod& lod : part.lods) vertices += lod.vertexCount;
      bytes += vertices * (3 * sizeof(Vec4) + 4 + 1 + 4 + sizeof(Vec4)) +
               (part.lods.size() + 1) * kPerPart;
      if (part.vertexCount > largestLevel) largestLevel = part.vertexCount;
      for (const SkelLod& lod : part.lods)
        if (lod.vertexCount > largestLevel) largestLevel = lod.vertexCount;
    }
  }
  if (!model.bindCache) {
    // Added by TyraX: and while it is built, repackBind's temporary table of
    // first corners - one level at a time, so the largest level's: per vertex
    // a hash node (the next pointer, the 32-byte key string, the index, the
    // cached hash) and the key's own 33-byte buffer, two blocks each with a
    // 16-byte heap header and rounded to 16, and a bucket. 132 bytes a vertex
    // on the EE (a 36-byte node and a 33-byte string served as 64 each, a
    // 4-byte bucket), 152 on a 64-bit host - as measured.
    auto served = [](size_t n) { return (n + 16 + 15) / 16 * 16; };
    const size_t node = sizeof(void*) +
                        sizeof(std::pair<const std::string, u32>) +
                        sizeof(size_t);
    bytes += largestLevel * (served(node) + served(32 + 1) + sizeof(void*));
  }
  const size_t nodes = model.nodes.size();
  // locals cur / prev (10 floats each), animated cur / prev, globals.
  bytes += nodes * (2 * 10 * sizeof(float) + 2 + sizeof(M4x4));
  bytes += model.palette.size() * sizeof(M4x4);
  size_t channels = 0;
  for (const SkelClip& clip : model.clips)
    if (clip.channels.size() > channels) channels = clip.channels.size();
  bytes += channels * sizeof(u32) * 2;  // the playing and the fading layer
  // Added by TyraX (blend spaces): a model with blend spaces also reserves
  // its sample layers' cursors, the frozen pose a switch fades from and a
  // clock per sample.
  size_t samples = 0;
  for (const SkelBlendSpace& b : model.blendSpaces)
    if (b.samples.size() > samples) samples = b.samples.size();
  if (samples > 0)
    bytes += 3 * channels * sizeof(u32) + nodes * (10 * sizeof(float) + 1) +
             samples * sizeof(float) + 4 * kPerPart;
  return bytes;
}

std::unique_ptr<SkelInstance> SkelInstance::create(const SkelModel* model) {
  if (model == nullptr) return nullptr;
  const size_t bytes = bytesFor(*model);
  {
#if TYRA_MEMORY_MANAGER
    // Added by TyraX: the probe asks the budget the constructor's allocations
    // are charged to (Skinning, which refuses when full), so a probe that
    // passes means the constructor's own requests are not refused either.
    Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
    // The room the constructor will take, a quarter and 16 KB over: refused,
    // nothing is made. A call to operator new, not a new-expression: a
    // compiler may drop a new-expression whose block is never used - and
    // fold its null check to "not null" - so that probe would always pass.
    void* room = ::operator new[](bytes + bytes / 4 + 16 * 1024, std::nothrow);
    if (room == nullptr) {
#if TYRA_MEMORY_MANAGER
      // Added by TyraX: say so - once a second per model (the key).
      Memory::Manager::instance().noteFallback(
          "skinned instance: no body yet",
          model->parts.empty() ? nullptr : model->parts[0].name.c_str(), 0,
          model);
#endif
      return nullptr;
    }
    ::operator delete[](room);
  }
  return std::unique_ptr<SkelInstance>(new SkelInstance(model));
}

void SkelInstance::play(u32 clip, bool loop, float fadeSeconds) {
  if (clip >= model->clips.size()) clip = 0;
  if (model->clips.empty()) return;

  if (bsSpace >= 0) {
    // Leaving blend-space mode: the samples are not one layer prev could
    // keep, so the fade starts from the frozen blended pose instead.
    if (fadeSeconds > 0.0F) freezeFrom(fadeSeconds);
    bsSpace = -1;
    bsCount = 0;
    prev.clip = -1;
    fadeT = 1.0F;
  } else if (fadeSeconds > 0.0F && cur.clip >= 0) {
    prev = cur;  // keeps its own time/loop/cursors alive during the fade
    fadeDuration = fadeSeconds;
    fadeT = 0.0F;
  } else {
    prev.clip = -1;
    fadeT = 1.0F;
  }
  cur.clip = (s32)clip;
  cur.time = 0.0F;
  cur.loop = loop;
  cur.held = false;
  cur.cursors.assign(model->clips[clip].channels.size(), 0);
  oneShotDone = false;
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::seek(float seconds) {
  if (cur.clip < 0) return;
  const SkelClip& clip = model->clips[cur.clip];
  if (seconds < 0.0F) seconds = 0.0F;
  if (seconds > clip.duration) seconds = clip.duration;
  cur.held = true;  // a held clip keeps its place while it fades out, too
  if (seconds == cur.time) return;
  cur.time = seconds;  // findKey rewinds its cursors on a backward jump
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::setBlend(s32 clip, float seconds, float weight) {
  if (clip < 0 || clip >= (s32)model->clips.size() || weight <= 0.0F) {
    if (aux.clip >= 0) poseDirty = globalsStale = clipStale = true;  // the layer just switched off
    aux.clip = -1;
    auxWeight = 0.0F;
    return;
  }
  const SkelClip& c = model->clips[clip];
  if (seconds < 0.0F) seconds = 0.0F;
  if (seconds > c.duration) seconds = c.duration;
  if (weight > 1.0F) weight = 1.0F;
  if (aux.clip != clip) {
    aux.clip = clip;
    aux.loop = false;
    aux.cursors.assign(c.channels.size(), 0);
    poseDirty = globalsStale = clipStale = true;
  }
  if (aux.time != seconds || auxWeight != weight) poseDirty = globalsStale = clipStale = true;
  aux.time = seconds;
  auxWeight = weight;
}

void SkelInstance::advanceLayer(Layer& layer, float dt) {
  if (layer.clip < 0) return;
  const SkelClip& clip = model->clips[layer.clip];
  if (clip.duration <= 0.0F) return;
  layer.time += dt;
  if (layer.time >= clip.duration) {
    if (layer.loop) {
      layer.time = fmodf(layer.time, clip.duration);
      for (u32& c : layer.cursors) c = 0;
      if (&layer == &cur) oneShotDone = false;  // fresh end if loop turns off
    } else {
      layer.time = clip.duration;
    }
  }
}

void SkelInstance::advanceFade(float dt) {
  advanceFrom(dt);
  if (dt <= 0.0F || fadeT >= 1.0F) return;
  if (!prev.held) advanceLayer(prev, dt);  // the fading-out clip keeps moving
  fadeT += fadeDuration > 0.0F ? dt / fadeDuration : 1.0F;
  if (fadeT >= 1.0F) {
    fadeT = 1.0F;
    prev.clip = -1;
  }
  poseDirty = globalsStale = clipStale = true;
}

bool SkelInstance::advance(float dt) {
  bool finished = false;
  if (cur.clip >= 0 && dt > 0.0F) {
    const SkelClip& clip = model->clips[cur.clip];
    if (clip.duration > 0.0F) {
      // the animFinished contract: once for one-shots, every wrap for loops
      const float next = cur.time + dt;
      if (next >= clip.duration)
        finished = cur.loop ? true : !oneShotDone;
      if (!cur.loop && next >= clip.duration) oneShotDone = true;
      advanceLayer(cur, dt);
      poseDirty = globalsStale = clipStale = true;
    }
    advanceFade(dt);
  }
  return finished;
}

bool SkelInstance::ensurePose(u8 lod) {
  if (lod >= maxLodLevels) lod = maxLodLevels - 1;
  // An overridden pose changes only when the game writes a new one - the
  // clips' bookkeeping (advance/seek) does not dirty it.
  const bool dirty = overridden ? overrideDirty : poseDirty;
  if (!dirty && lod == lastSkinnedLod) return false;
  // a pure LOD switch reuses the current palette - only the skin reruns
#if TYRA_SKEL_PROFILE
  const u32 t0 = skelTicks();
#endif
  if (dirty) evalPose();
#if TYRA_SKEL_PROFILE
  const u32 t1 = skelTicks();
#endif
  skinParts(lod);
#if TYRA_SKEL_PROFILE
  const u32 t2 = skelTicks();
  profilePose += t1 - t0;
  profileSkin += t2 - t1;
  if (++profileCount == 100) {
    char msg[192];
    snprintf(msg, sizeof(msg), "SKELTIME instance=%p nodes=%u lod=%u pose=%.3f skin=%.3f ms",
             this, (unsigned)model->nodes.size(), lod,
             profilePose / 29491200.0, profileSkin / 29491200.0);
    TYRA_LOG(msg);
    profileCount = profilePose = profileSkin = 0;
  }
#endif
  poseDirty = false;
  overrideDirty = false;
  lastSkinnedLod = lod;
  return true;
}

bool SkelInstance::update(float dt) {
  const bool finished = advance(dt);
  ensurePose(lastSkinnedLod);
  return finished;
}

s32 SkelInstance::findNode(const char* name) const {
  if (!name || !name[0]) return -1;
  for (size_t i = 0; i < model->nodes.size(); i++)
    if (model->nodes[i].name == name) return (s32)i;
  return -1;
}

size_t SkelInstance::nodeCount() const { return model ? model->nodes.size() : 0; }

void SkelInstance::adoptPose(const SkelInstance& owner) {
  if (&owner == this || owner.model != model || !owner.posedOnce ||
      owner.globals.size() != globals.size())
    return;
  globals = owner.globals;
  posedOnce = true;
  globalsStale = false;  // these ARE the pose it is drawn with
}

bool SkelInstance::setPoseOverride(const M4x4* nodeGlobals, size_t count) {
  if (!nodeGlobals || !model || count != model->nodes.size()) return false;
  if (!overridden) {
    overrideGlobals.resize(count);
    overridden = true;
    overrideDirty = true;
  } else if (!overrideDirty &&
             memcmp(overrideGlobals.data(), nodeGlobals, count * sizeof(M4x4)) != 0) {
    overrideDirty = true;
  }
  memcpy(static_cast<void*>(overrideGlobals.data()), nodeGlobals,
         count * sizeof(M4x4));
  if (overrideDirty) globalsStale = true;
  return true;
}

void SkelInstance::clearPoseOverride() {
  if (!overridden) return;
  overridden = false;
  overrideDirty = false;
  poseDirty = globalsStale = clipStale = true;  // back to the clips' pose
}

bool SkelInstance::nodeGlobal(s32 node, M4x4* out) const {
  if (!posedOnce || node < 0 || (size_t)node >= globals.size()) return false;
  *out = globals[node];
  return true;
}

bool SkelInstance::currentNodeGlobal(s32 node, M4x4* out) {
  if (node < 0 || (size_t)node >= globals.size()) return false;
  if (!posedOnce || (following && globalsStale)) evalPose();
  return nodeGlobal(node, out);
}

bool SkelInstance::vertexModel(const float bindPos[3], float out[3],
                               float maxDist) {
  // The bind cache holds every part's bind positions (the constructor built it
  // from the compact meshes), so the scan reads floats and allocates nothing.
  const SkelBindCache* cache = model->bindCache.get();
  if (!cache) return false;
  // nearest bind vertex (duplicated corners of one vertex skin identically)
  const SkelPart* bestPart = nullptr;
  size_t bestPi = 0;
  u32 best = 0;
  float bestD2 = maxDist * maxDist;
  for (size_t pi = 0; pi < model->parts.size(); pi++) {
    const SkelPart& part = model->parts[pi];
    const std::vector<Vec4>& bindPositions = cache->parts[pi][0].bindPositions;
    for (u32 v = 0; v < part.vertexCount; v++) {
      const Vec4& q = bindPositions[v];
      const float dx = q.x - bindPos[0], dy = q.y - bindPos[1],
                  dz = q.z - bindPos[2];
      const float d2 = dx * dx + dy * dy + dz * dz;
      if (d2 <= bestD2) bestD2 = d2, bestPart = &part, bestPi = pi, best = v;
    }
  }
  if (!bestPart) return false;
  // The current pose; ensurePose re-evaluates it (poseDirty stays as is), so
  // the renderer's skinning is unaffected.
  if (poseDirty || !posedOnce) evalPose();
  const Vec4& bestBind = cache->parts[bestPi][0].bindPositions[best];
  const float p[3] = {bestBind.x, bestBind.y, bestBind.z};
  const u8* j = &bestPart->joints[(size_t)best * 4];
  const u8* w = &bestPart->weights[(size_t)best * 4];
  float sum = 0.0F;
  out[0] = out[1] = out[2] = 0.0F;
  for (int k = 0; k < 4; k++) {
    if (!w[k]) continue;
    const float* m = palette[j[k]].data;  // column-major
    const float wk = (float)w[k];
    for (int r = 0; r < 3; r++)
      out[r] += wk * (m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r]);
    sum += wk;
  }
  if (sum <= 0.0F) return false;
  for (int r = 0; r < 3; r++) out[r] /= sum;
  return true;
}

SkelInstance::LodArrays SkelInstance::lodArrays(size_t part, u8 lod) {
  auto& chain = partLods[part];
  // The level skinParts skinned for this request - the wanted one, or the
  // cheapest held when the heap had no room for the wanted one's output.
  lod = heldLevel(chain, lod, false);
  PartLod& pl = chain[lod];
  return {pl.outV, pl.outN, pl.uvPtr, pl.count, lod};
}

void SkelInstance::evalLocals(Layer& layer, std::vector<float>& locals,
                              std::vector<u8>& animated) {
  const size_t nodeCount = model->nodes.size();
  for (size_t i = 0; i < nodeCount; i++) {
    const SkelNode& node = model->nodes[i];
    float* l = &locals[i * 10];
    memcpy(l, node.t, 3 * sizeof(float));
    memcpy(l + 3, node.r, 4 * sizeof(float));
    memcpy(l + 7, node.s, 3 * sizeof(float));
    animated[i] = 0;
  }
  if (layer.clip < 0) return;
  const SkelClip& clip = model->clips[layer.clip];
  for (size_t c = 0; c < clip.channels.size(); c++) {
    const SkelChannel& ch = clip.channels[c];
    float* l = &locals[(size_t)ch.node * 10];
    float* dst = ch.path == 0 ? l : (ch.path == 1 ? l + 3 : l + 7);
    sampleChannel(ch, layer.time, &layer.cursors[c], dst);
    animated[ch.node] = 1;
  }
}

void SkelInstance::blendLocals(const std::vector<float>& from,
                               const std::vector<u8>& fromAnimated, float w) {
  // Nodes only one clip animates blend against the other pose's bind
  // values, which is what a full-pose blend means.
  const size_t nodeCount = model->nodes.size();
  for (size_t i = 0; i < nodeCount; i++) {
    float* a = &localsCur[i * 10];    // blend target (in place), weight w
    const float* b = &from[i * 10];   // the other pose, weight 1 - w
    for (int c = 0; c < 3; ++c) {
      a[c] = b[c] + (a[c] - b[c]) * w;          // translation
      a[7 + c] = b[7 + c] + (a[7 + c] - b[7 + c]) * w;  // scale
    }
    float dot = 0.0F;
    for (int c = 0; c < 4; ++c) dot += a[3 + c] * b[3 + c];
    const float sign = dot < 0.0F ? -1.0F : 1.0F;  // short arc
    float len = 0.0F;
    for (int c = 0; c < 4; ++c) {
      a[3 + c] = sign * b[3 + c] + (a[3 + c] - sign * b[3 + c]) * w;
      len += a[3 + c] * a[3 + c];
    }
    len = sqrtf(len);
    if (len > 1e-6F)
      for (int c = 0; c < 4; ++c) a[3 + c] /= len;
    else
      a[6] = 1.0F;  // w component
    animatedCur[i] |= fromAnimated[i];
  }
}

void SkelInstance::evalPose() {
  posedOnce = true;  // the globals below are a real pose from here on
  globalsStale = false;
  if (overridden) {
    // The game's pose (setPoseOverride): its model-space node matrices ARE
    // the globals; only the palette is built here.
    memcpy(static_cast<void*>(globals.data()), overrideGlobals.data(),
           globals.size() * sizeof(M4x4));
    for (size_t j = 0; j < model->palette.size(); j++)
      mulM4Vu0(palette[j].data, globals[model->palette[j].node].data,
               model->palette[j].ibm);
    return;
  }
  evalClipGlobals(globals);
  for (size_t j = 0; j < model->palette.size(); j++)
    mulM4Vu0(palette[j].data, globals[model->palette[j].node].data,
             model->palette[j].ibm);
}

bool SkelInstance::clipNodeGlobal(s32 node, M4x4* out) {
  if (node < 0 || (size_t)node >= globals.size()) return false;
  if (!overridden) return currentNodeGlobal(node, out);
  if (clipGlobals.size() != globals.size()) {
    clipGlobals.resize(globals.size());
    clipStale = true;
  }
  if (clipStale) {
    evalClipGlobals(clipGlobals);
    clipStale = false;
  }
  *out = clipGlobals[node];
  return true;
}

void SkelInstance::evalBaseLocals() {
  if (bsSpace >= 0 && bsCount > 0) {
    // Blend-space mode: the shown samples folded one by one - after k of
    // them localsCur is their normalised blend, so the next one enters with
    // its weight over the running total (the same nlerp as the crossfade).
    evalLocals(bs[0], localsCur, animatedCur);
    float total = bsWeight[0];
    for (int k = 1; k < bsCount; k++) {
      evalLocals(bs[k], localsPrev, animatedPrev);
      const float next = total + bsWeight[k];
      blendLocals(localsPrev, animatedPrev, next > 0.0F ? total / next : 1.0F);
      total = next;
    }
  } else {
    evalLocals(cur, localsCur, animatedCur);
    if (prev.clip >= 0 && fadeT < 1.0F) {
      // Crossfade: blend the two poses' local transforms (nlerp rotations),
      // then walk the hierarchy once.
      evalLocals(prev, localsPrev, animatedPrev);
      blendLocals(localsPrev, animatedPrev, fadeT);  // 0 = all prev, 1 = all cur
    }
  }
  if (fromT < 1.0F && localsFrom.size() == localsCur.size())
    blendLocals(localsFrom, animatedFrom, fromT);  // 0 = all the frozen pose
}

void SkelInstance::evalClipGlobals(std::vector<M4x4>& out) {
  evalBaseLocals();
  if (aux.clip >= 0 && auxWeight > 0.0F) {
    // The held layer rides on top of the (cross-faded) playing pose. The
    // prev scratch is free again by now, so it samples into that.
    evalLocals(aux, localsPrev, animatedPrev);
    blendLocals(localsPrev, animatedPrev, 1.0F - auxWeight);
  }

  // globals, parents-first; matrix nodes are never animated (glTF spec),
  // an animated flag overrides just in case a file breaks that rule
  for (u32 i : model->order) {
    const SkelNode& node = model->nodes[i];
    alignas(16) float local[16];  // mulM4Vu0 loads it with lqc2
    const float* localPtr = local;
    if (node.hasMatrix && !animatedCur[i]) {
      localPtr = node.matrix;
    } else {
      const float* l = &localsCur[(size_t)i * 10];
      fromTrs(local, l, l + 3, l + 7);
    }
    if (node.parent >= 0)
      mulM4Vu0(out[i].data, out[node.parent].data, localPtr);
    else
      memcpy(out[i].data, localPtr, 16 * sizeof(float));
  }
}

// The VU0 blend of one part's corners: the body of skinParts' corner loop,
// shared with the compact stream path (SkelInstance::skinInto), which blends
// into float scratch and quantizes it. $vf20/$vf21 carry the running AABB
// across the loop: the caller sets them (skinParts its box; the stream path any
// value, nothing reads them). Model-space outputs, as skinParts writes them.
static void skinBlendCorners(const M4x4* pal, const SkelBindLod& bind, u32 count,
                             Vec4* outV, Vec4* outN) {
  const Vec4* srcP = bind.bindPositions.data();
  const Vec4* srcN = bind.bindNormals.data();
  const Vec4* wq = bind.skinWeights.data();
  const u8* joints = bind.sortedJoints.data();
  const u8* infl = bind.influences.data();
  const u32* skinSource = bind.skinSource.data();

  for (u32 v = 0; v < count; v++) {
    const u32 source = skinSource[v];
    if (source != v) {
      // No Vec4 helper here: VU0's running AABB must survive the copy.
      asm volatile(
          "lqc2 $vf1, 0(%[srcv]) \n\t"
          "lqc2 $vf2, 0(%[srcn]) \n\t"
          "sqc2 $vf1, 0(%[dstv]) \n\t"
          "sqc2 $vf2, 0(%[dstn]) \n\t"
          : : [srcv] "r"(outV + source), [srcn] "r"(outN + source),
              [dstv] "r"(outV + v), [dstn] "r"(outN + v) : "memory");
      continue;
    }
    const u8* j = &joints[(size_t)v * 4];
    const u8 n = infl[v];

    if (n == 2) {
      // two influences - the common case for smooth skinning
      const float* p0 = pal[j[0]].data;
      const float* p1 = pal[j[1]].data;
      asm volatile(
          "lqc2         $vf9, 0x00(%[wgt])       \n\t"
          "lqc2         $vf1, 0x00(%[p0])        \n\t"
          "lqc2         $vf2, 0x00(%[p1])        \n\t"
          "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
          "vmaddy.xyzw  $vf5, $vf2, $vf9         \n\t"
          "lqc2         $vf1, 0x10(%[p0])        \n\t"
          "lqc2         $vf2, 0x10(%[p1])        \n\t"
          "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
          "vmaddy.xyzw  $vf6, $vf2, $vf9         \n\t"
          "lqc2         $vf1, 0x20(%[p0])        \n\t"
          "lqc2         $vf2, 0x20(%[p1])        \n\t"
          "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
          "vmaddy.xyzw  $vf7, $vf2, $vf9         \n\t"
          "lqc2         $vf1, 0x30(%[p0])        \n\t"
          "lqc2         $vf2, 0x30(%[p1])        \n\t"
          "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
          "vmaddy.xyzw  $vf8, $vf2, $vf9         \n\t"
          :
          : [wgt] "r"(wq + v), [p0] "r"(p0), [p1] "r"(p1)
          : "memory");
    } else if (n == 1) {
      // single influence - weight is exactly 1, use the matrix as-is
      const float* p0 = pal[j[0]].data;
      asm volatile(
          "lqc2         $vf5, 0x00(%[p0])        \n\t"
          "lqc2         $vf6, 0x10(%[p0])        \n\t"
          "lqc2         $vf7, 0x20(%[p0])        \n\t"
          "lqc2         $vf8, 0x30(%[p0])        \n\t"
          :
          : [p0] "r"(p0)
          : "memory");
    } else if (n >= 3) {
      // 3 or 4 influences - blend all four slots (a 0-weight 4th slot
      // contributes exactly 0; the loader validated every index)
      const float* p0 = pal[j[0]].data;
      const float* p1 = pal[j[1]].data;
      const float* p2 = pal[j[2]].data;
      const float* p3 = pal[j[3]].data;
      asm volatile(
          "lqc2         $vf9, 0x00(%[wgt])       \n\t"
          "lqc2         $vf1, 0x00(%[p0])        \n\t"
          "lqc2         $vf2, 0x00(%[p1])        \n\t"
          "lqc2         $vf3, 0x00(%[p2])        \n\t"
          "lqc2         $vf4, 0x00(%[p3])        \n\t"
          "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
          "vmadday.xyzw $ACC, $vf2, $vf9         \n\t"
          "vmaddaz.xyzw $ACC, $vf3, $vf9         \n\t"
          "vmaddw.xyzw  $vf5, $vf4, $vf9         \n\t"
          "lqc2         $vf1, 0x10(%[p0])        \n\t"
          "lqc2         $vf2, 0x10(%[p1])        \n\t"
          "lqc2         $vf3, 0x10(%[p2])        \n\t"
          "lqc2         $vf4, 0x10(%[p3])        \n\t"
          "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
          "vmadday.xyzw $ACC, $vf2, $vf9         \n\t"
          "vmaddaz.xyzw $ACC, $vf3, $vf9         \n\t"
          "vmaddw.xyzw  $vf6, $vf4, $vf9         \n\t"
          "lqc2         $vf1, 0x20(%[p0])        \n\t"
          "lqc2         $vf2, 0x20(%[p1])        \n\t"
          "lqc2         $vf3, 0x20(%[p2])        \n\t"
          "lqc2         $vf4, 0x20(%[p3])        \n\t"
          "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
          "vmadday.xyzw $ACC, $vf2, $vf9         \n\t"
          "vmaddaz.xyzw $ACC, $vf3, $vf9         \n\t"
          "vmaddw.xyzw  $vf7, $vf4, $vf9         \n\t"
          "lqc2         $vf1, 0x30(%[p0])        \n\t"
          "lqc2         $vf2, 0x30(%[p1])        \n\t"
          "lqc2         $vf3, 0x30(%[p2])        \n\t"
          "lqc2         $vf4, 0x30(%[p3])        \n\t"
          "vmulax.xyzw  $ACC, $vf1, $vf9         \n\t"
          "vmadday.xyzw $ACC, $vf2, $vf9         \n\t"
          "vmaddaz.xyzw $ACC, $vf3, $vf9         \n\t"
          "vmaddw.xyzw  $vf8, $vf4, $vf9         \n\t"
          :
          : [wgt] "r"(wq + v), [p0] "r"(p0), [p1] "r"(p1), [p2] "r"(p2),
            [p3] "r"(p3)
          : "memory");
    } else {
      // all-zero weights - zero matrix collapses the vertex to the
      // origin, the same degenerate result the stage-1 baker produced
      asm volatile(
          "vsub.xyzw    $vf5, $vf0, $vf0         \n\t"
          "vsub.xyzw    $vf6, $vf0, $vf0         \n\t"
          "vsub.xyzw    $vf7, $vf0, $vf0         \n\t"
          "vsub.xyzw    $vf8, $vf0, $vf0         \n\t" ::);
    }

    // shared tail: transform by $vf5-$vf8, normalize, fold the AABB
    asm volatile(
        // position (x, y, z, 1) -> $vf11, force w = 1
        "lqc2         $vf10, 0x00(%[pos])      \n\t"
        "vmulax.xyzw  $ACC, $vf5, $vf10        \n\t"
        "vmadday.xyzw $ACC, $vf6, $vf10        \n\t"
        "vmaddaz.xyzw $ACC, $vf7, $vf10        \n\t"
        "vmaddw.xyzw  $vf11, $vf8, $vf10       \n\t"
        "vmulw.w      $vf11, $vf0, $vf0        \n\t"
        // normal (nx, ny, nz, 0) -> $vf12 (w = 0 drops the translation)
        "lqc2         $vf12, 0x00(%[nrm])      \n\t"
        "vmulax.xyzw  $ACC, $vf5, $vf12        \n\t"
        "vmadday.xyzw $ACC, $vf6, $vf12        \n\t"
        "vmaddaz.xyzw $ACC, $vf7, $vf12        \n\t"
        "vmaddw.xyzw  $vf12, $vf8, $vf12       \n\t"
        // 1 / sqrt(len^2 + eps); AABB fold hides the vrsqrt latency
        "vmul.xyz     $vf13, $vf12, $vf12      \n\t"
        "vaddy.x      $vf13, $vf13, $vf13      \n\t"
        "vaddz.x      $vf13, $vf13, $vf13      \n\t"
        "vaddw.x      $vf13, $vf13, $vf20      \n\t"
        "vrsqrt       $Q, $vf0w, $vf13x        \n\t"
        "vmini.xyz    $vf20, $vf20, $vf11      \n\t"
        "vmax.xyz     $vf21, $vf21, $vf11      \n\t"
        "sqc2         $vf11, 0x00(%[outv])     \n\t"
        "vwaitq                                \n\t"
        "vmulq.xyz    $vf12, $vf12, $Q         \n\t"
        "vmulw.w      $vf12, $vf0, $vf0        \n\t"
        "sqc2         $vf12, 0x00(%[outn])     \n\t"
        :
        : [outv] "r"(outV + v), [outn] "r"(outN + v), [pos] "r"(srcP + v),
          [nrm] "r"(srcN + v)
        : "memory");
  }
}

void SkelInstance::skinParts(u8 lod) {
  // The whole per-vertex job runs on VU0 in macro mode (the era-correct
  // split: animation on VU0, 3D on VU1, game code on EE): build the blended
  // palette matrix in $vf5-$vf8 - dispatching on the vertex's influence
  // count so 1- and 2-bone vertices (the vast majority) pay for exactly
  // that many matrix loads - then transform position and normal, normalize
  // the normal with vrsqrt, and fold the skinned AABB with vmini/vmax.
  //
  // VU0 register state deliberately spans the asm blocks below: $vf5-$vf8
  // carry the matrix from the blend block into the shared tail, and
  // $vf20/$vf21 hold the running AABB across the whole loop ($vf20.w
  // carries the epsilon added to squared normal lengths, so a degenerate
  // zero normal divides by sqrt(eps) and comes out ~0 instead of blowing
  // up - the EE loop's `len > 1e-6` guard). That is safe for the same
  // reason every M4x4/Vec4 helper is: GCC never emits COP2 code of its
  // own, and nothing else in the engine runs VU0 concurrently. Don't call
  // anything between these blocks.
  // (first-use output buffers are allocated here, before the VU0 state
  // below is set up; a part whose wanted level's buffers the heap could not
  // supply skins - and draws - the cheapest level it holds instead)
  for (size_t pi = 0; pi < partLods.size(); pi++) {
    partLevel[pi] = heldLevel(partLods[pi], lod, true);
#if TYRA_MEMORY_MANAGER
    // Added by TyraX: the wanted level's output was refused - say which level
    // was drawn instead (Memory::Manager::noteFallback, once a second per
    // model, part and level: the key is their shared bind data).
    const u8 wanted =
        lod < partLods[pi].size() ? lod : (u8)(partLods[pi].size() - 1);
    if (partLevel[pi] != wanted) {
      char what[48];
      snprintf(what, sizeof what, "skin output: drew lod %u",
               static_cast<unsigned>(partLevel[pi]));
      Memory::Manager::instance().noteFallback(
          what, model->parts[pi].name.c_str(), wanted,
          partLods[pi][wanted].bind);
    }
#endif
  }
  float bmin[4] alignas(16) = {FLT_MAX, FLT_MAX, FLT_MAX, 1e-12F};
  float bmax[4] alignas(16) = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0F};
  asm volatile(
      "lqc2         $vf20, 0x00(%[bmin])     \n\t"
      "lqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");  // reads the two initializers above (TyraX)

  const M4x4* pal = palette.data();
  for (size_t pi = 0; pi < partLods.size(); pi++) {
    const auto& chain = partLods[pi];
    const PartLod& plod = chain[partLevel[pi]];
    skinBlendCorners(pal, *plod.bind, plod.count, plod.outV, plod.outN);
  }

  asm volatile(
      "sqc2         $vf20, 0x00(%[bmin])     \n\t"
      "sqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");

  // Refresh the frame bboxes from the skinned result - the bbox cache is
  // keyed by frame data and would otherwise stay at the bind pose (matters
  // for anything that culls by mesh bbox).
  Vec4 corners[2];
  corners[0].set(bmin[0], bmin[1], bmin[2], 1.0F);
  corners[1].set(bmax[0], bmax[1], bmax[2], 1.0F);
  const BBox box(corners, 2);
  if (!mesh->frames.empty() && mesh->frames[0]->bbox)
    *mesh->frames[0]->bbox = box;
  for (auto* material : mesh->materials)
    if (!material->frames.empty() && material->frames[0]->bbox)
      *material->frames[0]->bbox = box;
}

// --- blend spaces (docs/blend-spaces.md) ------------------------------------

namespace {
constexpr float kBlendLodHysteresis = 0.1F;  // weight a rival needs to take over
}

s32 SkelInstance::findBlendSpace(const char* name) const {
  if (!name || !name[0]) return -1;
  for (size_t i = 0; i < model->blendSpaces.size(); i++)
    if (model->blendSpaces[i].name == name) return (s32)i;
  return -1;
}

void SkelInstance::setBlendSpace(s32 space, float fadeSeconds) {
  if (space >= (s32)model->blendSpaces.size()) space = -1;
  if (space == bsSpace) return;
  if (fadeSeconds > 0.0F && (bsSpace >= 0 || cur.clip >= 0)) freezeFrom(fadeSeconds);
  bsSpace = space;
  bsCount = 0;
  bsPointSet = false;  // the first input is taken as is: nothing to smooth from
  bsLodSample = -1;
  bsWasSingle = false;
  prev.clip = -1;  // the frozen pose replaces any layer crossfade
  fadeT = 1.0F;
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::setBlendInput(float dirDeg, float speed, float phase) {
  bsInDir = dirDeg;
  bsInSpeed = speed > 0.0F ? speed : 0.0F;
  bsHeldPhase = phase < 0.0F ? -1.0F : (phase > 1.0F ? 1.0F : phase);
}

void SkelInstance::freezeFrom(float seconds) {
  if (seconds <= 0.0F) {
    fromT = 1.0F;
    return;
  }
  if (localsFrom.size() != localsCur.size()) {  // a model without blend spaces: sized once
    localsFrom.resize(localsCur.size());
    animatedFrom.resize(animatedCur.size());
  }
  evalBaseLocals();  // the pose showing now (an in-flight fade included)
  memcpy(localsFrom.data(), localsCur.data(), localsCur.size() * sizeof(float));
  memcpy(animatedFrom.data(), animatedCur.data(), animatedCur.size());
  fromT = 0.0F;
  fromDuration = seconds;
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::advanceFrom(float dt) {
  if (dt <= 0.0F || fromT >= 1.0F) return;
  fromT += fromDuration > 0.0F ? dt / fromDuration : 1.0F;
  if (fromT > 1.0F) fromT = 1.0F;
  poseDirty = globalsStale = clipStale = true;
}

void SkelInstance::advanceBlend(float dt) {
  if (bsSpace < 0) {
    advanceFrom(dt);
    return;
  }
  const SkelBlendSpace& sp = model->blendSpaces[bsSpace];
  const int n = (int)sp.samples.size();
  // the smoothing chases the input clamped to the fastest ring (see
  // blendspace::inputPoint: unclamped, a run's start would pop)
  const blendspace::Point target =
      blendspace::inputPoint(sp.samples.data(), n, bsInDir, bsInSpeed);
  if (!bsPointSet) {
    bsPoint = target;
    bsPointSet = true;
  } else {
    bsPoint = blendspace::smooth(bsPoint, target, dt, sp.smoothing);
  }
  float dir = 0.0F, speed = 0.0F;
  blendspace::fromPoint(bsPoint, &dir, &speed);
  blendspace::Weights w =
      blendspace::weightsFor(sp.samples.data(), n, sp.grid ? &sp.lines : nullptr, dir, speed);

  // The sync phase: held by the game, or played - by distance with a
  // stride, else at the shown sync samples' weighted rate.
  if (bsHeldPhase >= 0.0F) {
    bsPhase = bsHeldPhase;
  } else if (dt > 0.0F) {
    float rate = 0.0F;
    if (sp.stride > 0.0F) {
      rate = bsInSpeed / sp.stride;  // the real speed: the feet cover real ground
    } else {
      float wSum = 0.0F;
      for (int k = 0; k < w.count; k++) {
        const blendspace::Sample& sm = sp.samples[w.sample[k]];
        const float d = model->clips[sm.clip].duration;
        if (sm.ownTime || d <= 0.0F) continue;
        rate += w.weight[k] / d;
        wSum += w.weight[k];
      }
      rate = wSum > 0.0F ? rate / wSum : 0.0F;
    }
    bsPhase += dt * rate;
    bsPhase -= floorf(bsPhase);
  }
  if (dt > 0.0F)
    for (int k = 0; k < n && k < (int)bsOwnClock.size(); k++) {
      const blendspace::Sample& sm = sp.samples[k];
      const float d = model->clips[sm.clip].duration;
      if (!sm.ownTime || d <= 0.0F) continue;
      bsOwnClock[k] = fmodf(bsOwnClock[k] + dt, d);
    }

  // The blend LOD: the best-fitting sample alone; every change of what is
  // shown (into or out of the LOD, or another sample) crossfades.
  if (bsSingle && w.count > 0) {
    const int pick = blendspace::heaviest(w, bsLodSample, kBlendLodHysteresis);
    if ((bsWasSingle && pick != bsLodSample) || (!bsWasSingle && bsCount > 0))
      freezeFrom(sp.lodFade);
    bsLodSample = pick;
    w.count = 1;
    w.sample[0] = pick;
    w.weight[0] = 1.0F;
  } else {
    if (bsWasSingle && bsCount > 0) freezeFrom(sp.lodFade);
    bsLodSample = -1;
  }
  bsWasSingle = bsSingle;

  bool changed = w.count != bsCount;
  for (int k = 0; k < w.count; k++) {
    const blendspace::Sample& sm = sp.samples[w.sample[k]];
    const SkelClip& clip = model->clips[sm.clip];
    const float t = sm.ownTime && w.sample[k] < (int)bsOwnClock.size()
                        ? bsOwnClock[w.sample[k]]
                        : bsPhase * clip.duration;
    Layer& l = bs[k];
    if (l.clip != sm.clip) {
      l.clip = sm.clip;
      l.loop = true;
      l.held = true;
      l.cursors.assign(clip.channels.size(), 0);  // reserved: no allocation
      changed = true;
    }
    if (l.time != t || bsWeight[k] != w.weight[k]) changed = true;
    l.time = t;
    bsWeight[k] = w.weight[k];
  }
  bsCount = w.count;
  if (changed) poseDirty = globalsStale = clipStale = true;
  advanceFrom(dt);
}

int SkelInstance::shownSamples(s32 clips[3], float weights[3]) const {
  if (bsSpace < 0) return 0;
  const float share = fromT < 1.0F ? fromT : 1.0F;  // the rest: the pose faded from
  for (int k = 0; k < 3; k++) {
    clips[k] = k < bsCount ? bs[k].clip : -1;
    weights[k] = k < bsCount ? bsWeight[k] * share : 0.0F;
  }
  return bsCount;
}

int SkelInstance::baseKey(s32 clips[3], float times[3], float weights[3]) const {
  if (fromT < 1.0F) return -1;
  if (bsSpace >= 0 && bsCount > 0) {
    for (int k = 0; k < bsCount; k++)
      clips[k] = bs[k].clip, times[k] = bs[k].time, weights[k] = bsWeight[k];
    return bsCount;
  }
  if (fadeT < 1.0F) return -1;
  clips[0] = cur.clip, times[0] = cur.time, weights[0] = 1.0F;
  return 1;
}

// ---------------------------------------------------------------------------
// Compact streams (skinInto): a part's corners skinned by the scalar core
// straight into the compact stream the renderer draws (renderer/3d/mesh/dynamic/
// skel_skin.hpp, docs/animated-models.md "Skinning into compact streams").

namespace {

// Three 16-corner parts a package: the stream's package size.
constexpr u32 kSkinPackage = 48;

u8 levelOf(size_t levels, u8 lod) {
  return lod < levels ? lod : (u8)(levels - 1);
}

// The chain's mesh for a level: level 0 the part, level k the part's lods[k - 1].
const SkelMesh& meshAt(const SkelModel& model, size_t part, u8 level) {
  if (level == 0) return model.parts[part];
  return model.parts[part].lods[level - 1];
}

// The stream check (SkelSkin::gStreamCheck): the written stream's corner v against
// the float reference - the palette on the bind position decoded in metres (the
// same decode the bind data uses, independent of the skin core's strides).
void checkStream(const SkelModel& model, size_t part, u8 level, const float* palette,
                 u32 slots, const StaPipCompactStream& written) {
  const SkelMesh& mesh = meshAt(model, part, level);
  const u32 n = mesh.vertexCount;
  if (n == 0 || written.vertexCount() != n) return;
  std::vector<float> pos((size_t)n * 3);
  std::vector<float> nrm((size_t)n * 3);
  decodeMesh(model.box, mesh, pos.data(), nrm.data(), nullptr);
  const StaPipCompactView view(written);
  const float step = written.box().step;
  float worst = 0.0F;
  for (u32 v = 0; v < n; v++) {
    const float* p = &pos[(size_t)v * 3];
    float sum = 0.0F;
    for (int k = 0; k < 4; k++) sum += (float)mesh.weights[(size_t)v * 4 + k];
    float ref[3] = {0.0F, 0.0F, 0.0F};
    if (sum > 0.0F)
      for (int k = 0; k < 4; k++) {
        const u32 j = mesh.joints[(size_t)v * 4 + k];
        if (j >= slots) continue;
        const float wk = (float)mesh.weights[(size_t)v * 4 + k] / sum;
        const float* m = palette + 16 * j;
        for (int i = 0; i < 3; i++)
          ref[i] += wk * (m[0 * 4 + i] * p[0] + m[1 * 4 + i] * p[1] + m[2 * 4 + i] * p[2] +
                          m[3 * 4 + i]);
      }
    float got[4];
    view.position(v, got);
    for (int i = 0; i < 3; i++) {
      const float d = fabsf(got[i] - ref[i]) / step;
      if (d > worst) worst = d;
    }
  }
  SkelSkin::StreamCheck& c = SkelSkin::gStreamCheck;
  c.passes++;
  c.corners += n;
  if (worst > c.worstSteps) c.worstSteps = worst;
}

}  // namespace

bool SkelInstance::ensureStreams(size_t pi, u8 level) {
  PartLod& pl = partLods[pi][level];
  if (pl.stream[0].allocated() && pl.stream[1].allocated()) return true;
  if (!pl.bind) return false;
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  // The layout the part's bag reads (set by the owner), else the default: a lit
  // part with the texture its mesh carries.
  if (!skinBoxReady) {
    skinBox = skinOutputBox(*model);
    skinBoxReady = true;
  }
  const SkelMesh& mesh = meshAt(*model, pi, level);
  const u32 n = mesh.vertexCount;
  if (n == 0 || pl.bind->count != n) return false;
  const bool textured = !mesh.uv.empty();
  const StaPipCompactLayout layout =
      pi < streamLayouts.size() && streamLayouts[pi].packageSize != 0
          ? streamLayouts[pi]
          : StaPipCompactLayout::forClass(textured, false, true, false, kSkinPackage, false);

  // The packer's source: four floats a corner for position, normal and ST.
  // Decoded once here and dropped after both sides are packed (transient).
  float* decoded = new (std::nothrow) float[(size_t)n * 8];
  float* source = new (std::nothrow) float[(size_t)n * 12];
  if (decoded == nullptr || source == nullptr) {
    delete[] decoded;
    delete[] source;
    return false;
  }
  float* pos3 = decoded;
  float* nrm3 = decoded + 3 * (size_t)n;
  float* uv2 = decoded + 6 * (size_t)n;
  decodeMesh(model->box, mesh, pos3, nrm3, textured ? uv2 : nullptr);
  float* spos = source;
  float* snrm = source + 4 * (size_t)n;
  float* sst = source + 8 * (size_t)n;
  for (u32 v = 0; v < n; v++) {
    for (int c = 0; c < 3; c++) {
      spos[v * 4 + c] = pos3[v * 3 + c];
      snrm[v * 4 + c] = nrm3[v * 3 + c];
    }
    spos[v * 4 + 3] = 1.0F;
    snrm[v * 4 + 3] = 0.0F;
    sst[v * 4 + 0] = textured ? uv2[v * 2] : 0.0F;
    sst[v * 4 + 1] = textured ? uv2[v * 2 + 1] : 0.0F;
    sst[v * 4 + 2] = 1.0F;
    sst[v * 4 + 3] = 0.0F;
  }
  const StaPipCompactSource packed{spos, sst, nullptr, snrm, n};
  for (int k = 0; k < 2; k++)
    if (!pl.stream[k].allocated())
      Compact::packArrays(layout, packed, skinBox, pl.stream[k]);
  delete[] decoded;
  delete[] source;
  return pl.stream[0].allocated() && pl.stream[1].allocated();
}

SkelInstance::LodStream SkelInstance::lodStream(size_t part, u8 lod) {
  if (part >= partLods.size() || partLods[part].empty()) return LodStream{nullptr, 0};
  auto& chain = partLods[part];
  const u8 want = levelOf(chain.size(), lod);
  if (chain[want].streamed)
    return LodStream{&chain[want].stream[chain[want].side], want};
  // The requested level has no stream yet (or was refused): the cheapest level
  // that has one is drawn instead.
  for (size_t l = chain.size(); l-- > 0;)
    if (chain[l].streamed) return LodStream{&chain[l].stream[chain[l].side], (u8)l};
  return LodStream{nullptr, want};
}

StaPipCompactStream* SkelInstance::spareStream(size_t part, u8 lod) {
  if (part >= partLods.size() || partLods[part].empty()) return nullptr;
  const u8 level = levelOf(partLods[part].size(), lod);
  if (!ensureStreams(part, level)) return nullptr;
  PartLod& pl = partLods[part][level];
  StaPipCompactStream* spare = &pl.stream[pl.streamed ? 1 - pl.side : 0];
  if (spare->isSpanRegistered()) {
    // A frame may still read the span it was drawn from: wait until none does.
    if (!Vif1Queue::collectRetiredImmutableSpan(spare->data())) return nullptr;
    spare->setSpanRegistered(false);
  }
  return spare;
}

void SkelInstance::setStreamLayout(size_t part, const StaPipCompactLayout& layout) {
  if (!model || part >= partLods.size()) return;
  for (const PartLod& pl : partLods[part])
    if (pl.stream[0].allocated() || pl.stream[1].allocated()) return;
  if (streamLayouts.size() < partLods.size())
    streamLayouts.resize(partLods.size());
  streamLayouts[part] = layout;
}

#if TYRA_SKIN_PROBE
#define SKININTO_EARLY(v) (SkelSkin::gProbe.early++, (v))
#else
#define SKININTO_EARLY(v) (v)
#endif

// Scratch for the vector path: the blended floats of the part being streamed. One
// pair serves every part, since a pass consumes it before the next part starts. It
// grows on demand under the Skinning budget; a refused grow means the scalar core.
struct SkinScratch {
  Vec4* pos = nullptr;
  Vec4* nrm = nullptr;
  u32 cap = 0;
};
SkinScratch gSkinScratch;

bool ensureSkinScratch(u32 count) {
  if (count <= gSkinScratch.cap) return true;
  delete[] gSkinScratch.pos;
  delete[] gSkinScratch.nrm;
  gSkinScratch = SkinScratch();
#if TYRA_MEMORY_MANAGER
  Memory::MemScope memoryScope(Memory::Budget::Skinning);
#endif
  Vec4* pos = new (std::nothrow) Vec4[count];
  Vec4* nrm = pos ? new (std::nothrow) Vec4[count] : nullptr;
  if (!pos || !nrm) {
    delete[] pos;
    return false;
  }
  gSkinScratch.pos = pos;
  gSkinScratch.nrm = nrm;
  gSkinScratch.cap = count;
  return true;
}

// One part's stream pass (SkelInstance::skinInto). The vector path blends the part
// with the VU0 corner loop into scratch and quantizes it; the first vector pass of
// the process is checked against the scalar core, which switches the vector path
// off when the streams differ. The scalar core is the fallback everywhere else.
u32 skinPartStream(const SkelSkin::Folded& folded, const SkelSkin::SkelSkinPart& part,
                   const SkelBindLod& bind, const M4x4* pal, StaPipCompactStream& target) {
#if defined(_EE)
  if (SkelSkin::gUseVector && ensureSkinScratch(part.count)) {
    s32 oi[3];  // the pose box's origin in steps (the scalar core's; unused here)
    if (!SkelSkin::posePlace(folded, part, target, oi)) return SkelSkin::kSkinRefused;
    // $vf20/$vf21: the AABB registers skinBlendCorners folds into. The stream path
    // reads no box from them, so they only start from the same values skinParts uses.
    float bmin[4] alignas(16) = {FLT_MAX, FLT_MAX, FLT_MAX, 1e-12F};
    float bmax[4] alignas(16) = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0F};
    asm volatile(
        "lqc2         $vf20, 0x00(%[bmin])     \n\t"
        "lqc2         $vf21, 0x00(%[bmax])     \n\t"
        :
        : [bmin] "r"(bmin), [bmax] "r"(bmax)
        : "memory");
    skinBlendCorners(pal, bind, part.count, gSkinScratch.pos, gSkinScratch.nrm);
    const u32 clamped = SkelSkin::quantizeIntoStream(
        reinterpret_cast<const float*>(gSkinScratch.pos),
        reinterpret_cast<const float*>(gSkinScratch.nrm), part, target);
    if (SkelSkin::gVectorChecked) return clamped;
    SkelSkin::gVectorChecked = true;
    const SkelSkin::VectorCheck check = SkelSkin::checkVectorStream(
        folded, part, target, reinterpret_cast<float*>(gSkinScratch.pos),
        reinterpret_cast<float*>(gSkinScratch.nrm));
    SkelSkin::gUseVector = check.exact == 0 && check.worst <= 1 && check.boxMismatch == 0;
    char msg[176];
    snprintf(msg, sizeof msg,
             "SKINVEC self-check: corners=%u exact-mismatch=%u worst=%u LSB box-mismatch=%u -> %s",
             (unsigned)part.count, (unsigned)check.exact, (unsigned)check.worst,
             (unsigned)check.boxMismatch,
             SkelSkin::gUseVector ? "vector path on" : "vector path OFF (scalar core)");
    TYRA_LOG(msg);
    return check.clamped;
  }
#else
  (void)bind;
  (void)pal;
#endif
  return SkelSkin::skinIntoStream(folded, part, target);
}

// The unique-vertex pass (SkelSkin::gUseUnique): the VU0 corner loop blends the part's unique
// corners (bl.uniq) into float scratch, and writeUniqueStream boxes, quantizes and scatters
// them to every destination. Only the uniques are floats in the pass: the stream is int16.
// Refused when the scratch cannot be had (the caller skips the body this pass).
static u32 skinIntoUniqueVu0(const M4x4* pal, const SkelSkin::SkelSkinPart& part,
                             const SkelBindLod& bl, StaPipCompactStream& target) {
#if defined(_EE)
  const SkelBindLod& u = *bl.uniq;
  if (!ensureSkinScratch(u.count)) return SkelSkin::kSkinRefused;
  // $vf20 is the normal's epsilon source (skinBlendCorners' vaddw.x) and the AABB fold
  // registers: start them the way skinPartStream does, so the normals do not depend on
  // what the previous pass left in them.
  float bmin[4] alignas(16) = {FLT_MAX, FLT_MAX, FLT_MAX, 1e-12F};
  float bmax[4] alignas(16) = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0.0F};
  asm volatile(
      "lqc2         $vf20, 0x00(%[bmin])     \n\t"
      "lqc2         $vf21, 0x00(%[bmax])     \n\t"
      :
      : [bmin] "r"(bmin), [bmax] "r"(bmax)
      : "memory");
  skinBlendCorners(pal, u, u.count, gSkinScratch.pos, gSkinScratch.nrm);
  return SkelSkin::writeUniqueStream(reinterpret_cast<const float*>(gSkinScratch.pos),
                                     reinterpret_cast<const float*>(gSkinScratch.nrm),
                                     u.uniqCorner.data(), u.count, part, target);
#else
  (void)pal;
  (void)part;
  (void)bl;
  (void)target;
  return SkelSkin::kSkinRefused;
#endif
}

// Passes a level may go unused before its stream pair is freed. A body that flickers
// between two levels keeps both; a body that has left a level frees it.
static constexpr u8 kStreamIdlePasses = 8;

void SkelInstance::releaseIdleStreams(u8 lod) {
  for (auto& chain : partLods) {
    const u8 keep = levelOf(chain.size(), lod);
    // A refused level (no stream written for it) keeps the levels it holds as the
    // body's fallback: nothing is freed until the level in use has a drawn stream.
    if (!chain[keep].streamed) continue;
    for (size_t l = 0; l < chain.size(); ++l) {
      PartLod& pl = chain[l];
      if (!pl.stream[0].allocated() && !pl.stream[1].allocated()) continue;
      if (l == keep) {
        pl.idlePasses = 0;
        continue;
      }
      if (pl.idlePasses < kStreamIdlePasses) {
        ++pl.idlePasses;
        continue;
      }
      // release() waits until no frame reads either side (the immutable span lease).
      pl.stream[0].release();
      pl.stream[1].release();
      pl.side = 0;
      pl.streamed = false;
      pl.idlePasses = 0;
    }
  }
}

void SkelInstance::releaseFloatMesh() {
  if (!mesh || floatMeshReleased) return;
  for (const auto& chain : partLods) {
    bool drawn = false;
    for (const PartLod& pl : chain) drawn = drawn || pl.streamed;
    if (!drawn) return;  // a part with no stream still reads the float arrays
  }
  for (size_t pi = 0; pi < mesh->materials.size(); pi++) {
    MeshMaterial* material = mesh->materials[pi];
    if (material == nullptr || material->frames.empty()) continue;
    MeshMaterialFrame* f = material->frames[0];
    delete[] f->vertices;
    f->vertices = nullptr;
    if (f->normals) delete[] f->normals;
    f->normals = nullptr;
    if (f->textureCoords) delete[] f->textureCoords;
    f->textureCoords = nullptr;
  }
  for (auto& chain : partLods) {
    if (chain.empty()) continue;
    chain[0].outV = chain[0].outN = chain[0].uvPtr = nullptr;
  }
  floatMeshReleased = true;
}

u32 SkelInstance::skinInto(u8 lod, StaPipCompactStream* const* partStreams) {
#if TYRA_SKIN_PROBE
  SkelSkin::gProbe.calls++;
  SkelSkin::gProbe.paletteSize = (u32)palette.size();
  SkelSkin::probeReport();
#endif
  if (!model || partLods.empty() || partStreams == nullptr) return SKININTO_EARLY(0);
  if (palette.size() > SkelSkin::kMaxSlots) return SKININTO_EARLY(0);
  // A pass is due when the pose moved or the level changed; otherwise the drawn
  // sides already hold this pose and nothing is written (a split frame's second
  // half, a follower's owner).
  if (!streamPassDue(lod)) return 0;
  const bool dirty = overridden ? overrideDirty : poseDirty;
  if (!compactValid || dirty) {
    evalPose();
    poseDirty = false;
    overrideDirty = false;
  }
  if (!skinBoxReady) {
    skinBox = skinOutputBox(*model);
    skinBoxReady = true;
  }
  // One pose's folds, shared by every part. Static: the EE runs one thread and
  // the fold is consumed before this returns.
  static SkelSkin::Folded folded;
  SkelSkin::foldPalette(reinterpret_cast<const float*>(palette.data()),
                        (u32)palette.size(), model->box, skinBox, folded);
  u32 outside = 0;
  for (size_t pi = 0; pi < partLods.size(); pi++) {
    StaPipCompactStream* target = partStreams[pi];
    if (target == nullptr) {
#if TYRA_SKIN_PROBE
      SkelSkin::gProbe.noTarget++;
#endif
      continue;
    }
    const u8 level = levelOf(partLods[pi].size(), lod);
    PartLod& pl = partLods[pi][level];
    if (target != &pl.stream[0] && target != &pl.stream[1]) continue;
    // The drawn side is never rewritten: a pass writes the spare only.
    if (pl.streamed && target == &pl.stream[pl.side]) continue;
    if (!target->allocated() || !pl.bind) {
#if TYRA_SKIN_PROBE
      SkelSkin::gProbe.noTarget++;
#endif
      continue;
    }
    const SkelMesh& mesh = meshAt(*model, pi, level);
    const SkelSkin::SkelSkinPart part{mesh.pos.data(),         mesh.nrm.data(),
                                      mesh.joints.data(),      mesh.weights.data(),
                                      pl.bind->skinSource.data(), pl.bind->count,
                                      &pl.bind->extent,
                                      pl.bind->infl.size() == pl.bind->count
                                          ? pl.bind->infl.data()
                                          : nullptr,
                                      &pl.bind->scatter};
    u32 r;
    if (SkelSkin::gUseUnique && pl.bind->uniq)
      r = skinIntoUniqueVu0(palette.data(), part, *pl.bind, *target);
    else
      r = SkelSkin::gUseFixed
                       ? SkelSkin::skinIntoFixed(folded, reinterpret_cast<const float*>(palette.data()),
                                                 (u32)palette.size(), model->box, part, *target)
                       : skinPartStream(folded, part, *pl.bind, palette.data(), *target);
    if (r == SkelSkin::kSkinRefused) continue;
    if (SkelSkin::gStreamCheck.enabled)
      checkStream(*model, pi, level, reinterpret_cast<const float*>(palette.data()),
                  (u32)palette.size(), *target);
    const u8 side = target == &pl.stream[1] ? 1 : 0;
    if (pl.streamed && side != pl.side) {
      // The side it replaces is no longer drawn: retire its span (no new
      // reference may name it; the frames that do finish first).
      StaPipCompactStream& old = pl.stream[pl.side];
      if (old.allocated() && old.isSpanRegistered())
        Vif1Queue::retireImmutableSpan(old.data());
    }
    pl.side = side;
    pl.streamed = true;
    if (r > 0) {
      outside += r;
#if TYRA_MEMORY_MANAGER
      // Once a second per model (the key), like the other fallbacks.
      char what[64];
      snprintf(what, sizeof what, "skin out of box count=%u", (unsigned)r);
      Memory::Manager::instance().noteFallback(what, model->parts[pi].name.c_str(), level,
                                               model);
#endif
    }
  }
  releaseIdleStreams(lod);
  releaseFloatMesh();
  compactValid = true;
  compactLod = lod;
  ++compactPasses;
#if TYRA_SKIN_PROBE
  SkelSkin::probeReport();
#endif
  return outside;
}

}  // namespace Tyra

