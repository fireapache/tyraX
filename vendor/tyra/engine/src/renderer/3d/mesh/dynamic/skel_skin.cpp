// The scalar skin core (renderer/3d/mesh/dynamic/skel_skin.hpp).

#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#if TYRA_SKIN_PROBE
#include <cstdio>

#include "debug/debug.hpp"
#endif

namespace Tyra {

#if TYRA_SKIN_PROBE
namespace SkelSkin {
Probe gProbe;

// COP0 Count (294.912 MHz on the EE): the same clock the frame probes read.
static unsigned probeTicks() {
  unsigned t;
  asm volatile("mfc0 %0, $9" : "=r"(t));
  return t;
}
}  // namespace SkelSkin

#define SKIN_PROBE_NOW() SkelSkin::probeTicks()
#define SKIN_PROBE_LAP(field, from) \
  do {                              \
    const unsigned now_ = SkelSkin::probeTicks(); \
    SkelSkin::gProbe.field += now_ - (from);     \
    from = now_;                    \
  } while (0)
#endif

#if TYRA_SKIN_PROBE
#define SKIN_REFUSE() (++SkelSkin::gProbe.refused, kSkinRefused)
#else
#define SKIN_REFUSE() kSkinRefused
#endif

namespace {

// Rounds half away from zero, as Compact::quantize does: floor(|v| + 0.5) with the
// sign kept, taken with a truncating integer conversion (the EE's cvt.w.s) rather
// than a libm floor. The two agree for every |v| + 0.5 below 2^30; past that the
// value saturates (a corner that far out is clamped by the caller anyway).
float roundAway(float v) {
  const float kLimit = 1073741824.0F;  // 2^30
  if (v < 0.0F) {
    const float a = -v + 0.5F;
    return a < kLimit ? -(float)(s32)a : -kLimit;
  }
  const float a = v + 0.5F;
  return a < kLimit ? (float)(s32)a : kLimit;
}

// The whole number of steps of an origin on its grid.
float originIndex(float origin, float step) { return roundAway(origin / step); }

// out = a x b for column-major 4x4 matrices.
void mul4(const float* a, const float* b, float* out) {
  for (int c = 0; c < 4; ++c)
    for (int row = 0; row < 4; ++row) {
      float acc = 0.0F;
      for (int k = 0; k < 4; ++k) acc += a[k * 4 + row] * b[c * 4 + k];
      out[c * 4 + row] = acc;
    }
}

// The bounding box of one set of corners: the largest distance from the origin
// to a corner of [mn, mx].
float cornerRadius(const float mn[3], const float mx[3]) {
  float r2 = 0.0F;
  for (int corner = 0; corner < 8; ++corner) {
    float d2 = 0.0F;
    for (int i = 0; i < 3; ++i) {
      const float v = (corner >> i) & 1 ? mx[i] : mn[i];
      d2 += v * v;
    }
    if (d2 > r2) r2 = d2;
  }
  return std::sqrt(r2);
}

}  // namespace

Compact::Box skinOutputBox(const SkelModel& m) {
  // R: the largest distance from the origin to a corner of the pose box, or of
  // the bind box (the parts' base meshes decoded in the model box).
  float r = cornerRadius(m.min, m.max);
  bool any = false;
  float bmn[3] = {0.0F, 0.0F, 0.0F}, bmx[3] = {0.0F, 0.0F, 0.0F};
  for (const SkelPart& part : m.parts)
    for (size_t v = 0; v < part.vertexCount; ++v)
      for (int i = 0; i < 3; ++i) {
        const float p = m.box.origin[i] + m.box.step * (float)part.pos[v * 3 + i];
        if (!any || p < bmn[i]) bmn[i] = p;
        if (!any || p > bmx[i]) bmx[i] = p;
        any = true;
      }
  if (any) {
    const float rb = cornerRadius(bmn, bmx);
    if (rb > r) r = rb;
  }

  // The smallest power of two step with kSkinBoxFactor x R <= 32767 steps: the
  // step is the format's; each pose's origin is set by skinIntoStream.
  const double half = (double)kSkinBoxFactor * (double)r;
  Compact::Box box{{0.0F, 0.0F, 0.0F}, 1.0F};
  if (!(half > 0.0)) return box;
  int e = 0;
  const double x = half / 32767.0;
  const double mant = std::frexp(x, &e);  // x = mant x 2^e, mant in [0.5, 1)
  // 2^(e-1) <= x < 2^e; the power of two at or above x is 2^e, or 2^(e-1) when
  // x is exactly that power.
  box.step = (float)std::ldexp(1.0, mant == 0.5 ? e - 1 : e);
  return box;
}

namespace SkelSkin {

StreamCheck gStreamCheck;

void foldPalette(const float* palette, u32 slots, const Compact::Box& bind,
                 const Compact::Box& out, Folded& folded) {
#if TYRA_SKIN_PROBE
  const unsigned t0 = SKIN_PROBE_NOW();
  gProbe.folds++;
#endif
  // Dbind: the bind index (q, 1) to the model position, step x (q + origin index).
  float dbind[16] = {};
  dbind[0] = dbind[5] = dbind[10] = bind.step;
  dbind[12] = bind.step * originIndex(bind.origin[0], bind.step);
  dbind[13] = bind.step * originIndex(bind.origin[1], bind.step);
  dbind[14] = bind.step * originIndex(bind.origin[2], bind.step);
  dbind[15] = 1.0F;
  // Qout: the model position to the output index, p / step - origin index.
  const float inv = 1.0F / out.step;
  float qout[16] = {};
  qout[0] = qout[5] = qout[10] = inv;
  qout[12] = -originIndex(out.origin[0], out.step);
  qout[13] = -originIndex(out.origin[1], out.step);
  qout[14] = -originIndex(out.origin[2], out.step);
  qout[15] = 1.0F;
  folded.out = out;
  folded.slots = slots < kMaxSlots ? slots : kMaxSlots;
  for (u32 k = 0; k < slots && k < kMaxSlots; ++k) {
    const float* m = palette + 16 * k;
    float qm[16];
    mul4(qout, m, qm);
    mul4(qm, dbind, folded.f[k]);
    for (int c = 0; c < 3; ++c)
      for (int row = 0; row < 3; ++row) folded.r[k][c * 3 + row] = m[c * 4 + row];
  }
#if TYRA_SKIN_PROBE
  gProbe.fold += SKIN_PROBE_NOW() - t0;
#endif
}

void blendCorner(const Folded& folded, const s16* pos, const s8* nrm,
                 const u8* joints, const u8* weights, float px[3], float nn[3]) {
  float sum = 0.0F;
  for (int k = 0; k < 4; ++k) sum += (float)weights[k];
  // One reciprocal per corner (the weights' sum) and one per normal scale, not a
  // divide per weight: the EE's divide is the expensive operation here.
  const float invSum = sum > 0.0F ? 1.0F / sum : 0.0F;
  const float q[4] = {(float)pos[0], (float)pos[1], (float)pos[2], 1.0F};
  const float nv[3] = {(float)nrm[0] * (1.0F / 127.0F), (float)nrm[1] * (1.0F / 127.0F),
                       (float)nrm[2] * (1.0F / 127.0F)};
  px[0] = px[1] = px[2] = 0.0F;
  nn[0] = nn[1] = nn[2] = 0.0F;
  for (int k = 0; k < 4; ++k) {
    const float wk = (float)weights[k] * invSum;
    const u32 j = joints[k];
    const float* f = folded.f[j];
    const float* r = folded.r[j];
    for (int i = 0; i < 3; ++i)
      px[i] += wk * (f[0 * 4 + i] * q[0] + f[1 * 4 + i] * q[1] + f[2 * 4 + i] * q[2] +
                     f[3 * 4 + i] * q[3]);
    for (int i = 0; i < 3; ++i)
      nn[i] += wk * (r[0 * 3 + i] * nv[0] + r[1 * 3 + i] * nv[1] + r[2 * 3 + i] * nv[2]);
  }
}

u32 skinCorner(const Folded& folded, const s16* pos, const s8* nrm,
               const u8* joints, const u8* weights, const s32* originIndex,
               s16* outPos, s8* outNrm) {
  float px[3], nn[3];
  blendCorner(folded, pos, nrm, joints, weights, px, nn);
  u32 clamped = 0;
  for (int i = 0; i < 3; ++i) {
    float v = roundAway(px[i]) - (float)originIndex[i];
    if (v > 32767.0F) {
      v = 32767.0F;
      clamped = 1;
    } else if (v < -32767.0F) {
      v = -32767.0F;
      clamped = 1;
    }
    outPos[i] = (s16)v;
  }
  outPos[3] = 0;
  const float len = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
  const float scale = len > 0.0F ? 127.0F / len : 0.0F;
  for (int i = 0; i < 3; ++i) {
    float v = roundAway(nn[i] * scale);
    if (v > 127.0F) v = 127.0F;
    if (v < -127.0F) v = -127.0F;
    outNrm[i] = (s8)v;
  }
  outNrm[3] = 0;
  return clamped;
}

void bindExtent(const s16* pos, const u8* joints, const u8* weights, u32 count,
                SkelSkinExtent& out) {
  out.qmin[0] = out.qmin[1] = out.qmin[2] = 32767;
  out.qmax[0] = out.qmax[1] = out.qmax[2] = -32768;
  out.used = 0;
  for (u32 v = 0; v < count; ++v) {
    const s16* q = pos + 3 * v;
    for (int i = 0; i < 3; ++i) {
      if (q[i] < out.qmin[i]) out.qmin[i] = q[i];
      if (q[i] > out.qmax[i]) out.qmax[i] = q[i];
    }
    for (int k = 0; k < 4; ++k) {
      const u32 j = joints[4 * v + k];
      if (weights[4 * v + k] > 0 && j < kMaxSlots) out.used |= 1ULL << j;
    }
  }
}

// Places the stream's box for this pose (the extent and rebox of skinIntoStream):
// centred on the grid point at the middle of the pose's own extent. False when
// the stream refuses the rebox.
bool posePlace(const Folded& folded, const SkelSkinPart& part, StaPipCompactStream& out,
               s32 oi[3]) {
#if TYRA_SKIN_PROBE
  unsigned pt = SKIN_PROBE_NOW();
#endif
  // The stream must take the part: allocated, the same vertex count, the folds' step
  // and a position first and a normal (the checks skinIntoStream makes).
  if (!out.allocated() || part.count == 0 || out.vertexCount() != part.count) return false;
  if (out.box().step != folded.out.step) return false;
  const StaPipCompactLayout& lay = out.layout();
  if (lay.attrs[0] != Compact::Attr::Position) return false;
  if (lay.indexOf(Compact::Attr::Normal) < 0 && lay.indexOf(Compact::Attr::EnvNormal) < 0)
    return false;
  // This pose's box: centred on the grid point at the middle of the pose's own
  // extent. The extent is bounded, not blended: a corner is a convex combination
  // of its joints' folds applied to its bind index, and the bind indices lie in
  // the part's index box [qmin, qmax]. The image of that box under each used
  // joint's fold therefore covers every corner (and the origin, which a corner
  // whose weights sum to 0 yields), so the union of the images is the extent.
  // The index box and the used slots are bind data (bindExtent, computed with the
  // bind cache); a part without it gets them here. A pose spanning up to 2 x 32767
  // steps fits; a wider one clamps and counts.
  SkelSkinExtent computed;
  const SkelSkinExtent* ext = part.extent;
  if (ext == nullptr) {
    bindExtent(part.pos, part.joints, part.weights, part.count, computed);
    ext = &computed;
  }
  float lo[3] = {0.0F, 0.0F, 0.0F}, hi[3] = {0.0F, 0.0F, 0.0F};
  for (u32 j = 0; j < folded.slots; ++j) {
    if (((ext->used >> j) & 1ULL) == 0) continue;
    const float* f = folded.f[j];
    for (int c = 0; c < 8; ++c) {
      const float v4[4] = {(float)((c & 1) ? ext->qmax[0] : ext->qmin[0]),
                           (float)((c & 2) ? ext->qmax[1] : ext->qmin[1]),
                           (float)((c & 4) ? ext->qmax[2] : ext->qmin[2]), 1.0F};
      for (int i = 0; i < 3; ++i) {
        const float p = f[0 * 4 + i] * v4[0] + f[1 * 4 + i] * v4[1] + f[2 * 4 + i] * v4[2] +
                        f[3 * 4 + i] * v4[3];
        if (p < lo[i]) lo[i] = p;
        if (p > hi[i]) hi[i] = p;
      }
    }
  }
  // A step of margin each way: the blend is rounded in float.
  for (int i = 0; i < 3; ++i) {
    lo[i] -= 1.0F;
    hi[i] += 1.0F;
  }
#if TYRA_SKIN_PROBE
  SKIN_PROBE_LAP(extent, pt);
#endif
  const float step = folded.out.step;
  Compact::Box box;
  box.step = step;
  for (int i = 0; i < 3; ++i) {
    oi[i] = (s32)roundAway(0.5F * (lo[i] + hi[i]));  // already in steps
    box.origin[i] = (float)oi[i] * step;
  }
#if TYRA_SKIN_PROBE
  SKIN_PROBE_LAP(box, pt);
#endif
  return out.rebox(box);
}

u32 skinIntoStream(const Folded& folded, const SkelSkinPart& part,
                   StaPipCompactStream& out) {
#if TYRA_SKIN_PROBE
  gProbe.entries++;
#endif
  if (!out.allocated() || part.count == 0 || out.vertexCount() != part.count)
    return SKIN_REFUSE();
  if (out.box().step != folded.out.step) return SKIN_REFUSE();
  const StaPipCompactLayout& lay = out.layout();
  if (lay.attrs[0] != Compact::Attr::Position) return SKIN_REFUSE();
  s32 nslot = lay.indexOf(Compact::Attr::Normal);
  if (nslot < 0) nslot = lay.indexOf(Compact::Attr::EnvNormal);
  if (nslot < 0) return SKIN_REFUSE();

  const u32 size = lay.packageSize;
  const u32 posBytes = Compact::elementBytes(Compact::Attr::Position);  // 6
  const u32 nrmBytes = Compact::elementBytes(lay.attrs[nslot]);        // 4
  const u8 nslotIndex = static_cast<u8>(nslot);
#if TYRA_SKIN_PROBE
  unsigned pt = SKIN_PROBE_NOW();
#endif

  s32 oi[3];
  if (!posePlace(folded, part, out, oi)) return SKIN_REFUSE();
#if TYRA_SKIN_PROBE
  pt = SKIN_PROBE_NOW();
#endif
  u32 clamped = 0;
  // Corner v is package v / size, element v % size. The loop walks the packages
  // and keeps each package's base pointers, so a corner costs no divide.
  for (u32 p = 0, v = 0; v < part.count; ++p) {
    u8* const pos0 = out.run(p, 0);
    u8* const nrm0 = out.run(p, nslotIndex);
    const u32 end = part.count - v > size ? v + size : part.count;
    for (u32 e = 0; v < end; ++v, ++e) {
      u8* dp = pos0 + e * posBytes;
      u8* dn = nrm0 + e * nrmBytes;
      const u32 src = part.skinSource[v];
      if (src < v) {
        // A repeat copies the elements its source wrote this pose. A source out of
        // its box stores +-32767, so the repeat counts the same way.
        const u8* sp = out.run(src / size, 0) + (src % size) * posBytes;
        const u8* sn = out.run(src / size, nslotIndex) + (src % size) * nrmBytes;
        std::memcpy(dp, sp, posBytes);
        std::memcpy(dn, sn, nrmBytes);
        s16 q[3];
        std::memcpy(q, dp, sizeof(q));
        for (int i = 0; i < 3; ++i)
          if (q[i] == 32767 || q[i] == -32767) {
            ++clamped;
            break;
          }
        continue;
      }
      s16 q[4];
      s8 n[4];
      clamped += skinCorner(folded, part.pos + 3 * v, part.nrm + 4 * v,
                            part.joints + 4 * v, part.weights + 4 * v, oi, q, n);
      std::memcpy(dp, q, posBytes);
      std::memcpy(dn, n, nrmBytes);
    }
  }
#if TYRA_SKIN_PROBE
  SKIN_PROBE_LAP(corners, pt);
#endif
  out.finish();
#if TYRA_SKIN_PROBE
  SKIN_PROBE_LAP(finish, pt);
  Probe& p = gProbe;
  ++p.bodies;
#endif
  return clamped;
}

bool gUseFixed = TYRA_SKIN_FIXED != 0;

u32 skinIntoFixed(const Folded& folded, const float* palette, u32 slots,
                  const Compact::Box& bindBox, const SkelSkinPart& part,
                  StaPipCompactStream& out) {
  if (!out.allocated() || part.count == 0 || out.vertexCount() != part.count)
    return SKIN_REFUSE();
  const StaPipCompactLayout& lay = out.layout();
  if (lay.attrs[0] != Compact::Attr::Position) return SKIN_REFUSE();
  s32 nslot = lay.indexOf(Compact::Attr::Normal);
  if (nslot < 0) nslot = lay.indexOf(Compact::Attr::EnvNormal);
  if (nslot < 0) return SKIN_REFUSE();
  s32 oi[3];
  if (!posePlace(folded, part, out, oi)) return SKIN_REFUSE();
  // The folds' output box is the one posePlace just set: its origin is oi steps.
  Compact::Box ob;
  ob.step = folded.out.step;
  for (int i = 0; i < 3; ++i) ob.origin[i] = (float)oi[i] * ob.step;
  // The pre-scale bound: the part's largest bind index (its extent).
  SkelSkinExtent computed;
  const SkelSkinExtent* ext = part.extent;
  if (ext == nullptr) {
    bindExtent(part.pos, part.joints, part.weights, part.count, computed);
    ext = &computed;
  }
  u32 qMax = 1;
  for (int i = 0; i < 3; ++i) {
    const u32 lo = (u32)(ext->qmin[i] < 0 ? -ext->qmin[i] : ext->qmin[i]);
    const u32 hi = (u32)(ext->qmax[i] < 0 ? -ext->qmax[i] : ext->qmax[i]);
    if (lo > qMax) qMax = lo;
    if (hi > qMax) qMax = hi;
  }
  static Fixed::Fold ff;  // one pose's fixed folds, consumed before this returns
  Fixed::fold(palette, slots, bindBox, ob, ff, qMax);

  const u32 size = lay.packageSize;
  const u32 posBytes = Compact::elementBytes(Compact::Attr::Position);  // 6
  const u32 nrmBytes = Compact::elementBytes(lay.attrs[nslot]);        // 4
  const u8 nslotIndex = static_cast<u8>(nslot);
  u32 clamped = 0;
  for (u32 p = 0, v = 0; v < part.count; ++p) {
    u8* const pos0 = out.run(p, 0);
    u8* const nrm0 = out.run(p, nslotIndex);
    const u32 end = part.count - v > size ? v + size : part.count;
    for (u32 e = 0; v < end; ++v, ++e) {
      u8* dp = pos0 + e * posBytes;
      u8* dn = nrm0 + e * nrmBytes;
      const u32 src = part.skinSource[v];
      if (src < v) {
        // A repeat has the same bind data as its source: it copies the source's bytes.
        const u8* sp = out.run(src / size, 0) + (src % size) * posBytes;
        const u8* sn = out.run(src / size, nslotIndex) + (src % size) * nrmBytes;
        std::memcpy(dp, sp, posBytes);
        std::memcpy(dn, sn, nrmBytes);
        s16 q[3];
        std::memcpy(q, dp, sizeof(q));
        for (int i = 0; i < 3; ++i)
          if (q[i] == 32767 || q[i] == -32767) {
            ++clamped;
            break;
          }
        continue;
      }
      s16 q[4];
      s8 n[4];
      clamped += Fixed::cornerFixed(ff, part.pos + 3 * v, part.nrm + 4 * v, part.joints + 4 * v,
                                    part.weights + 4 * v, q, n, nullptr);
      std::memcpy(dp, q, posBytes);
      std::memcpy(dn, n, nrmBytes);
    }
  }
  out.finish();
  return clamped;
}

// The vector path (EE, SkelInstance::skinInto): the VU0 blend writes model-space
// floats into scratch, and quantizeIntoStream quantizes them into the stream.
// Given the same floats, the quantize is the scalar encode: the stream's box is
// the pose's box, and the vector positions and normals match the scalar core's
// encode of the same values bit for bit (the self-check below).
bool gUseVector = true;
bool gVectorChecked = false;

u32 quantizeIntoStream(const float* pos4, const float* nrm4, const SkelSkinPart& part,
                       StaPipCompactStream& out) {
#if TYRA_SKIN_PROBE
  unsigned pt = SKIN_PROBE_NOW();
#endif
  const StaPipCompactLayout& lay = out.layout();
  s32 nslot = lay.indexOf(Compact::Attr::Normal);
  if (nslot < 0) nslot = lay.indexOf(Compact::Attr::EnvNormal);
  if (nslot < 0) return kSkinRefused;
  const u32 size = lay.packageSize;
  const u32 partSize = size / 3;  // the stream's parts (StaPipCompactStream::finish)
  if (partSize == 0) return kSkinRefused;
  const u8 nslotIndex = static_cast<u8>(nslot);
  u32 row[4];
  Compact::rowFor(out.box(), row);  // the grid index of the box origin
  u32 clamped = 0;
  // The part boxes are taken here, from the positions this pass has just written
  // (the same integers finish() would read back), so the stream's finish does not
  // read the positions a second time. `left` counts the corners to the part's end.
  s32 lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
  u32 pi = 0, left = partSize;
  // Package p holds corners p x size ..; positions are 6 bytes apart, normals 4.
  for (u32 p = 0, v = 0; v < part.count; ++p) {
    u8* const pos0 = out.run(p, 0);
    u8* const nrm0 = out.run(p, nslotIndex);
    const u32 n = part.count - v > size ? size : part.count - v;
    Compact::quantizePositions(pos4 + 4 * v, n, out.box(), pos0, 6);
    Compact::quantizeNormals(nrm4 + 4 * v, n, nrm0);
    for (u32 e = 0; e < n; ++e, ++v) {
      s16 q[3];
      std::memcpy(q, pos0 + e * 6, sizeof(q));
      for (int i = 0; i < 3; ++i)
        if (q[i] == 32767 || q[i] == -32767) {
          ++clamped;
          break;
        }
      if (left == partSize)
        for (int i = 0; i < 3; ++i) {
          lo[i] = 32767;
          hi[i] = -32768;
        }
      for (int i = 0; i < 3; ++i) {
        if (q[i] < lo[i]) lo[i] = q[i];
        if (q[i] > hi[i]) hi[i] = q[i];
      }
      if (--left == 0 || v + 1 == part.count) {
        s32 glo[3], ghi[3];
        for (int i = 0; i < 3; ++i) {
          glo[i] = lo[i] + (s32)row[i];
          ghi[i] = hi[i] + (s32)row[i];
        }
        out.setPartBounds(pi, glo, ghi);
        ++pi;
        left = partSize;
      }
    }
  }
#if TYRA_SKIN_PROBE
  SKIN_PROBE_LAP(quant, pt);
#endif
  out.finishBounded();
  return clamped;
}

VectorCheck checkVectorStream(const Folded& folded, const SkelSkinPart& part,
                              StaPipCompactStream& out, float* pos4, float* nrm4) {
  VectorCheck c;
  const float step = out.box().step;
  const float inv = 1.0F / step;
  s32 oi[3];
  for (int i = 0; i < 3; ++i) oi[i] = (s32)originIndex(out.box().origin[i], step);
  // (a) the scalar encode of the scratch floats, written over them in place: the
  // expected positions (6 bytes) and normals (4 bytes) of each corner.
  for (u32 v = 0; v < part.count; ++v) {
    float* p = pos4 + 4 * v;
    float* n = nrm4 + 4 * v;
    s16 ep[3];
    for (int i = 0; i < 3; ++i) {
      s32 q = (s32)roundAway(p[i] * inv) - oi[i];
      q = q > 32767 ? 32767 : (q < -32767 ? -32767 : q);
      ep[i] = (s16)q;
    }
    s8 en[4];
    for (int i = 0; i < 3; ++i) {
      s32 q = (s32)roundAway(n[i] * 127.0F);
      q = q > 127 ? 127 : (q < -127 ? -127 : q);
      en[i] = (s8)q;
    }
    en[3] = 0;
    std::memcpy(p, ep, sizeof(ep));
    std::memcpy(n, en, sizeof(en));
  }
  // The vector stream against the expected bytes: bit for bit.
  const StaPipCompactLayout& lay = out.layout();
  s32 nslot = lay.indexOf(Compact::Attr::Normal);
  if (nslot < 0) nslot = lay.indexOf(Compact::Attr::EnvNormal);
  if (nslot < 0) {
    c.worst = 0xFFFFFFFFu;
    return c;
  }
  const u32 size = lay.packageSize;
  auto compare = [&](u32& exact, u32& worst) {
    for (u32 v = 0; v < part.count; ++v) {
      const u32 p = v / size, e = v % size;
      const u8* dp = out.run(p, 0) + e * 6;
      const u8* dn = out.run(p, (u8)nslot) + e * 4;
      s16 q[3];
      std::memcpy(q, dp, sizeof(q));
      s8 nn[4];
      std::memcpy(nn, dn, sizeof(nn));
      const s16* ep = reinterpret_cast<const s16*>(pos4 + 4 * v);
      const s8* en = reinterpret_cast<const s8*>(nrm4 + 4 * v);
      u32 bad = 0;
      for (int i = 0; i < 3; ++i) {
        const u32 dq = (u32)(q[i] > ep[i] ? q[i] - ep[i] : ep[i] - q[i]);
        const u32 dn2 = (u32)(nn[i] > en[i] ? nn[i] - en[i] : en[i] - nn[i]);
        if (dq > worst) worst = dq;
        if (dn2 > worst) worst = dn2;
        if (dq != 0 || dn2 != 0) bad = 1;
      }
      exact += bad;
    }
  };
  u32 exact = 0, worstVec = 0;
  compare(exact, worstVec);
  c.exact = exact;
  // The part boxes the vector pass set (quantizeIntoStream) must be the boxes
  // finish() makes from the stream's own integers. The stream is exact, so those
  // integers are the expected ones: finish() them and compare.
  const u32 np = out.partCount();
  std::vector<float> vectorBoxes(np * 6);
  if (np > 0 && out.partBoxes() != nullptr)
    std::memcpy(vectorBoxes.data(), out.partBoxes(), np * 6 * sizeof(float));
  out.finish();
  for (u32 i = 0; i < np * 6 && out.partBoxes() != nullptr; ++i)
    if (out.partBoxes()[i] != vectorBoxes[i]) ++c.boxMismatch;
  // (b) the scalar core on the same pose: its stream against the same expected
  // bytes (still in the scratch), within one LSB.
  const u32 scalar = skinIntoStream(folded, part, out);
  if (scalar == kSkinRefused) {
    c.worst = 0xFFFFFFFFu;
    return c;
  }
  u32 worst = 0, unused = 0;
  compare(unused, worst);
  c.worst = worst;
  c.clamped = scalar;
  return c;
}

namespace Fixed {

namespace {

typedef long long i64;

// 1 / sqrt(m) in Q16 for m = 1 + (i + 0.5) / 256, i in [0, 768): the table the
// normal's renormalisation reads for its mantissa (m is the normal's |n|^2 divided by
// the largest even power of two below it, so m is in [1, 4)). Built once, at first
// use; this is init arithmetic, not per corner.
struct RsqrtTable {
  s32 v[768];
  RsqrtTable() {
    for (int i = 0; i < 768; ++i) {
      const double m = 1.0 + (i + 0.5) / 256.0;
      v[i] = (s32)std::floor(65536.0 / std::sqrt(m) + 0.5);
    }
  }
};

const s32* rsqrtTable() {
  static const RsqrtTable table;
  return table.v;
}

// 2^k as a float, for the small exponents the fold uses (|k| <= 31).
float pow2f(int k) {
  float r = 1.0F;
  if (k >= 0) {
    for (int i = 0; i < k; ++i) r *= 2.0F;
  } else {
    for (int i = 0; i < -k; ++i) r *= 0.5F;
  }
  return r;
}

// Rounds v to the nearest whole number, half up, as the EE's truncating convert does
// (the fold's values are well inside the int32 range).
s32 roundS32(float v) {
  const float kLimit = 2147483520.0F;
  if (v > kLimit) return 2147483520;
  if (v < -kLimit) return -2147483520;
  return v < 0.0F ? -(s32)(-v + 0.5F) : (s32)(v + 0.5F);
}

// Quantizes v to a signed 16-bit value, rounding half up, with its exponent: the
// stored value is v x 2^(15 - e) and |v| < 2^e.
s16 toQ15(float v, s32 e) {
  const float s = v * pow2f(15 - (int)e);
  const s32 r = roundS32(s);
  if (r > 32767) return 32767;
  if (r < -32768) return -32768;
  return (s16)r;
}

// The smallest e >= 0 with max |v| < 2^e over the entries.
s32 exponentFor(const float* v, u32 count) {
  float mx = 0.0F;
  for (u32 i = 0; i < count; ++i)
    if (v[i] > mx) mx = v[i];
    else if (-v[i] > mx) mx = -v[i];
  s32 e = 0;
  while (mx >= pow2f((int)e) && e < 31) ++e;
  return e;
}

}  // namespace

void fold(const float* palette, u32 slots, const Compact::Box& bind, const Compact::Box& out,
          Fold& f, u32 qMax) {
  // Float throughout: the folds are per joint and per part, and the EE's double
  // arithmetic is software. The values stay far inside float's exact range.
  f.slots = slots < kMaxSlots ? slots : kMaxSlots;
  // The bind and output index origins (whole steps, as foldPalette takes them).
  s32 oiB[3];
  for (int i = 0; i < 3; ++i) {
    oiB[i] = (s32)originIndex(bind.origin[i], bind.step);
    f.oiOut[i] = (s32)originIndex(out.origin[i], out.step);
  }
  const float k = bind.step / out.step;
  static float ad[kMaxSlots][9], rd[kMaxSlots][9];
  static float bd[kMaxSlots][3];
  const float bindOrigin[3] = {bind.step * (float)oiB[0], bind.step * (float)oiB[1],
                               bind.step * (float)oiB[2]};
  for (u32 j = 0; j < f.slots; ++j) {
    const float* m = palette + 16 * j;
    for (int c = 0; c < 3; ++c)
      for (int r = 0; r < 3; ++r) {
        const float R = m[c * 4 + r];
        rd[j][c * 3 + r] = R;
        ad[j][c * 3 + r] = R * k;
      }
    // B_j = (R_j (bind step x origin index) + t_j) / output step - output origin index.
    for (int r = 0; r < 3; ++r) {
      float acc = m[12 + r];
      for (int c = 0; c < 3; ++c) acc += m[c * 4 + r] * bindOrigin[c];
      bd[j][r] = acc / out.step - (float)f.oiOut[r];
    }
  }
  // The pre-scale: the largest power of two (2^-8 .. 2^7) such that every joint term,
  // at the largest bind index, stays within int16 units. A term is |B| plus the sum of
  // |A| times qMax per axis.
  float bound = 0.0F;
  for (u32 j = 0; j < f.slots; ++j)
    for (int r = 0; r < 3; ++r) {
      float t = bd[j][r] < 0.0F ? -bd[j][r] : bd[j][r];
      for (int c = 0; c < 3; ++c) {
        const float a = ad[j][c * 3 + r];
        t += (a < 0.0F ? -a : a) * (float)qMax;
      }
      if (t > bound) bound = t;
    }
  s32 sc = 0;
  while (sc < 7 && bound * pow2f((int)sc + 1) <= 32767.0F) ++sc;
  while (sc > -8 && bound * pow2f((int)sc) > 32767.0F) --sc;
  f.expT = sc;
  f.expA = exponentFor(&ad[0][0], f.slots * 9);
  f.expR = exponentFor(&rd[0][0], f.slots * 9);
  for (u32 j = 0; j < f.slots; ++j) {
    for (int i = 0; i < 9; ++i) {
      f.a[j][i] = toQ15(ad[j][i], f.expA);
      f.r[j][i] = toQ15(rd[j][i], f.expR);
    }
    for (int r = 0; r < 3; ++r) {
      float b8 = bd[j][r] * 256.0F;
      if (b8 > 2147483520.0F) b8 = 2147483520.0F;
      if (b8 < -2147483520.0F) b8 = -2147483520.0F;
      f.b[j][r] = roundS32(b8);  // Q8 steps
    }
  }
}

u32 cornerFixed(const Fold& f, const s16* q, const s8* n, const u8* joints, const u8* weights,
                s16* outPos, s8* outNrm, u32* blendSaturations) {
  // The weights, Q15, summing to exactly 32768 (the largest takes the rounding).
  u32 wsum = 0;
  for (int k = 0; k < 4; ++k) wsum += weights[k];
  s32 w[4] = {0, 0, 0, 0};
  if (wsum > 0) {
    s32 tot = 0;
    int big = 0;
    for (int k = 0; k < 4; ++k) {
      w[k] = (s32)((weights[k] * 32768u + wsum / 2) / wsum);
      tot += w[k];
      if (weights[k] > weights[big]) big = k;
    }
    w[big] += 32768 - tot;
  }

  i64 sumPos[3] = {0, 0, 0};  // Q(15 + expT) of the output steps (joint terms x Q15 weights)
  i64 sumNrm[3] = {0, 0, 0};  // Q16 of the x127 normal (weights x rotated normal)
  u32 sat = 0;
  for (int k = 0; k < 4; ++k) {
    if (w[k] == 0) continue;
    const u32 j = joints[k];
    if (j >= f.slots) continue;
    for (int r = 0; r < 3; ++r) {
      // Output index in Q8: the A x q product (Q(15 - expA)) shifted to Q8 steps, + B.
      const i64 acc = (i64)f.a[j][0 * 3 + r] * q[0] + (i64)f.a[j][1 * 3 + r] * q[1] +
                      (i64)f.a[j][2 * 3 + r] * q[2];
      const i64 t8 = (((acc << 8) >> (15 - f.expA)) + f.b[j][r]);
      // The joint's term in units of 2^-expT steps (the int16 pre-scale).
      const s32 sh = 8 - f.expT;  // Q8 steps to Q(expT) units
      i64 th = sh > 0 ? (t8 + ((i64)1 << (sh - 1))) >> sh : (sh == 0 ? t8 : t8 << (-sh));
      if (th > 32767) {
        th = 32767;
        ++sat;
      } else if (th < -32768) {
        th = -32768;
        ++sat;
      }
      sumPos[r] += (i64)w[k] * th;
    }
    for (int r = 0; r < 3; ++r) {
      // The rotation of the normal, Q8 x127 units, weighted.
      const i64 acc = (i64)f.r[j][0 * 3 + r] * n[0] + (i64)f.r[j][1 * 3 + r] * n[1] +
                      (i64)f.r[j][2 * 3 + r] * n[2];
      const i64 p16 = (acc << 16) >> (15 - f.expR);
      sumNrm[r] += (i64)w[k] * p16;
    }
  }
  if (blendSaturations) *blendSaturations += sat;

  // Positions: the weighted sum is Q(15 + expT) of the steps.
  const s32 pe = 15 + f.expT;
  u32 clamped = 0;
  for (int r = 0; r < 3; ++r) {
    // b_j already carries the output origin index, so the sum is the output index.
    i64 v = (sumPos[r] + ((i64)1 << (pe - 1))) >> pe;
    if (v > 32767) {
      v = 32767;
      clamped = 1;
    } else if (v < -32767) {
      v = -32767;
      clamped = 1;
    }
    outPos[r] = (s16)v;
  }
  outPos[3] = 0;

  // Normals: N in Q16 (x127 units), renormalised with the table's 1/sqrt. The scale
  // cancels in N / sqrt(|N|^2) x 127.
  i64 N[3];
  for (int r = 0; r < 3; ++r) N[r] = (sumNrm[r] + 16384) >> 15;
  const i64 n2 = N[0] * N[0] + N[1] * N[1] + N[2] * N[2];
  if (n2 <= 0) {
    outNrm[0] = outNrm[1] = outNrm[2] = outNrm[3] = 0;
    return clamped;
  }
  const int lz = 63 - __builtin_clzll((unsigned long long)n2);
  const int e2 = lz & ~1;  // the largest even power of two not above n2
  const i64 mq = (n2 << 8) >> e2;  // m in Q8, [256, 1024)
  const s32 t = rsqrtTable()[mq - 256];
  const int sh = 16 + e2 / 2;  // 1/sqrt(n2) = t / 2^16 x 2^-(e2/2)
  for (int r = 0; r < 3; ++r) {
    const i64 val = N[r] * 127 * (i64)t;
    i64 v = (val + ((i64)1 << (sh - 1))) >> sh;
    if (v > 127) v = 127;
    if (v < -127) v = -127;
    outNrm[r] = (s8)v;
  }
  outNrm[3] = 0;
  return clamped;
}

}  // namespace Fixed

#if TYRA_SKIN_PROBE
// Logged every 20 calls (wrote or refused): the counts, then the mean ms per
// body for each phase (294912 COP0 ticks a ms) and the fold per pose.
void probeReport() {
  Probe& p = gProbe;
  if (p.calls == 0 || p.calls % 50 != 0 || p.calls == p.reported) return;
  p.reported = p.calls;
  const float perBody = 1.0F / (294912.0F * (float)(p.bodies ? p.bodies : 1));
  const float perPose = 1.0F / (294912.0F * (float)(p.folds ? p.folds : 1));
  char msg[320];
  snprintf(msg, sizeof msg,
           "SKINPROBE calls=%u early=%u palette=%u entries=%u wrote=%u refused=%u noTarget=%u poses=%u "
           "fold/pose=%.3f ms | per body: extent=%.3f box=%.3f corners=%.3f finish=%.3f ms",
           p.calls, p.early, p.paletteSize, p.entries, p.bodies, p.refused, p.noTarget, p.folds,
           (float)p.fold * perPose, (float)p.extent * perBody, (float)p.box * perBody,
           (float)p.corners * perBody, (float)p.finish * perBody);
  TYRA_LOG(msg);
}
#endif

}  // namespace SkelSkin
}  // namespace Tyra
