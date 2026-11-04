// The scalar skin core (renderer/3d/mesh/dynamic/skel_skin.hpp).

#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "skel_skin_mmi.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#include <cstdio>

#include "debug/debug.hpp"

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
bool gUseMmi = TYRA_SKIN_FIXED_MMI != 0;
bool gMmiChecked = false;

u32 skinIntoFixedRef(const Folded& folded, const float* palette, u32 slots,
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
#if TYRA_SKIN_PROBE
  const unsigned ppt = SKIN_PROBE_NOW();
#endif
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
#if TYRA_SKIN_PROBE
  unsigned fpt = SKIN_PROBE_NOW();
  SkelSkin::gProbe.fixPlace += fpt - ppt;
#endif
  static Fixed::Fold ff;  // one pose's fixed folds, consumed before this returns
  Fixed::fold(palette, slots, bindBox, ob, ff, qMax, ext->used);
#if TYRA_SKIN_PROBE
  const unsigned lpt = SKIN_PROBE_NOW();
  SkelSkin::gProbe.fixFold += lpt - fpt;
  unsigned uniq = 0;
  for (u32 v = 0; v < part.count; ++v)
    if (part.skinSource[v] >= v) ++uniq;
  SkelSkin::gProbe.fixUnique += uniq;
  SkelSkin::gProbe.fixRepeat += part.count - uniq;
  SkelSkin::gProbe.fixParts++;
#endif

  const u32 size = lay.packageSize;
  const u32 posBytes = Compact::elementBytes(Compact::Attr::Position);  // 6
  const u32 nrmBytes = Compact::elementBytes(lay.attrs[nslot]);        // 4
  const u8 nslotIndex = static_cast<u8>(nslot);
  u32 clamped = 0;
#if TYRA_SKIN_PROBE
  const unsigned lpStart = SKIN_PROBE_NOW();
#endif
  // The package base pointers, once per part: a repeat reads its source's element.
  const u32 packages = (part.count + size - 1) / size;
  const bool cached = packages <= 64;
  u8* posBase[64];
  u8* nrmBase[64];
  if (cached)
    for (u32 p = 0; p < packages; ++p) {
      posBase[p] = out.run(p, 0);
      nrmBase[p] = out.run(p, nslotIndex);
    }
  // The stream's part boxes are taken here, from the corners as they are written (the
  // same boxes finish() makes from the integers): a repeat's values are its source's.
  const u32 partSize = size / 3;  // the stream's parts (StaPipCompactStream::finish)
  if (partSize == 0) return SKIN_REFUSE();
  u32 row[4];
  Compact::rowFor(out.box(), row);
  s32 lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
  u32 pi = 0, left = partSize;
  for (u32 p = 0, v = 0; v < part.count; ++p) {
    u8* const pos0 = cached ? posBase[p] : out.run(p, 0);
    u8* const nrm0 = cached ? nrmBase[p] : out.run(p, nslotIndex);
    const u32 end = part.count - v > size ? v + size : part.count;
    for (u32 e = 0; v < end; ++v, ++e) {
      u8* dp = pos0 + e * posBytes;
      u8* dn = nrm0 + e * nrmBytes;
      const u32 src = part.skinSource[v];
      s16 q[3];
      if (src < v) {
        // A repeat has the same bind data as its source: it copies the source's bytes.
        const u32 sp_ = src / size, se = src - sp_ * size;
        const u8* sp = (cached ? posBase[sp_] : out.run(sp_, 0)) + se * posBytes;
        const u8* sn = (cached ? nrmBase[sp_] : out.run(sp_, nslotIndex)) + se * nrmBytes;
        std::memcpy(dp, sp, 6);  // a position is 6 bytes
        std::memcpy(dn, sn, 4);  // a normal is 4 bytes
        std::memcpy(q, dp, sizeof(q));
        for (int i = 0; i < 3; ++i)
          if (q[i] == 32767 || q[i] == -32767) {
            ++clamped;
            break;
          }
      } else {
        s16 qo[4];
        s8 n[4];
        clamped += Fixed::cornerFixed(ff, part.pos + 3 * v, part.nrm + 4 * v, part.joints + 4 * v,
                                      part.weights + 4 * v, qo, n, nullptr,
                                      part.infl ? part.infl + v : nullptr);
        std::memcpy(dp, qo, 6);
        std::memcpy(dn, n, 4);
        q[0] = qo[0];
        q[1] = qo[1];
        q[2] = qo[2];
      }
      // The part box of this corner's stream part, closed at its last corner.
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
  const unsigned fnStart = SKIN_PROBE_NOW();
  SkelSkin::gProbe.fixLoop += fnStart - lpStart;
#endif
  out.finishBounded();
#if TYRA_SKIN_PROBE
  SkelSkin::gProbe.fixFinish += SKIN_PROBE_NOW() - fnStart;
  if (SkelSkin::gProbe.fixParts >= 200) {
    SkelSkin::Probe& g = SkelSkin::gProbe;
    const float u = (float)(g.fixUnique ? g.fixUnique : 1);
    const float n = (float)(g.fixUnique + g.fixRepeat ? g.fixUnique + g.fixRepeat : 1);
    char msg[400];
    snprintf(msg, sizeof msg,
             "SKINFIXPROBE parts=%u unique/part=%.1f repeats/part=%.1f | cyc/unique: weights=%.0f "
             "pos=%.0f nrm=%.0f | cyc/part: place=%.0f fold=%.0f loop=%.0f finish=%.0f",
             g.fixParts, u / g.fixParts, (float)g.fixRepeat / g.fixParts,
             g.fixWeights / u, g.fixPos / u, g.fixNrm / u, (float)g.fixPlace / g.fixParts,
             (float)g.fixFold / g.fixParts, (float)g.fixLoop / g.fixParts,
             (float)g.fixFinish / g.fixParts);
    (void)n;
    TYRA_LOG(msg);
    g.fixParts = g.fixUnique = g.fixRepeat = g.fixPlace = g.fixFold = 0;
    g.fixWeights = g.fixPos = g.fixNrm = g.fixLoop = g.fixFinish = 0;
  }
#endif
  return clamped;
}

// The scatter map (SkinScatter): every unique corner's destinations in the stream, and the
// stream parts they fall in. Built from the part's skinSource (the first identical corner
// of each corner, itself unique), for one package size.
void buildScatter(const u32* skinSource, u32 count, u32 packageSize, SkinScatter& map) {
  map.size = packageSize;
  const u32 partSize = packageSize / 3;
  map.dstOff.assign(count + 1, 0);
  for (u32 v = 0; v < count; ++v) map.dstOff[skinSource[v] + 1]++;
  for (u32 u = 0; u < count; ++u) map.dstOff[u + 1] += map.dstOff[u];
  map.dst.assign(count, 0);
  std::vector<u32> fill(map.dstOff.begin(), map.dstOff.end() - 1);
  for (u32 v = 0; v < count; ++v) {
    const u32 u = skinSource[v];
    map.dst[fill[u]++] = ((v / packageSize) << 16) | (v % packageSize);
  }
  map.rngOff.assign(count + 1, 0);
  map.rng.clear();
  for (u32 u = 0; u < count; ++u) {
    map.rngOff[u] = (u32)map.rng.size();
    s32 last = -1;
    for (u32 d = map.dstOff[u]; d < map.dstOff[u + 1]; ++d) {
      const u32 dv = map.dst[d];
      const u32 v = (dv >> 16) * packageSize + (dv & 0xFFFFu);
      const s32 r = (s32)(v / partSize);
      if (r != last) {
        map.rng.push_back((u32)r);
        last = r;
      }
    }
  }
  map.rngOff[count] = (u32)map.rng.size();
}

// The corner-wise stream pass as it was: a unique corner is blended and written, a repeat
// copies its source's stream bytes. skinIntoFixed (the scatter) writes the same bytes; a
// first pass per process checks that they are equal (SkinScatterCheck).
namespace {
constexpr u32 kScatterRanges = 128;  // stream parts a scatter pass tracks boxes for
constexpr u32 kScatterPackages = 64;
bool gScatterChecked = false;
}  // namespace

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
  const u32 size = lay.packageSize;
  const u32 partSize = size / 3;  // the stream's parts (StaPipCompactStream::finish)
  const u32 packages = (part.count + size - 1) / size;
  const u32 ranges = partSize == 0 ? 0 : (part.count + partSize - 1) / partSize;
  if (partSize == 0 || packages > kScatterPackages || ranges > kScatterRanges)
    return skinIntoFixedRef(folded, palette, slots, bindBox, part, out);
#if TYRA_SKIN_PROBE
  const unsigned spt0 = SKIN_PROBE_NOW();
#endif
  s32 oi[3];
  if (!posePlace(folded, part, out, oi)) return SKIN_REFUSE();
#if TYRA_SKIN_PROBE
  const unsigned spt1 = SKIN_PROBE_NOW();
  SkelSkin::gProbe.fixPlace += spt1 - spt0;
#endif
  // The folds' output box is the one posePlace just set: its origin is oi steps.
  Compact::Box ob;
  ob.step = folded.out.step;
  for (int i = 0; i < 3; ++i) ob.origin[i] = (float)oi[i] * ob.step;
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
  Fixed::fold(palette, slots, bindBox, ob, ff, qMax, ext->used);

#if TYRA_SKIN_PROBE
  const unsigned sft = SKIN_PROBE_NOW();
  SkelSkin::gProbe.fixFold += sft - spt1;
#endif
  const u32 nrmBytes = Compact::elementBytes(lay.attrs[nslot]);  // 4
  SkinScatter localMap;
  SkinScatter* map = part.scatter;
  if (map == nullptr) {
    buildScatter(part.skinSource, part.count, size, localMap);
    map = &localMap;
  } else if (map->size != size) {
    buildScatter(part.skinSource, part.count, size, *map);
  }
  u8* posBase[kScatterPackages];
  u8* nrmBase[kScatterPackages];
  for (u32 p = 0; p < packages; ++p) {
    posBase[p] = out.run(p, 0);
    nrmBase[p] = out.run(p, static_cast<u8>(nslot));
  }
  u32 row[4];
  Compact::rowFor(out.box(), row);
  s32 lo[kScatterRanges][3], hi[kScatterRanges][3];
  for (u32 r = 0; r < ranges; ++r)
    for (int i = 0; i < 3; ++i) {
      lo[r][i] = 32767;
      hi[r][i] = -32768;
    }
  // Only unique corners are blended: a repeat's bytes are its source's, scattered to every
  // destination of the unique. A clamped unique counts once, and a repeat whose value sits
  // at the int16 limit counts as the old per-corner loop did.
  u32 clamped = 0;
#if TYRA_SKIN_PROBE
  const unsigned slp = SKIN_PROBE_NOW();
  u32 uniqueCount = 0;
#endif
  for (u32 u = 0; u < part.count; ++u) {
    if (part.skinSource[u] != u) continue;
#if TYRA_SKIN_PROBE
    ++uniqueCount;
#endif
    s16 qo[4];
    s8 n[4];
#if TYRA_SKIN_PROBE
    const unsigned blp = SKIN_PROBE_NOW();
#endif
    const u32 c = Fixed::cornerFixed(ff, part.pos + 3 * u, part.nrm + 4 * u, part.joints + 4 * u,
                                     part.weights + 4 * u, qo, n, nullptr,
                                     part.infl ? part.infl + u : nullptr);
#if TYRA_SKIN_PROBE
    SkelSkin::gProbe.fixPos += SKIN_PROBE_NOW() - blp;
#endif
    const u32 d0 = map->dstOff[u], d1 = map->dstOff[u + 1];
    const bool atLimit = qo[0] == 32767 || qo[0] == -32767 || qo[1] == 32767 ||
                         qo[1] == -32767 || qo[2] == 32767 || qo[2] == -32767;
    clamped += c + (atLimit ? d1 - d0 - 1 : 0);
#if TYRA_SKIN_PROBE
    const unsigned sct = SKIN_PROBE_NOW();
#endif
    // A position element is 6 bytes at an even offset (halfword stores), a normal 4 bytes
    // at a 4-byte offset (one word store): both are aligned, the stream block is.
    const s16 q0 = qo[0], q1 = qo[1], q2 = qo[2];
    u32 nword;
    std::memcpy(&nword, n, 4);
    for (u32 d = d0; d < d1; ++d) {
      const u32 dv = map->dst[d];
      const u32 p = dv >> 16, e = dv & 0xFFFFu;
      s16* pd = reinterpret_cast<s16*>(posBase[p] + e * 6);
      pd[0] = q0;
      pd[1] = q1;
      pd[2] = q2;
      if (nrmBytes == 4)
        *reinterpret_cast<u32*>(nrmBase[p] + e * 4) = nword;
      else
        std::memcpy(nrmBase[p] + e * nrmBytes, n, 4);
    }
#if TYRA_SKIN_PROBE
    SkelSkin::gProbe.fixScatter += SKIN_PROBE_NOW() - sct;
#endif
    for (u32 k = map->rngOff[u]; k < map->rngOff[u + 1]; ++k) {
      const u32 r = map->rng[k];
      for (int i = 0; i < 3; ++i) {
        if (qo[i] < lo[r][i]) lo[r][i] = qo[i];
        if (qo[i] > hi[r][i]) hi[r][i] = qo[i];
      }
    }
  }
  for (u32 r = 0; r < ranges; ++r) {
    s32 glo[3], ghi[3];
    for (int i = 0; i < 3; ++i) {
      glo[i] = lo[r][i] + (s32)row[i];
      ghi[i] = hi[r][i] + (s32)row[i];
    }
    out.setPartBounds(r, glo, ghi);
  }
  out.finishBounded();
#if TYRA_SKIN_PROBE
  {
    SkelSkin::Probe& g = SkelSkin::gProbe;
    g.fixLoop += SKIN_PROBE_NOW() - slp;
    g.fixUnique += uniqueCount;
    g.fixRepeat += part.count - uniqueCount;
    g.fixParts++;
    if (g.fixParts >= 200) {
      const float u = (float)(g.fixUnique ? g.fixUnique : 1);
      char msg[400];
      snprintf(msg, sizeof msg,
               "SKINFIXPROBE parts=%u unique/part=%.1f repeats/part=%.1f | cyc/unique: blend=%.0f "
               "scatter=%.0f | cyc/part: place=%.0f fold=%.0f loop=%.0f",
               g.fixParts, u / g.fixParts, (float)g.fixRepeat / g.fixParts, g.fixPos / u,
               g.fixScatter / u, (float)g.fixPlace / g.fixParts,
               (float)g.fixFold / g.fixParts, (float)g.fixLoop / g.fixParts);
      g.fixScatter = 0;
      g.fixPos = 0;
      TYRA_LOG(msg);
      g.fixParts = g.fixUnique = g.fixRepeat = g.fixPlace = g.fixFold = 0;
      g.fixWeights = g.fixPos = g.fixNrm = g.fixLoop = g.fixFinish = 0;
    }
  }
#endif
#if defined(_EE)
  if (gUseMmi && !gMmiChecked) {
    // The first pass per process: the MMI kernel's output against the scalar blend's, corner
    // by corner over this part with the same fold. The scatter writes the kernel's output
    // as it is, so equal outputs give equal stream bytes (SKINSCATTER checks the scatter).
    gMmiChecked = true;
    u32 mismatches = 0, first = 0;
    s16 firstScalar[4] = {0, 0, 0, 0}, firstKernel[4] = {0, 0, 0, 0};
    s8 firstScalarN[4] = {0, 0, 0, 0}, firstKernelN[4] = {0, 0, 0, 0};
    for (u32 v = 0; v < part.count; ++v) {
      const u8* jj = part.joints + 4 * v;
      const u8* ww = part.weights + 4 * v;
      const s16* qq = part.pos + 3 * v;
      const s8* nn = part.nrm + 4 * v;
      const Fixed::CornerInfl ci = Fixed::prepCorner(jj, ww);
      s16 ps[4] = {0, 0, 0, 0}, pk[4] = {0, 0, 0, 0};
      s8 ns[4] = {0, 0, 0, 0}, nk[4] = {0, 0, 0, 0};
      gUseMmi = false;
      const u32 cs = Fixed::cornerFixed(ff, qq, nn, jj, ww, ps, ns, nullptr, &ci);
      gUseMmi = true;
      const u32 ck = Fixed::cornerMmi(ff, qq, nn, jj, ww, ci, pk, nk);
      const bool same = cs == ck && std::memcmp(ps, pk, 6) == 0 && std::memcmp(ns, nk, 3) == 0;
      if (!same) {
        if (mismatches == 0) {
          first = v;
          std::memcpy(firstScalar, ps, sizeof ps);
          std::memcpy(firstKernel, pk, sizeof pk);
          std::memcpy(firstScalarN, ns, sizeof ns);
          std::memcpy(firstKernelN, nk, sizeof nk);
        }
        ++mismatches;
      }
    }
    char msg[480];
    if (mismatches == 0) {
      snprintf(msg, sizeof msg, "SKINMMI self-check: byte-equal to the scalar blend (%u corners)",
               (unsigned)part.count);
    } else {
      snprintf(msg, sizeof msg,
               "SKINMMI self-check: MISMATCH %u of %u corners; first corner %u: scalar pos "
               "%d %d %d nrm %d %d %d, kernel pos %d %d %d nrm %d %d %d",
               (unsigned)mismatches, (unsigned)part.count, (unsigned)first, (int)firstScalar[0],
               (int)firstScalar[1], (int)firstScalar[2], (int)firstScalarN[0],
               (int)firstScalarN[1], (int)firstScalarN[2], (int)firstKernel[0],
               (int)firstKernel[1], (int)firstKernel[2], (int)firstKernelN[0],
               (int)firstKernelN[1], (int)firstKernelN[2]);
    }
    TYRA_LOG(msg);
  }
#endif
  if (!gScatterChecked) {
    // The first pass per process: the corner-wise pass must write the same block, byte
    // for byte, and count the same clamps (logged once: the scatter self-check).
    gScatterChecked = true;
    const u32 bytes = out.qwords() * 16;
    u8* saved = new (std::nothrow) u8[bytes];
    if (saved != nullptr) {
      std::memcpy(saved, out.data(), bytes);
      const u32 refClamped = skinIntoFixedRef(folded, palette, slots, bindBox, part, out);
      const bool same = refClamped == clamped && std::memcmp(saved, out.data(), bytes) == 0;
      char msg[160];
      snprintf(msg, sizeof msg, "SKINSCATTER self-check: %s (%u bytes, clamps %u / %u)",
               same ? "byte-equal to the corner-wise pass" : "MISMATCH", (unsigned)bytes,
               (unsigned)clamped, (unsigned)refClamped);
      TYRA_LOG(msg);
      delete[] saved;
    }
  }
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

// Quantizes v to a signed 16-bit value, rounding half up: `scale` is 2^(15 - e), so the
// stored value is v x scale and |v| < 2^e.
s16 toQ15(float v, float scale) {
  const s32 r = roundS32(v * scale);
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
          Fold& f, u32 qMax, unsigned long long used) {
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
    // A slot no corner of the part weights is never read: it folds to zeros.
    if (((used >> j) & 1ULL) == 0) {
      for (int i = 0; i < 9; ++i) ad[j][i] = rd[j][i] = 0.0F;
      for (int r = 0; r < 3; ++r) bd[j][r] = 0.0F;
      continue;
    }
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
  const float scaleA = pow2f(15 - (int)f.expA);
  const float scaleR = pow2f(15 - (int)f.expR);
  float maxSum = 0.0F;  // the largest sum of |A| entries of one row, times qMax
  for (u32 j = 0; j < f.slots; ++j) {
    if (((used >> j) & 1ULL) == 0) continue;
    for (int i = 0; i < 9; ++i) {
      f.a[j][i] = toQ15(ad[j][i], scaleA);
      f.r[j][i] = toQ15(rd[j][i], scaleR);
    }
    for (int r = 0; r < 3; ++r) {
      float t = 0.0F;
      for (int c = 0; c < 3; ++c) {
        const s16 a = f.a[j][c * 3 + r];
        t += (float)(a < 0 ? -a : a);
      }
      if (t * (float)qMax > maxSum) maxSum = t * (float)qMax;
    }
    for (int r = 0; r < 3; ++r) {
      float b8 = bd[j][r] * 256.0F;
      if (b8 > 2147483520.0F) b8 = 2147483520.0F;
      if (b8 < -2147483520.0F) b8 = -2147483520.0F;
      f.b[j][r] = roundS32(b8);  // Q8 steps
    }
    // The MMI kernel's operands: the columns in lanes 0, 2, 4 (16-byte aligned), the joint
    // translations as words.
    for (int c = 0; c < 3; ++c) {
      for (int i = 0; i < 8; ++i) f.aV[j][c][i] = f.rV[j][c][i] = 0;
      f.aV[j][c][0] = f.a[j][c * 3 + 0];
      f.aV[j][c][2] = f.a[j][c * 3 + 1];
      f.aV[j][c][4] = f.a[j][c * 3 + 2];
      f.rV[j][c][0] = f.r[j][c * 3 + 0];
      f.rV[j][c][2] = f.r[j][c * 3 + 1];
      f.rV[j][c][4] = f.r[j][c * 3 + 2];
    }
    for (int r = 0; r < 3; ++r) f.bW[j][r] = f.b[j][r];
    f.bW[j][3] = 0;
  }
  // noSat: with the quantised entries, every joint term of every row fits int16 for any
  // bind index up to qMax (the blend's own shifts, one unit of slack), so the blend's
  // per-term clamp and its counter can never fire (cornerNarrow's fast path).
  f.noSat = f.expA <= 7 && f.expT < 8;
  for (u32 j = 0; j < f.slots && f.noSat; ++j) {
    if (((used >> j) & 1ULL) == 0) continue;
    for (int r = 0; r < 3 && f.noSat; ++r) {
      const i64 sumAbs = (i64)std::abs((int)f.a[j][r]) + std::abs((int)f.a[j][3 + r]) +
                         std::abs((int)f.a[j][6 + r]);
      const i64 accMax = sumAbs * (i64)qMax;
      const i64 t8 = (accMax >> (7 - f.expA)) + 1 + (i64)std::abs(f.b[j][r]);
      const i64 shT = 8 - f.expT;  // > 0 (checked above)
      const i64 th = ((t8 + (i64)(1LL << (shT - 1))) >> shT) + 1;
      if (th > 32767) f.noSat = false;
    }
  }
  // The 32-bit blend is exact when no sum can overflow: a row of A times the largest bind
  // index (below 2^31), the pre-scaled terms in Q8 below 2^31 (the bound), and a rotation
  // exponent small enough for the normal's weighted sum.
  f.narrow = maxSum < 2000000000.0F && bound < 8000000.0F && f.expR <= 3;
  // narrowN: a normal term, (row . n) x 16 >> (15 - expR), for any normal (|n| <= 127 per
  // axis) and the folded rows, plus one unit of slack for the floor, fits int16.
  f.narrowN = f.expR <= 3;
  for (u32 j = 0; j < f.slots && f.narrowN; ++j) {
    if (((used >> j) & 1ULL) == 0) continue;
    for (int r = 0; r < 3 && f.narrowN; ++r) {
      const i64 sumAbs = (i64)std::abs((int)f.r[j][r]) + std::abs((int)f.r[j][3 + r]) +
                         std::abs((int)f.r[j][6 + r]);
      const i64 term = (sumAbs * 127 * 16) >> (15 - f.expR);
      if (term + 1 > 32767) f.narrowN = false;
    }
  }
}

u32 cornerWide(const Fold& f, const s16* q, const s8* n, const u8* joints, const u8* weights,
               s16* outPos, s8* outNrm, u32* blendSaturations) {
#if TYRA_SKIN_PROBE
  const unsigned pt0 = SkelSkin::probeTicks();
#endif
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

#if TYRA_SKIN_PROBE
  const unsigned pt1 = SkelSkin::probeTicks();
  SkelSkin::gProbe.fixWeights += pt1 - pt0;
#endif
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
#if TYRA_SKIN_PROBE
  const unsigned pt2 = SkelSkin::probeTicks();
  SkelSkin::gProbe.fixPos += pt2 - pt1;
#endif

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
#if TYRA_SKIN_PROBE
    SkelSkin::gProbe.fixNrm += SkelSkin::probeTicks() - pt2;
#endif
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
#if TYRA_SKIN_PROBE
  SkelSkin::gProbe.fixNrm += SkelSkin::probeTicks() - pt2;
#endif
  return clamped;
}

// The weights of a corner whose u8 weights sum to 255 (the format's rule), as Q15 from
// a table: round(w x 32768 / 255). Other sums take the division in cornerWide.
struct WeightTable {
  s32 v[256];
  WeightTable() {
    for (int w = 0; w < 256; ++w) v[w] = (w * 32768 * 2 + 255) / 510;
  }
};

const s32* weightTable() {
  static const WeightTable table;
  return table.v;
}

void weightsQ15(const u8* weights, s32* w) {
  const u32 wsum = (u32)weights[0] + weights[1] + weights[2] + weights[3];
  if (wsum == 255) {
    const s32* t = weightTable();
    s32 tot = 0;
    int big = 0;
    for (int k = 0; k < 4; ++k) {
      w[k] = t[weights[k]];
      tot += w[k];
      if (weights[k] > weights[big]) big = k;
    }
    w[big] += 32768 - tot;
    return;
  }
  for (int k = 0; k < 4; ++k) w[k] = 0;
  if (wsum == 0) return;
  s32 tot = 0;
  int big = 0;
  for (int k = 0; k < 4; ++k) {
    w[k] = (s32)((weights[k] * 32768u + wsum / 2) / wsum);
    tot += w[k];
    if (weights[k] > weights[big]) big = k;
  }
  w[big] += 32768 - tot;
}

// The 32-bit corner (Fold::narrow): the same arithmetic as cornerWide, with every sum
// held in 32 bits. The normal is kept in Q4 (16 x x127), which the renormalisation
// reads as it is.
u32 cornerNarrowRef(const Fold& f, const s16* q, const s8* n, const u8* joints,
                    const u8* weights, s16* outPos, s8* outNrm, u32* blendSaturations) {
#if TYRA_SKIN_PROBE
  const unsigned pt0 = SkelSkin::probeTicks();
#endif
  s32 w[4];
  weightsQ15(weights, w);
#if TYRA_SKIN_PROBE
  const unsigned pt1 = SkelSkin::probeTicks();
  SkelSkin::gProbe.fixWeights += pt1 - pt0;
#endif
  s32 sumPos[3] = {0, 0, 0};
  s32 sumNrm[3] = {0, 0, 0};
  u32 sat = 0;
  const s32 shT = 8 - f.expT;  // Q8 steps to the term's units
  const s32 shA = 7 - f.expA;  // (acc << 8) >> (15 - expA) = acc x 2^(expA - 7)
  for (int k = 0; k < 4; ++k) {
    if (w[k] == 0) continue;
    const u32 j = joints[k];
    if (j >= f.slots) continue;
    const s16* A = f.a[j];
    for (int r = 0; r < 3; ++r) {
      const s32 acc = (s32)A[r] * q[0] + (s32)A[3 + r] * q[1] + (s32)A[6 + r] * q[2];
      const s32 t8 = (shA >= 0 ? (acc >> shA) : (acc << -shA)) + f.b[j][r];
      s32 th = shT > 0 ? (t8 + (1 << (shT - 1))) >> shT : (shT == 0 ? t8 : t8 << (-shT));
      if (th > 32767) {
        th = 32767;
        ++sat;
      } else if (th < -32768) {
        th = -32768;
        ++sat;
      }
      sumPos[r] += w[k] * th;  // |sum| <= 32768 x 32767: the weights sum to 32768
    }
    for (int r = 0; r < 3; ++r) {
      const s32 acc = (s32)f.r[j][r] * n[0] + (s32)f.r[j][3 + r] * n[1] + (s32)f.r[j][6 + r] * n[2];
      sumNrm[r] += w[k] * ((acc * 16) >> (15 - f.expR));  // Q4 x127 units
    }
  }
  if (blendSaturations) *blendSaturations += sat;
#if TYRA_SKIN_PROBE
  const unsigned pt2 = SkelSkin::probeTicks();
  SkelSkin::gProbe.fixPos += pt2 - pt1;
#endif
  const s32 pe = 15 + f.expT;
  u32 clamped = 0;
  for (int r = 0; r < 3; ++r) {
    s32 v = (sumPos[r] + (1 << (pe - 1))) >> pe;
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
  s32 N[3];
  for (int r = 0; r < 3; ++r) N[r] = (sumNrm[r] + 16384) >> 15;
  const s32 n2 = N[0] * N[0] + N[1] * N[1] + N[2] * N[2];
  // A blended normal shorter than about 8 x127 units cancelled: its Q4 sums lose the
  // direction, so that corner takes the 64-bit blend (rare: opposed joints only).
  if (n2 < 16384) return cornerWide(f, q, n, joints, weights, outPos, outNrm, nullptr);
  if (n2 <= 0) {
    outNrm[0] = outNrm[1] = outNrm[2] = outNrm[3] = 0;
#if TYRA_SKIN_PROBE
    SkelSkin::gProbe.fixNrm += SkelSkin::probeTicks() - pt2;
#endif
    return clamped;
  }
  const int lz = 31 - __builtin_clz((unsigned)n2);
  const int e2 = lz & ~1;  // the largest even power of two not above n2
  const s32 mq = e2 >= 8 ? (n2 >> (e2 - 8)) : (n2 << (8 - e2));  // m in Q8, [256, 1024)
  const s32 t = rsqrtTable()[mq - 256];
  const int sh = 16 + e2 / 2;
  for (int r = 0; r < 3; ++r) {
    const s32 u = (N[r] * t + 64) >> 7;  // the product in Q(-7)
    s32 v = (u * 127 + (1 << (sh - 8))) >> (sh - 7);
    if (v > 127) v = 127;
    if (v < -127) v = -127;
    outNrm[r] = (s8)v;
  }
  outNrm[3] = 0;
#if TYRA_SKIN_PROBE
  SkelSkin::gProbe.fixNrm += SkelSkin::probeTicks() - pt2;
#endif
  return clamped;
}

// The bind-time influences: the nonzero weights in slot order (the integer sums of the
// blend are order-free, so the same terms give the same blend), with their joints.
CornerInfl prepCorner(const u8* joints, const u8* weights) {
  CornerInfl ci;
  s32 w[4];
  weightsQ15(weights, w);
  ci.n = 0;
  for (int k = 0; k < 4; ++k) {
    ci.w[k] = 0;
    ci.j[k] = 0;
  }
  for (int k = 0; k < 4; ++k) {
    if (w[k] == 0) continue;
    ci.w[ci.n] = (u16)w[k];
    ci.j[ci.n] = joints[k];
    ++ci.n;
  }
  return ci;
}

// The blend's tail, shared by the generic and the fast corner: the positions round out
// of the Q(15 + expT) sums and clamp to the box (the out-of-box clamp), the normal
// renormalises through the rsqrt table. A blended normal shorter than about 8 x127 units
// cancelled: that corner takes the 64-bit blend (cornerWide).
u32 narrowTail(const Fold& f, const s32 sumPos[3], const s32 sumNrm[3], const s16* q,
               const s8* n, const u8* joints, const u8* weights, s16* outPos, s8* outNrm) {
  const s32 pe = 15 + f.expT;
  u32 clamped = 0;
  for (int r = 0; r < 3; ++r) {
    s32 v = (sumPos[r] + (1 << (pe - 1))) >> pe;
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
  s32 N[3];
  for (int r = 0; r < 3; ++r) N[r] = (sumNrm[r] + 16384) >> 15;
  const s32 n2 = N[0] * N[0] + N[1] * N[1] + N[2] * N[2];
  if (n2 < 16384) return cornerWide(f, q, n, joints, weights, outPos, outNrm, nullptr);
  if (n2 <= 0) {
    outNrm[0] = outNrm[1] = outNrm[2] = outNrm[3] = 0;
    return clamped;
  }
  const int lz = 31 - __builtin_clz((unsigned)n2);
  const int e2 = lz & ~1;
  const s32 mq = e2 >= 8 ? (n2 >> (e2 - 8)) : (n2 << (8 - e2));
  const s32 t = rsqrtTable()[mq - 256];
  const int sh = 16 + e2 / 2;
  for (int r = 0; r < 3; ++r) {
    const s32 u = (N[r] * t + 64) >> 7;
    s32 v = (u * 127 + (1 << (sh - 8))) >> (sh - 7);
    if (v > 127) v = 127;
    if (v < -127) v = -127;
    outNrm[r] = (s8)v;
  }
  outNrm[3] = 0;
  return clamped;
}

// The same blend as cornerNarrowRef, over the precomputed influences: no weight
// conversion per pose, only the corner's nonzero terms. The per-term clamp stays here
// for folds that cannot rule it out (the fast blend below otherwise).
u32 cornerNarrow(const Fold& f, const s16* q, const s8* n, const u8* joints, const u8* weights,
                 const CornerInfl& ci, s16* outPos, s8* outNrm, u32* blendSaturations) {
  s32 sumPos[3] = {0, 0, 0};
  s32 sumNrm[3] = {0, 0, 0};
  u32 sat = 0;
  const s32 shT = 8 - f.expT;  // Q8 steps to the term's units
  const s32 shA = 7 - f.expA;  // (acc << 8) >> (15 - expA) = acc x 2^(expA - 7)
  for (int k = 0; k < ci.n; ++k) {
    const s32 wk = ci.w[k];
    const u32 j = ci.j[k];
    if (j >= f.slots) continue;
    const s16* A = f.a[j];
    for (int r = 0; r < 3; ++r) {
      const s32 acc = (s32)A[r] * q[0] + (s32)A[3 + r] * q[1] + (s32)A[6 + r] * q[2];
      const s32 t8 = (shA >= 0 ? (acc >> shA) : (acc << -shA)) + f.b[j][r];
      s32 th = shT > 0 ? (t8 + (1 << (shT - 1))) >> shT : (shT == 0 ? t8 : t8 << (-shT));
      if (th > 32767) {
        th = 32767;
        ++sat;
      } else if (th < -32768) {
        th = -32768;
        ++sat;
      }
      sumPos[r] += wk * th;
    }
    for (int r = 0; r < 3; ++r) {
      const s32 acc = (s32)f.r[j][r] * n[0] + (s32)f.r[j][3 + r] * n[1] + (s32)f.r[j][6 + r] * n[2];
      sumNrm[r] += wk * ((acc * 16) >> (15 - f.expR));
    }
  }
  if (blendSaturations) *blendSaturations += sat;
  return narrowTail(f, sumPos, sumNrm, q, n, joints, weights, outPos, outNrm);
}

// The fast blend, for a fold with noSat: every term fits int16, so there is no clamp and
// no counter, and the shifts are the same for every term (shA >= 0, shT > 0), so the loops
// carry no per-term branch. N is the corner's term count: 1 and 2 are instantiated, 0 runs
// ci.n terms. A single term's weight is 32768 exactly (prepCorner), so it is a shift.
template <int N>
u32 cornerNarrowFast(const Fold& f, const s16* q, const s8* n, const u8* joints,
                     const u8* weights, const CornerInfl& ci, s16* outPos, s8* outNrm) {
  s32 sumPos[3] = {0, 0, 0};
  s32 sumNrm[3] = {0, 0, 0};
  const s32 shT = 8 - f.expT;  // > 0 (the fold's noSat)
  const s32 shA = 7 - f.expA;  // >= 0 (the fold's noSat)
  const s32 shR = 15 - f.expR;
  const s32 roundT = 1 << (shT - 1);
  const int count = N != 0 ? N : ci.n;
  for (int k = 0; k < count; ++k) {
    const u32 j = ci.j[k];
    if (j >= f.slots) continue;
    const s16* A = f.a[j];
    const s32 wk = ci.w[k];
    for (int r = 0; r < 3; ++r) {
      const s32 acc = (s32)A[r] * q[0] + (s32)A[3 + r] * q[1] + (s32)A[6 + r] * q[2];
      const s32 th = ((acc >> shA) + f.b[j][r] + roundT) >> shT;
      if (N == 1)
        sumPos[r] += th * 32768;
      else
        sumPos[r] += wk * th;
    }
    for (int r = 0; r < 3; ++r) {
      const s32 acc = (s32)f.r[j][r] * n[0] + (s32)f.r[j][3 + r] * n[1] + (s32)f.r[j][6 + r] * n[2];
      const s32 term = (acc * 16) >> shR;
      if (N == 1)
        sumNrm[r] += term * 32768;
      else
        sumNrm[r] += wk * term;
    }
  }
  return narrowTail(f, sumPos, sumNrm, q, n, joints, weights, outPos, outNrm);
}

#if defined(_EE)
// The EE kernel's dot products: the sum over three lane pairs (lanes 0, 2, 4) of a x b,
// as the four words PMFHL.LW returns (LO0, HI0, LO2, HI2). PMADDH does the
// multiply-accumulates; the operands are 16-byte aligned (lq). The shifts, rounds and
// sums between them are scalar integer ops: the EE's variable word shift (PSRAVW) moves
// words 0 and 2 only, so it cannot shift three rows at once.
static inline void mmiDot3(const s16* a0, const s16* a1, const s16* a2, const s16* b0,
                           const s16* b1, const s16* b2, s32* out) {
  asm volatile(
      "lq         $8, 0x00(%[a0])     \n\t"
      "lq         $9, 0x00(%[b0])     \n\t"
      "lq         $10, 0x00(%[a1])    \n\t"
      "lq         $11, 0x00(%[b1])    \n\t"
      "lq         $12, 0x00(%[a2])    \n\t"
      "lq         $13, 0x00(%[b2])    \n\t"
      "pmthl.lw   $0                  \n\t"
      "pmaddh     $0, $8, $9          \n\t"
      "pmaddh     $0, $10, $11        \n\t"
      "pmaddh     $0, $12, $13        \n\t"
      "pmfhl.lw   $14                 \n\t"
      "sq         $14, 0x00(%[out])   \n\t"
      :
      : [a0] "r"(a0), [a1] "r"(a1), [a2] "r"(a2), [b0] "r"(b0), [b1] "r"(b1), [b2] "r"(b2),
        [out] "r"(out)
      : "$8", "$9", "$10", "$11", "$12", "$13", "$14", "memory");
}

// One lane pair: mmiDot3 with a single product.
static inline void mmiDot1(const s16* a0, const s16* b0, s32* out) {
  asm volatile(
      "lq         $8, 0x00(%[a0])     \n\t"
      "lq         $9, 0x00(%[b0])     \n\t"
      "pmthl.lw   $0                  \n\t"
      "pmaddh     $0, $8, $9          \n\t"
      "pmfhl.lw   $10                 \n\t"
      "sq         $10, 0x00(%[out])   \n\t"
      :
      : [a0] "r"(a0), [b0] "r"(b0), [out] "r"(out)
      : "$8", "$9", "$10", "memory");
}

// The MMI kernel on the EE: cornerMmi's dataflow, with the dot products on PMADDH. The
// corner's q and n lane vectors are built here (three stores each).
u32 cornerMmiEE(const Fold& f, const s16* q, const s8* n, const u8* joints, const u8* weights,
                const CornerInfl& ci, s16* outPos, s8* outNrm) {
  alignas(16) s16 qV[3][8];
  alignas(16) s16 nV[3][8];
  for (int c = 0; c < 3; ++c)
    for (int i = 0; i < 8; ++i) qV[c][i] = nV[c][i] = 0;
  for (int c = 0; c < 3; ++c) {
    qV[c][0] = qV[c][2] = qV[c][4] = q[c];
    nV[c][0] = nV[c][2] = nV[c][4] = n[c];
  }
  const int shA = 7 - f.expA;   // >= 0 (noSat)
  const int shT = 8 - f.expT;   // > 0 (noSat)
  const int shR = 15 - f.expR;  // >= 12 (expR <= 3)
  const s32 roundT = 1 << (shT - 1);
  s32 sumPos[3] = {0, 0, 0};
  s32 sumNrm[3] = {0, 0, 0};
  for (int k = 0; k < ci.n; ++k) {
    const u32 j = ci.j[k];
    if (j >= f.slots) continue;
    const s32 wk = ci.w[k];
    alignas(16) s16 wV[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    wV[0] = wV[2] = wV[4] = (s16)wk;
    alignas(16) s32 p[4];
    // Positions: the rows, the pre-scale, then the weighted sum over the joints.
    alignas(16) s32 rows[4];
    mmiDot3(qV[0], qV[1], qV[2], f.aV[j][0], f.aV[j][1], f.aV[j][2], rows);
    alignas(16) s16 tV[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    s32 th[3];
    for (int r = 0; r < 3; ++r) {
      th[r] = ((rows[r] >> shA) + f.bW[j][r] + roundT) >> shT;
      tV[2 * r] = (s16)th[r];
    }
    if (wk == 32768) {
      for (int r = 0; r < 3; ++r) sumPos[r] += th[r] * 32768;
    } else {
      mmiDot1(tV, wV, p);
      for (int r = 0; r < 3; ++r) sumPos[r] += p[r];
    }
    // Normals: the rotation's rows, x16 and the shift, then the same weighted sum.
    alignas(16) s32 nrows[4];
    mmiDot3(nV[0], nV[1], nV[2], f.rV[j][0], f.rV[j][1], f.rV[j][2], nrows);
    alignas(16) s16 mV[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    s32 term[3];
    for (int r = 0; r < 3; ++r) {
      term[r] = (s32)((u32)nrows[r] << 4) >> shR;
      mV[2 * r] = (s16)term[r];
    }
    if (wk == 32768) {
      for (int r = 0; r < 3; ++r) sumNrm[r] += term[r] * 32768;
    } else {
      mmiDot1(mV, wV, p);
      for (int r = 0; r < 3; ++r) sumNrm[r] += p[r];
    }
  }
  return narrowTail(f, sumPos, sumNrm, q, n, joints, weights, outPos, outNrm);
}
#endif

// The MMI kernel's corner: the blend of cornerNarrowFast in the lane model. Per joint, the
// three coordinates are one PMADDH each (q_c broadcast to lanes 0, 2, 4 against the
// folded column c), the rows come out of PMFHL.LW as words, the pre-scale is word shifts
// and adds, and the weighted sum over the joints is PMADDH again over the words (their
// low halves are the even lanes). The normal is the same with the rotation and PSLLW 4 /
// PSRAVW before the weight. The tail (the position box clamp, the renormalisation) is the
// scalar narrowTail, shared with cornerNarrowFast, so the bytes are equal by construction
// of the tail; the blend is what is checked against the scalar terms.
u32 cornerMmi(const Fold& f, const s16* q, const s8* n, const u8* joints, const u8* weights,
              const CornerInfl& ci, s16* outPos, s8* outNrm) {
  if (!(f.noSat && f.narrow && f.narrowN))
    return cornerFixed(f, q, n, joints, weights, outPos, outNrm, nullptr, &ci);
#if defined(_EE)
  return cornerMmiEE(f, q, n, joints, weights, ci, outPos, outNrm);
#else
  using namespace Lanes;
  const int shA = 7 - f.expA;   // >= 0 (noSat)
  const int shT = 8 - f.expT;   // > 0 (noSat)
  const int shR = 15 - f.expR;  // >= 12 (expR <= 3)
  const W4 roundT = splatW(1 << (shT - 1));
  W4 sumPos = zeroW();
  W4 sumNrm = zeroW();
  for (int k = 0; k < ci.n; ++k) {
    const u32 j = ci.j[k];
    if (j >= f.slots) continue;
    const s32 wk = ci.w[k];
    // Positions: the three rows as words, then the pre-scale.
    Acc accP = {{0, 0, 0, 0}, {0, 0, 0, 0}};
    for (int c = 0; c < 3; ++c)
      pmaddh(accP, evenLanes(q[c], q[c], q[c]),
             evenLanes(f.a[j][c * 3 + 0], f.a[j][c * 3 + 1], f.a[j][c * 3 + 2]));
    W4 rows = pmfhlLW(accP);
    W4 th = W4{{f.b[j][0], f.b[j][1], f.b[j][2], 0}};
    th = paddW(sraW(rows, shA), th);
    th = sraW(paddW(th, roundT), shT);
    if (wk == 32768) {
      sumPos = paddW(sumPos, sllW(th, 15));  // a single term's weight is exactly 32768
    } else {
      Acc accW = {{0, 0, 0, 0}, {0, 0, 0, 0}};
      pmaddh(accW, asHalves(th), evenLanes(wk, wk, wk));
      sumPos = paddW(sumPos, pmfhlLW(accW));
    }
    // Normals: the rotation's rows (x16, the shift), then the same weighted sum.
    Acc accN = {{0, 0, 0, 0}, {0, 0, 0, 0}};
    for (int c = 0; c < 3; ++c)
      pmaddh(accN, evenLanes(n[c], n[c], n[c]),
             evenLanes(f.r[j][c * 3 + 0], f.r[j][c * 3 + 1], f.r[j][c * 3 + 2]));
    W4 term = sraW(sllW(pmfhlLW(accN), 4), shR);
    if (wk == 32768) {
      sumNrm = paddW(sumNrm, sllW(term, 15));
    } else {
      Acc accM = {{0, 0, 0, 0}, {0, 0, 0, 0}};
      pmaddh(accM, asHalves(term), evenLanes(wk, wk, wk));
      sumNrm = paddW(sumNrm, pmfhlLW(accM));
    }
  }
  const s32 sp[3] = {sumPos.w[0], sumPos.w[1], sumPos.w[2]};
  const s32 sn[3] = {sumNrm.w[0], sumNrm.w[1], sumNrm.w[2]};
  return narrowTail(f, sp, sn, q, n, joints, weights, outPos, outNrm);
#endif
}

u32 cornerFixed(const Fold& f, const s16* q, const s8* n, const u8* joints, const u8* weights,
                s16* outPos, s8* outNrm, u32* blendSaturations, const CornerInfl* infl) {
  if (f.narrow) {
    const CornerInfl local = infl ? *infl : prepCorner(joints, weights);
#if defined(_EE)
    if (gUseMmi && f.noSat && f.narrowN)
      return cornerMmi(f, q, n, joints, weights, local, outPos, outNrm);
#endif
    if (f.noSat) {
      if (local.n == 1) return cornerNarrowFast<1>(f, q, n, joints, weights, local, outPos, outNrm);
      if (local.n == 2) return cornerNarrowFast<2>(f, q, n, joints, weights, local, outPos, outNrm);
      return cornerNarrowFast<0>(f, q, n, joints, weights, local, outPos, outNrm);
    }
    return cornerNarrow(f, q, n, joints, weights, local, outPos, outNrm, blendSaturations);
  }
  return cornerWide(f, q, n, joints, weights, outPos, outNrm, blendSaturations);
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
