// The unique-vertex skin (skel_unique.hpp): plain C++, the host model of the EE kernel.

#include "renderer/3d/mesh/dynamic/skel_unique.hpp"

#include <cmath>
#include <cstring>

namespace Tyra {
namespace SkelSkin {
namespace Unique {

namespace {

// Rounds half away from zero (the same as skel_skin.cpp's roundAway): floor(|v| + 0.5)
// with the sign kept, through a truncating integer conversion, saturating at 2^30.
float roundAway(float v) {
  const float kLimit = 1073741824.0F;  // 2^30
  if (v < 0.0F) {
    const float a = -v + 0.5F;
    return a < kLimit ? -(float)(s32)a : -kLimit;
  }
  const float a = v + 0.5F;
  return a < kLimit ? (float)(s32)a : kLimit;
}

}  // namespace

void buildUnique(const s16* q, const s8* nrm, const u8* joints, const u8* weights,
                 const u32* skinSource, u32 corners, const Compact::Box& bind, UniqueBind& out) {
  // The bind box's grid origin, as Fixed::fold decodes it: position = step x (q + index).
  float oi[3];
  for (int i = 0; i < 3; ++i) oi[i] = roundAway(bind.origin[i] / bind.step);
  u32 count = 0;
  for (u32 v = 0; v < corners; ++v)
    if (skinSource[v] == v) ++count;
  out.count = count;
  out.corner.assign(count, 0);
  out.pos.assign((size_t)count * 4, 0.0F);
  out.nrm.assign((size_t)count * 4, 0.0F);
  out.wgt.assign((size_t)count * 4, 0.0F);
  out.joints.assign((size_t)count * 4, 0);
  u32 u = 0;
  for (u32 v = 0; v < corners; ++v) {
    if (skinSource[v] != v) continue;
    out.corner[u] = v;
    float* p = &out.pos[(size_t)u * 4];
    for (int i = 0; i < 3; ++i) p[i] = bind.step * ((float)q[3 * v + i] + oi[i]);
    p[3] = 1.0F;
    float* n = &out.nrm[(size_t)u * 4];
    for (int i = 0; i < 3; ++i) n[i] = (float)nrm[4 * v + i] * (1.0F / 127.0F);
    n[3] = 0.0F;
    u32 wsum = 0;
    for (int k = 0; k < 4; ++k) wsum += weights[4 * v + k];
    const float inv = wsum > 0 ? 1.0F / (float)wsum : 0.0F;
    for (int k = 0; k < 4; ++k) {
      out.wgt[(size_t)u * 4 + k] = (float)weights[4 * v + k] * inv;
      out.joints[(size_t)u * 4 + k] = joints[4 * v + k];
    }
    ++u;
  }
}

void blend(const UniqueBind& ub, const float* palette, float* outPos, float* outNrm) {
  for (u32 u = 0; u < ub.count; ++u) {
    const float* p = &ub.pos[(size_t)u * 4];
    const float* n = &ub.nrm[(size_t)u * 4];
    float px[4] = {0.0F, 0.0F, 0.0F, 0.0F};
    float nn[4] = {0.0F, 0.0F, 0.0F, 0.0F};
    for (int k = 0; k < 4; ++k) {
      const float w = ub.wgt[(size_t)u * 4 + k];
      if (w == 0.0F) continue;
      const float* m = palette + 16 * (u32)ub.joints[(size_t)u * 4 + k];
      // The matrix-vector product in the VU0 order: column 0 x x, then + column 1 x y,
      // column 2 x z, the translation column (w = 1).
      for (int r = 0; r < 3; ++r) {
        const float a = m[0 + r] * p[0];
        const float b = a + m[4 + r] * p[1];
        const float c = b + m[8 + r] * p[2];
        const float d = c + m[12 + r] * p[3];
        px[r] += w * d;
      }
      for (int r = 0; r < 3; ++r) {
        const float a = m[0 + r] * n[0];
        const float b = a + m[4 + r] * n[1];
        nn[r] += w * (b + m[8 + r] * n[2]);
      }
    }
    float* op = &outPos[(size_t)u * 4];
    float* on = &outNrm[(size_t)u * 4];
    op[0] = px[0];
    op[1] = px[1];
    op[2] = px[2];
    op[3] = 1.0F;
    // Unit length, as the VU0 tail writes it (1 / sqrt(len^2 + eps), then the product): the
    // quantize takes the normal as it is.
    const float len2 = nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2];
    const float inv = 1.0F / std::sqrt(len2 + 1e-12F);
    on[0] = nn[0] * inv;
    on[1] = nn[1] * inv;
    on[2] = nn[2] * inv;
    on[3] = 0.0F;
  }
}

void poseOrigin(const float* pos, u32 count, float step, s32 oi[3]) {
  oi[0] = oi[1] = oi[2] = 0;
  if (count == 0 || !(step > 0.0F)) return;
  float mn[3] = {pos[0], pos[1], pos[2]}, mx[3] = {pos[0], pos[1], pos[2]};
  for (u32 u = 1; u < count; ++u)
    for (int i = 0; i < 3; ++i) {
      const float v = pos[(size_t)u * 4 + i];
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
    }
  const float invStep = 1.0F / step;
  for (int i = 0; i < 3; ++i)
    oi[i] = (s32)roundAway(0.5F * (mn[i] + mx[i]) * invStep);
}

u32 quantizeOne(const float* p, const float* a, float invStep, const s32 oi[3], s16* qPos,
                s8* qNrm) {
  // One clamp in float, then round half away from zero by the sign of the value and a
  // truncating conversion: no saturation branches in the loop. A value within half a step of
  // the limit may round to the limit without counting as clamped (the counter is the
  // exact-limit one; the error is one step at the boundary).
  u32 clip = 0;
  for (int i = 0; i < 3; ++i) {
    float x = p[i] * invStep - (float)oi[i];
    if (x > 32767.0F) {
      x = 32767.0F;
      clip = 1;
    } else if (x < -32767.0F) {
      x = -32767.0F;
      clip = 1;
    }
    qPos[i] = (s16)(s32)(x + (x < 0.0F ? -0.5F : 0.5F));
  }
  qPos[3] = 0;
  // The normal is unit length already (blend): x127, rounded, clamped.
  for (int i = 0; i < 3; ++i) {
    float y = a[i] * 127.0F;
    if (y > 127.0F) y = 127.0F;
    else if (y < -127.0F) y = -127.0F;
    qNrm[i] = (s8)(s32)(y + (y < 0.0F ? -0.5F : 0.5F));
  }
  qNrm[3] = 0;
  return clip;
}

u32 quantize(const float* pos, const float* nrm, u32 count, float step, const s32 oi[3],
             s16* qPos, s8* qNrm, u8* clamped) {
  const float invStep = 1.0F / step;
  u32 n = 0;
  for (u32 u = 0; u < count; ++u) {
    const u32 clip = quantizeOne(&pos[(size_t)u * 4], &nrm[(size_t)u * 4], invStep, oi,
                                 &qPos[(size_t)u * 4], &qNrm[(size_t)u * 4]);
    clamped[u] = (u8)clip;
    n += clip;
  }
  return n;
}

u32 scatterFlat(const SkinScatter& map, u32 size, const UniqueBind& ub, const s16* qPos,
                const s8* qNrm, const u8* clamped, u32 corners, s16* outPos, s8* outNrm) {
  u32 clampedDst = 0;
  for (u32 u = 0; u < ub.count; ++u) {
    const u32 src = ub.corner[u];
    const u32 d0 = map.dstOff[src], d1 = map.dstOff[src + 1];
    if (clamped[u]) clampedDst += d1 - d0;
    for (u32 d = d0; d < d1; ++d) {
      const u32 dv = map.dst[d];
      const u32 v = (dv >> 16) * size + (dv & 0xFFFFu);
      if (v >= corners) continue;
      std::memcpy(&outPos[(size_t)v * 4], &qPos[(size_t)u * 4], 8);
      std::memcpy(&outNrm[(size_t)v * 4], &qNrm[(size_t)u * 4], 4);
    }
  }
  return clampedDst;
}

}  // namespace Unique
}  // namespace SkelSkin
}  // namespace Tyra
