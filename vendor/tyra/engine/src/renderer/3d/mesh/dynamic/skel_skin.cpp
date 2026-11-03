// The scalar skin core (renderer/3d/mesh/dynamic/skel_skin.hpp).

#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

#include <cmath>

namespace Tyra {

namespace {

// Rounds half away from zero, as Compact::quantize does.
float roundAway(float v) {
  return v < 0.0F ? -std::floor(-v + 0.5F) : std::floor(v + 0.5F);
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

  // The smallest power of two step with kSkinBoxFactor x R <= 32767 steps.
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

void foldPalette(const float* palette, u32 slots, const Compact::Box& bind,
                 const Compact::Box& out, Folded& folded) {
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
  for (u32 k = 0; k < slots && k < kMaxSlots; ++k) {
    const float* m = palette + 16 * k;
    float qm[16];
    mul4(qout, m, qm);
    mul4(qm, dbind, folded.f[k]);
    for (int c = 0; c < 3; ++c)
      for (int row = 0; row < 3; ++row) folded.r[k][c * 3 + row] = m[c * 4 + row];
  }
}

u32 skinCorner(const Folded& folded, const s16* pos, const s8* nrm,
               const u8* joints, const u8* weights, s16* outPos, s8* outNrm) {
  float sum = 0.0F;
  for (int k = 0; k < 4; ++k) sum += (float)weights[k];
  const float q[4] = {(float)pos[0], (float)pos[1], (float)pos[2], 1.0F};
  const float nv[3] = {(float)nrm[0] / 127.0F, (float)nrm[1] / 127.0F,
                       (float)nrm[2] / 127.0F};
  float px[3] = {0.0F, 0.0F, 0.0F};
  float nn[3] = {0.0F, 0.0F, 0.0F};
  for (int k = 0; k < 4; ++k) {
    const float wk = sum > 0.0F ? (float)weights[k] / sum : 0.0F;
    const u32 j = joints[k];
    const float* f = folded.f[j];
    const float* r = folded.r[j];
    for (int i = 0; i < 3; ++i)
      px[i] += wk * (f[0 * 4 + i] * q[0] + f[1 * 4 + i] * q[1] + f[2 * 4 + i] * q[2] +
                     f[3 * 4 + i] * q[3]);
    for (int i = 0; i < 3; ++i)
      nn[i] += wk * (r[0 * 3 + i] * nv[0] + r[1 * 3 + i] * nv[1] + r[2 * 3 + i] * nv[2]);
  }
  u32 clamped = 0;
  for (int i = 0; i < 3; ++i) {
    float v = roundAway(px[i]);
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
  for (int i = 0; i < 3; ++i) {
    float v = len > 0.0F ? roundAway(nn[i] / len * 127.0F) : 0.0F;
    if (v > 127.0F) v = 127.0F;
    if (v < -127.0F) v = -127.0F;
    outNrm[i] = (s8)v;
  }
  outNrm[3] = 0;
  return clamped;
}

}  // namespace SkelSkin
}  // namespace Tyra
