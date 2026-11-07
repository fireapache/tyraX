#pragma once

// Skinning straight to the GS, the EE half (docs/animated-models.md, "Skinning straight to the
// GS (EE only)"): the integer transform, projection and lighting of a skinned part's UNIQUE
// vertices. Plain C++ (no PS2 headers), so the host verifier (tools/verify-skel-eegs.cpp) runs
// the same code the EE does.
//
// A pose pass leaves, per part level, its unique vertices blended in int16 (Result): a position
// as a signed index in the pose's box (model position = step x (q + oi)) and an int8 x 127
// normal. A draw then maps each unique to the GS in fixed point:
//
//   acc_r  = A_r . q                r = x, y, z, w: int16 x int16 products in int32
//   clip_r = float(acc_r) x k_r + T_r   (k_r a power of two, T_r the translation in float)
//   inv    = 1 / clip_w             one divide per unique
//   X, Y   = 12.4 screen position   clip_x x inv x 16 s + 16 s, truncated as the VU1 does
//   Z      = the depth range's own scale, 24 bits (RendererCoreDepth::scale)
//   Q      = 1 / w  (S, T of a corner are s x Q, t x Q: perspective-correct, as the VU1 does)
//   RGB    = ambient + sum_i colour_i x clamp(D_i . (L n) / 127)  in Q10 / Q16 integers
//
// The matrix A is the model-view-projection x the box decode (step), quantised per row to 15
// bits with a power-of-two scale chosen so the three products of the part's own index extent fit
// int32. The translation (the box origin and the model-view-projection's own) is added in float
// after the integer sum: folding it into the integer sum would cost the matrix most of its bits
// for a part far from the camera, where the depth range needs them most.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

namespace Tyra {
namespace SkelSkin {
namespace EeGs {

/** The view's GS mapping: ndc x s + s per axis (RendererCoreDepth), the depth range and fog. */
struct Screen {
  float sx = 2048.0F, sy = 2048.0F, sz = 8388607.5F;
  float maxZ = 16777215.0F;
  bool fog = false;  // PRIM.FGE: F per vertex
  float fogScale = 0.0F, fogOffset = 255.0F;
};

/** One part's unique vertices after a pose (owned by the instance's part level). */
struct Result {
  u32 count = 0;          // unique vertices
  u32 capacity = 0;       // what pos and nrm hold
  s16* pos = nullptr;     // [count][4] index in the pose box (fourth lane 0)
  s8* nrm = nullptr;      // [count][4] normal x 127 (fourth lane 0)
  s32 oi[3] = {0, 0, 0};  // the pose box origin, in steps
  s16 lo[3] = {0, 0, 0};  // the least / greatest index per axis over the uniques
  s16 hi[3] = {0, 0, 0};
  u32 clamped = 0;        // uniques with a position axis at the box limit
  float step = 0.0F;      // the box step (a power of two)
  u32 version = 0;        // moves with every pose written (a stream compares it)
};

/** A part level's bind-time draw data, shared by every instance of the model (built on first
 * draw): the unique each corner takes its position and light from, and each corner's own
 * texture coordinate (the positions are shared by corners, the coordinates are not). */
struct Bind {
  bool ready = false;
  u32 count = 0;                 // corners
  std::vector<u16> cornerUnique;  // [count]
  std::vector<float> st;          // [count][2] s, t
};

/** One part in one view: bind-to-clip as integers. */
struct Xform {
  s16 a[4][3];  // rows x, y, z, w
  float k[4];   // clip_r = float(acc_r) * k_r + t_r
  float t[4];
  float cx, cy, cz;  // X16 = clip_x * inv * cx + ox
  float ox, oy, oz;
  float fogScale;    // F = clip_w * fogScale + fogOffset
  float fogOffset;
  bool fog;
  float maxZ;
};

enum class Box : u8 { Inside, Crossing, Outside };

inline float pow2f(int e) {
  const u32 bits = (u32)(e + 127) << 23;
  float f;
  std::memcpy(&f, &bits, 4);
  return f;
}

inline int floorLog2(float x) {  // x > 0, normal
  u32 bits;
  std::memcpy(&bits, &x, 4);
  return (int)((bits >> 23) & 0xFF) - 127;
}

/** The part's index box [lo, hi] against the view: Inside when all eight corners are in front
 * of the near plane and inside the guard band (|x|, |y| <= band x w) and the far plane;
 * Outside when all eight are outside one plane (the part is culled); else Crossing. `mvp` is the
 * column-major model-view-projection (clip = sum_c mvp[4c + r] p_c + mvp[12 + r]). */
inline Box classifyBox(const float* mvp, float step, const s32 oi[3], const s16 lo[3],
                       const s16 hi[3], float band) {
  u32 outside[6] = {0, 0, 0, 0, 0, 0};
  u32 all = 0;
  for (int c = 0; c < 8; ++c) {
    float p[3];
    for (int i = 0; i < 3; ++i) p[i] = step * (float)(((c >> i) & 1 ? hi[i] : lo[i]) + oi[i]);
    float clip[4];
    for (int r = 0; r < 4; ++r)
      clip[r] = mvp[r] * p[0] + mvp[4 + r] * p[1] + mvp[8 + r] * p[2] + mvp[12 + r];
    const float w = clip[3];
    if (clip[0] > band * w) ++outside[0];
    if (-clip[0] > band * w) ++outside[1];
    if (clip[1] > band * w) ++outside[2];
    if (-clip[1] > band * w) ++outside[3];
    if (clip[2] > w) ++outside[4];
    if (clip[2] < -w || w <= 0.0F) ++outside[5];
    (void)all;
  }
  u32 any = 0;
  for (int i = 0; i < 6; ++i) {
    if (outside[i] == 8) return Box::Outside;
    any |= outside[i];
  }
  return any ? Box::Crossing : Box::Inside;
}

/** Builds the integer transform of a part for one view. `mvp` column-major as above. Returns
 * false when the part cannot be mapped (a degenerate matrix). */
inline bool buildXform(const float* mvp, float step, const s32 oi[3], const s16 lo[3],
                       const s16 hi[3], const Screen& scr, Xform& x) {
  float qext[3];
  for (int c = 0; c < 3; ++c) {
    const float l = std::fabs((float)lo[c]), h = std::fabs((float)hi[c]);
    qext[c] = l > h ? l : h;
  }
  int e[4];
  for (int r = 0; r < 4; ++r) {
    float A[3];
    float amax = 0.0F, bound = 0.0F;
    for (int c = 0; c < 3; ++c) {
      A[c] = mvp[4 * c + r] * step;
      const float aa = std::fabs(A[c]);
      if (aa > amax) amax = aa;
      bound += aa * qext[c];
    }
    float T = mvp[12 + r];
    for (int c = 0; c < 3; ++c) T += mvp[4 * c + r] * step * (float)oi[c];
    if (!(bound > 0.0F) || !(amax > 0.0F)) return false;
    // Matrix entries to 15 bits, and the three products to 30 (int32 with room): the index
    // extent of a part is a few thousand steps, so 15 bits of matrix fit.
    float s = 32700.0F / amax;
    const float s2 = 1073741824.0F / bound;
    if (s2 < s) s = s2;
    e[r] = floorLog2(s);
    const float sc = pow2f(e[r]);
    x.k[r] = pow2f(-e[r]);
    x.t[r] = T;
    for (int c = 0; c < 3; ++c) {
      const float v = A[c] * sc;
      x.a[r][c] = (s16)(v < 0.0F ? v - 0.5F : v + 0.5F);
    }
  }
  x.cx = 16.0F * scr.sx;
  x.cy = 16.0F * scr.sy;
  x.cz = scr.sz;
  x.ox = 16.0F * scr.sx;
  x.oy = 16.0F * scr.sy;
  x.oz = scr.sz;
  x.fog = scr.fog;
  x.fogScale = scr.fogScale;
  x.fogOffset = scr.fogOffset;
  x.maxZ = scr.maxZ;
  return true;
}

/** One unique vertex on the GS: the three words the per-corner packet carries (the corner adds
 * its own S x Q, T x Q). */
struct Out {
  u32 rgba;
  float q;
  u32 xy;    // y << 16 | x, 12.4
  u32 zf;    // z (24 bits) | f << 24
};

/** The transform of one unique's index `q` (three int16). Returns false when its clip w is not
 * positive (the part was classified Inside, so this does not happen unless the matrix lost
 * precision at the near plane); out is then not written. */
inline bool project(const Xform& x, const s16* q, Out& out) {
  const s32 q0 = q[0], q1 = q[1], q2 = q[2];
  const s32 ax = x.a[0][0] * q0 + x.a[0][1] * q1 + x.a[0][2] * q2;
  const s32 ay = x.a[1][0] * q0 + x.a[1][1] * q1 + x.a[1][2] * q2;
  const s32 az = x.a[2][0] * q0 + x.a[2][1] * q1 + x.a[2][2] * q2;
  const s32 aw = x.a[3][0] * q0 + x.a[3][1] * q1 + x.a[3][2] * q2;
  const float cw = (float)aw * x.k[3] + x.t[3];
  if (!(cw > 1e-6F)) return false;
  const float cxv = (float)ax * x.k[0] + x.t[0];
  const float cyv = (float)ay * x.k[1] + x.t[1];
  const float czv = (float)az * x.k[2] + x.t[2];
  const float inv = 1.0F / cw;
  float fx = cxv * (inv * x.cx) + x.ox;
  float fy = cyv * (inv * x.cy) + x.oy;
  float fz = czv * (inv * x.cz) + x.oz;
  // 12.4 positions inside the GS window (0 .. 4095.94 px); depth inside the buffer's range.
  if (fx < 0.0F) fx = 0.0F;
  if (fy < 0.0F) fy = 0.0F;
  if (fx > 65535.0F) fx = 65535.0F;
  if (fy > 65535.0F) fy = 65535.0F;
  if (fz < 0.0F) fz = 0.0F;
  if (fz > x.maxZ) fz = x.maxZ;
  u32 f = 255;
  if (x.fog) {
    float ff = cw * x.fogScale + x.fogOffset;
    if (ff < 0.0F) ff = 0.0F;
    if (ff > 255.0F) ff = 255.0F;
    f = (u32)ff;
  }
  out.xy = ((u32)fy << 16) | (u32)fx;
  out.zf = (u32)fz | (f << 24);
  out.q = inv;
  return true;
}

/** The lighting of a part for one draw: the VU1 program's arithmetic in integers. A normal n
 * (int8 x 127) gives e_i = sum_c g[i][c] n_c (Q16 per unit of n: D_i . (L n) / 127), clamped
 * to [floor, 1] in Q10; the colour is (amb x 65536 + sum_i col[j][i] x e_i) >> 16, clamped to
 * 0..255 (colours in Q6). */
struct Light {
  s32 g[3][3];
  s32 col[3][3];  // col[j][i]: light i's channel j
  s32 amb[3];     // ambient channel j x 65536
  s32 floorQ10;   // the lower clamp of a light factor: 0, or -1.0 for signed SH
};

/** `lm` the light matrix (three columns of four floats, as the VU1 program reads it), `dirs` the
 * three light directions (Vec4 each, a matrix whose row i dotted with the normal gives the
 * factor of light i), `colors` four Vec4: the three light colours then the ambient. */
inline void buildLight(const float* lm, const float* dirs, const float* colors, bool signedSH,
                       Light& l) {
  for (int i = 0; i < 3; ++i)
    for (int c = 0; c < 3; ++c) {
      float g = 0.0F;
      for (int cp = 0; cp < 3; ++cp) g += dirs[4 * cp + i] * lm[4 * c + cp];
      g *= (1.0F / 127.0F) * 65536.0F;
      l.g[i][c] = (s32)(g < 0.0F ? g - 0.5F : g + 0.5F);
    }
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i) {
      const float v = colors[4 * i + j] * 64.0F;
      l.col[j][i] = (s32)(v < 0.0F ? v - 0.5F : v + 0.5F);
    }
    const float a = colors[12 + j] * 65536.0F;
    l.amb[j] = (s32)(a < 0.0F ? a - 0.5F : a + 0.5F);
  }
  l.floorQ10 = signedSH ? -1024 : 0;
}

/** The packed GS colour (r | g << 8 | b << 16 | alpha 128 << 24) of a normal. */
inline u32 light(const Light& l, const s8* n) {
  const s32 n0 = n[0], n1 = n[1], n2 = n[2];
  s32 e[3];
  for (int i = 0; i < 3; ++i) {
    s32 v = (l.g[i][0] * n0 + l.g[i][1] * n1 + l.g[i][2] * n2) >> 6;
    if (v > 1024) v = 1024;
    if (v < l.floorQ10) v = l.floorQ10;
    e[i] = v;
  }
  u32 rgba = 128u << 24;
  for (int j = 0; j < 3; ++j) {
    s32 c = (l.amb[j] + l.col[j][0] * e[0] + l.col[j][1] * e[1] + l.col[j][2] * e[2]) >> 16;
    if (c > 255) c = 255;
    if (c < 0) c = 0;
    rgba |= (u32)c << (8 * j);
  }
  return rgba;
}

}  // namespace EeGs
}  // namespace SkelSkin
}  // namespace Tyra
