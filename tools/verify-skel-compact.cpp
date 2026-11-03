// Host verifier for compact skinning (docs/animated-models.md, "Skinning into
// compact streams"; docs/compact-vertex-format.md). The skin core
// (renderer/3d/mesh/dynamic/skel_skin.hpp) folds the output box into the
// palette once per pose and skins each corner in grid units. The reference
// here is the plain definition: decode the bind data in its box, skin in
// double precision with the palette, encode in the output box with rounding
// half away from zero.
// Checks: positions and normals within one LSB of the reference over 10 000
// random poses of a synthetic 4-bone part; a corner pushed twice past the
// output box clamps to +-32767, is counted, and never wraps; the output box of
// a model is the smallest power-of-two step whose int16 range holds 1.25 x R
// (R = the largest distance from the origin to a corner of the pose or bind
// box). For the combatent's pose box (its .tskl header) that step is 2^-13 m =
// 1.22e-4 m: power-of-two steps are the format rule.
// Build and run from the TyraX root:
//   g++ -std=c++17 -O1 -Wall -Wextra -DTYRA_HARDWARE_TRACE=0 -Ibuild/host-shim
//       -Ivendor/tyra/engine/inc -o build/verify-skel-compact tools/verify-skel-compact.cpp
//       vendor/tyra/engine/src/renderer/3d/mesh/dynamic/skel_skin.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//   build/verify-skel-compact
//       (prints "verify-skel-compact: <n> checks, 0 failed")

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

namespace {

using namespace Tyra;

int checks = 0;
int failures = 0;

void check(bool ok, const char* what) {
  ++checks;
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
  }
}

double roundAway(double v) {
  return v < 0.0 ? -std::floor(-v + 0.5) : std::floor(v + 0.5);
}

// Column-major 4x4 applied to (p, 1): out = M * (p, 1).
void affine(const float* m, const double p[3], double out[3]) {
  for (int r = 0; r < 3; ++r)
    out[r] = (double)m[0 * 4 + r] * p[0] + (double)m[1 * 4 + r] * p[1] +
             (double)m[2 * 4 + r] * p[2] + (double)m[3 * 4 + r];
}

// The reference for one corner, in double precision.
void referenceCorner(const float* palette, const Compact::Box& bind,
                     const Compact::Box& out, const s16* q, const s8* n,
                     const u8* joints, const u8* weights, s16* oq, s8* on,
                     bool* clamped) {
  double sum = 0.0;
  for (int k = 0; k < 4; ++k) sum += weights[k];
  double p[3];
  for (int i = 0; i < 3; ++i) p[i] = (double)bind.origin[i] + (double)bind.step * q[i];
  double pp[3] = {0.0, 0.0, 0.0};
  double nn[3] = {0.0, 0.0, 0.0};
  const double nv[3] = {n[0] / 127.0, n[1] / 127.0, n[2] / 127.0};
  for (int k = 0; k < 4; ++k) {
    const double wk = sum > 0.0 ? weights[k] / sum : 0.0;
    const float* m = palette + 16 * joints[k];
    double a[3];
    affine(m, p, a);
    for (int i = 0; i < 3; ++i) pp[i] += wk * a[i];
    for (int i = 0; i < 3; ++i)
      nn[i] += wk * ((double)m[0 * 4 + i] * nv[0] + (double)m[1 * 4 + i] * nv[1] +
                     (double)m[2 * 4 + i] * nv[2]);
  }
  *clamped = false;
  for (int i = 0; i < 3; ++i) {
    const double t = roundAway(pp[i] / out.step) - roundAway(out.origin[i] / out.step);
    if (t > 32767.0 || t < -32767.0) *clamped = true;
    oq[i] = (s16)(t > 32767.0 ? 32767.0 : t < -32767.0 ? -32767.0 : t);
  }
  const double len = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
  for (int i = 0; i < 3; ++i) {
    const double v = len > 0.0 ? roundAway(nn[i] / len * 127.0) : 0.0;
    on[i] = (s8)(v > 127.0 ? 127.0 : v < -127.0 ? -127.0 : v);
  }
  oq[3] = 0;
  on[3] = 0;
}

// A column-major palette slot of rigid rotation, uniform scale near 1 and a
// translation of up to maxShift metres per axis.
void randomSlot(std::mt19937& rng, float* m, float maxShift) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  double qv[4] = {gauss(rng), gauss(rng), gauss(rng), gauss(rng)};
  const double len =
      std::sqrt(qv[0] * qv[0] + qv[1] * qv[1] + qv[2] * qv[2] + qv[3] * qv[3]);
  for (double& c : qv) c /= len;
  const double x = qv[0], y = qv[1], z = qv[2], w = qv[3];
  const double r[3][3] = {
      {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
      {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
      {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  const double s = 1.0 + 0.1 * uni(rng);
  for (int c = 0; c < 3; ++c)
    for (int row = 0; row < 3; ++row) m[c * 4 + row] = (float)(s * r[row][c]);
  for (int row = 0; row < 3; ++row) m[12 + row] = (float)(maxShift * uni(rng));
  m[3] = m[7] = m[11] = 0.0F;
  m[15] = 1.0F;
}

// A model with the given pose box and no parts: its output box follows from the
// pose box alone.
SkelModel poseBoxModel(const float mn[3], const float mx[3]) {
  SkelModel m;
  for (int i = 0; i < 3; ++i) {
    m.min[i] = mn[i];
    m.max[i] = mx[i];
  }
  return m;
}

}  // namespace

int main() {
  // The combatent's pose union (its .tskl header, version 3, origin at 0).
  const float combatentMin[3] = {-0.5681297F, -0.0397436F, -0.3430586F};
  const float combatentMax[3] = {0.5480978F, 1.7200003F, 0.7414294F};
  {
    const SkelModel model = poseBoxModel(combatentMin, combatentMax);
    const Compact::Box box = skinOutputBox(model);
    const double r = std::sqrt(0.5480978 * 0.5480978 + 1.7200003 * 1.7200003 +
                               0.7414294 * 0.7414294);
    const double half = kSkinBoxFactor * r;
    check(box.origin[0] == 0.0F && box.origin[1] == 0.0F && box.origin[2] == 0.0F,
          "combatent output box is centred on the model origin");
    check(box.step == std::ldexp(1.0F, -13), "combatent output step is 2^-13 m");
    check(box.step <= 1.25e-4F, "combatent output step <= 1.25e-4 m");
    check(half <= 32767.0 * box.step, "combatent output box holds 1.25 R");
    check(half > 16383.0 * box.step, "combatent output step is the smallest power of two");
  }

  // The synthetic part: 100 triangles (300 corners) skinned by 4 palette slots,
  // bind data quantized in a box with a non-zero origin.
  const Compact::Box bind{{0.5F, 0.25F, -0.5F}, std::ldexp(1.0F, -14)};
  const float synthMin[3] = {-1.0F, -1.0F, -1.0F};
  const float synthMax[3] = {1.0F, 1.0F, 1.0F};
  const SkelModel synth = poseBoxModel(synthMin, synthMax);
  const Compact::Box out = skinOutputBox(synth);
  check(out.step == std::ldexp(1.0F, -13), "synthetic output step is a power of two");

  constexpr u32 kCorners = 300;
  constexpr u32 kSlots = 4;
  constexpr int kPoses = 10000;
  std::mt19937 rng(0x5EED52u);
  std::uniform_int_distribution<int> qd(-15000, 15000);
  std::uniform_int_distribution<int> nd(-127, 127);
  std::uniform_int_distribution<int> cutd(0, 255);
  std::vector<s16> q(kCorners * 4, 0);
  std::vector<s8> n(kCorners * 4, 0);
  std::vector<u8> joints(kCorners * 4, 0);
  std::vector<u8> weights(kCorners * 4, 0);
  for (u32 c = 0; c < kCorners; ++c) {
    for (int i = 0; i < 3; ++i) q[c * 4 + i] = (s16)qd(rng);
    for (int i = 0; i < 3; ++i) n[c * 4 + i] = (s8)nd(rng);
    // Four weights that sum to 255: three cut points in [0, 255].
    int cuts[3] = {cutd(rng), cutd(rng), cutd(rng)};
    if (cuts[0] > cuts[1]) std::swap(cuts[0], cuts[1]);
    if (cuts[1] > cuts[2]) std::swap(cuts[1], cuts[2]);
    if (cuts[0] > cuts[1]) std::swap(cuts[0], cuts[1]);
    const int wsum[4] = {cuts[0], cuts[1] - cuts[0], cuts[2] - cuts[1], 255 - cuts[2]};
    for (u32 k = 0; k < 4; ++k) {
      joints[c * 4 + k] = (u8)k;
      weights[c * 4 + k] = (u8)wsum[k];
    }
  }

  std::vector<float> palette(kSlots * 16);
  u32 maxPosDiff = 0, maxNrmDiff = 0, clampMismatch = 0;
  SkelSkin::Folded folded;
  for (int pose = 0; pose < kPoses; ++pose) {
    for (u32 j = 0; j < kSlots; ++j) randomSlot(rng, &palette[j * 16], 0.3F);
    SkelSkin::foldPalette(palette.data(), kSlots, bind, out, folded);
    for (u32 c = 0; c < kCorners; ++c) {
      s16 refQ[4], gotQ[4];
      s8 refN[4], gotN[4];
      bool refClamped = false;
      referenceCorner(palette.data(), bind, out, &q[c * 4], &n[c * 4], &joints[c * 4],
                      &weights[c * 4], refQ, refN, &refClamped);
      const u32 clampedNow = SkelSkin::skinCorner(folded, &q[c * 4], &n[c * 4],
                                                  &joints[c * 4], &weights[c * 4],
                                                  gotQ, gotN);
      for (int i = 0; i < 3; ++i) {
        const u32 d = (u32)std::abs((int)gotQ[i] - (int)refQ[i]);
        if (d > maxPosDiff) maxPosDiff = d;
        const u32 dn = (u32)std::abs((int)gotN[i] - (int)refN[i]);
        if (dn > maxNrmDiff) maxNrmDiff = dn;
      }
      if ((clampedNow != 0) != refClamped) ++clampMismatch;
    }
  }
  check(maxPosDiff <= 1, "positions within one LSB of the reference over 10 000 poses");
  check(maxNrmDiff <= 1, "normals within one int8 step of the reference over 10 000 poses");
  check(clampMismatch == 0, "clamped corners counted as the reference counts them");

  // A corner pushed twice past the output box: +-32767, counted, never wrapped.
  {
    const float halfBox = 32767.0F * out.step;
    const s16 cq[4] = {0, 0, 0, 0};
    const s8 cn[4] = {127, 0, 0, 0};
    const u8 j0[4] = {0, 0, 0, 0};
    const u8 w0[4] = {255, 0, 0, 0};
    for (int sign = -1; sign <= 1; sign += 2) {
      const float pal[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
                             (float)(sign * 2.0 * halfBox), 0, 0, 1};
      SkelSkin::Folded f;
      SkelSkin::foldPalette(pal, 1, bind, out, f);
      s16 gq[4];
      s8 gn[4];
      const u32 clamped = SkelSkin::skinCorner(f, cq, cn, j0, w0, gq, gn);
      check(clamped == 1, "a corner twice past the box is counted once");
      check(gq[0] == (sign > 0 ? 32767 : -32767),
            "a corner twice past the box clamps to +-32767 (no wrap)");
      check(gq[1] == 0 && gq[2] == 0, "the other axes of a clamped corner are unchanged");
    }
  }

  std::printf("verify-skel-compact: %d checks, %d failed (positions max %u LSB, normals max %u)\n",
              checks, failures, maxPosDiff, maxNrmDiff);
  return failures == 0 ? 0 : 1;
}
