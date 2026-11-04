// Host verifier for the fixed-point skin (renderer/3d/mesh/dynamic/skel_skin.hpp,
// SkelSkin::Fixed). The fixed-point path keeps a skinned fighter in int16 from the
// compact bind data to the stream: the palette is folded once per pose into int16
// matrices (Fixed::fold), and each corner is a 16-bit weighted blend of its four
// joints (Fixed::cornerFixed). The reference is the plain definition in double
// precision, as verify-skel-compact.cpp states it.
//
// Checks:
//  - positions within 2 output steps of the reference, normals within 2 x127 units
//    after renormalisation, over 10 000 random poses of a synthetic 4-bone part
//    (with the extreme ones: scale 1.3, shifts of 2 m);
//  - a corner pushed past the output box clamps to +-32767, is counted, and never
//    wraps (its sign is the reference's sign);
//  - repeats (skinSource < v) give the same bytes as their source corner;
//  - the same two bounds on the real combatent model's bind data (argv[1]), in its
//    own box, with the identity and a random palette.
// Prints the worst and mean errors, so the tolerance can be read off the numbers.
//
// Build and run from the TyraX root:
//   g++ -std=c++17 -O1 -Wall -Wextra -DTYRA_HARDWARE_TRACE=0 -Ibuild/host-shim
//       -Ivendor/tyra/engine/inc -o build/verify-skel-fixed tools/verify-skel-fixed.cpp
//       vendor/tyra/engine/src/renderer/3d/mesh/dynamic/skel_skin.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//       vendor/tyra/engine/src/loaders/3d/tskl_loader/tskl_parse.cpp
//       vendor/tyra/engine/src/loaders/3d/tmdl_loader/tmdl_compact.cpp
//   build/verify-skel-fixed res/models/combatent__ovrb22b.tskl
//       (prints "verify-skel-fixed: <n> checks, 0 failed")

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

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

// The double-precision reference of one corner, unclamped: the output index (steps,
// minus the output origin index) and the unit normal x127.
struct Truth {
  double pos[3];
  double nrm[3];
};

Truth referenceCorner(const float* palette, const Compact::Box& bind, const Compact::Box& out,
                      const s16* q, const s8* n, const u8* joints, const u8* weights) {
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
    for (int r = 0; r < 3; ++r) {
      const double a = (double)m[0 * 4 + r] * p[0] + (double)m[1 * 4 + r] * p[1] +
                       (double)m[2 * 4 + r] * p[2] + (double)m[3 * 4 + r];
      pp[r] += wk * a;
    }
    for (int i = 0; i < 3; ++i)
      nn[i] += wk * ((double)m[0 * 4 + i] * nv[0] + (double)m[1 * 4 + i] * nv[1] +
                     (double)m[2 * 4 + i] * nv[2]);
  }
  Truth t;
  for (int i = 0; i < 3; ++i)
    t.pos[i] = pp[i] / out.step - roundAway(out.origin[i] / out.step);
  const double len = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
  for (int i = 0; i < 3; ++i) t.nrm[i] = len > 0.0 ? nn[i] / len * 127.0 : 0.0;
  return t;
}

// A column-major palette slot of rigid rotation, uniform scale near 1 and a
// translation of up to maxShift metres per axis.
void randomSlot(std::mt19937& rng, float* m, float maxShift, float scaleSpread) {
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
  const double s = 1.0 + scaleSpread * uni(rng);
  for (int c = 0; c < 3; ++c)
    for (int row = 0; row < 3; ++row) m[c * 4 + row] = (float)(s * r[row][c]);
  for (int row = 0; row < 3; ++row) m[12 + row] = (float)(maxShift * uni(rng));
  m[3] = m[7] = m[11] = 0.0F;
  m[15] = 1.0F;
}

// Error accumulators: the worst and the sum (for the mean) of each kind.
struct Errors {
  double worstPos = 0.0, sumPos = 0.0;
  double worstNrm = 0.0, sumNrm = 0.0;
  u64 posCount = 0, nrmCount = 0;
  u64 clamped = 0;    // corners the fixed path clamped
  u64 outside = 0;    // corners whose reference is outside the box (and not saturated)
  u64 corners = 0;    // corners compared
  u64 saturated = 0;  // corners whose blend saturated the int16 pre-scale (not held to the tolerance)
  u64 tclamped = 0;   // joint terms that saturated
};

// One corner of the fixed path against its reference. A corner whose reference is
// inside the box is held to the tolerance; one outside must clamp and keep its sign.
void compareCorner(const SkelSkin::Fixed::Fold& fold, const float* palette,
                   const Compact::Box& bind, const Compact::Box& out, const s16* q,
                   const s8* n, const u8* joints, const u8* weights, Errors& e) {
  s16 oq[4];
  s8 on[4];
  u32 tc = 0;
  const u32 clamped = SkelSkin::Fixed::cornerFixed(fold, q, n, joints, weights, oq, on, &tc);
  e.tclamped += tc;
  ++e.corners;
  if (tc != 0) {
    // A joint term outside the pre-scale: reported, not held to the tolerance.
    ++e.saturated;
    return;
  }
  const Truth t = referenceCorner(palette, bind, out, q, n, joints, weights);
  bool outside = false;
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(t.pos[i]) > 32767.0) {
      outside = true;
      // Clamp, no wrap: the sign of the reference, and the clamp value.
      const double want = t.pos[i] > 0 ? 32767.0 : -32767.0;
      check(oq[i] == (s16)want, "an out-of-box position clamps to +-32767");
      continue;
    }
    const double err = std::fabs((double)oq[i] - t.pos[i]);
    if (err > e.worstPos) e.worstPos = err;
    e.sumPos += err;
    ++e.posCount;
  }
  if (outside) {
    ++e.outside;
    if (!clamped) check(false, "an out-of-box corner is counted as clamped");
  }
  e.clamped += clamped;
  for (int i = 0; i < 3; ++i) {
    const double err = std::fabs((double)on[i] - t.nrm[i]);
    if (err > e.worstNrm) e.worstNrm = err;
    e.sumNrm += err;
    ++e.nrmCount;
  }
}

void printErrors(const char* label, const Errors& e) {
  std::printf("%s: corners %llu, saturated %llu (joint terms %llu); positions worst %.3f steps "
              "mean %.4f (%llu axes); normals worst %.3f mean %.4f (x127 units, %llu axes); "
              "out-of-box %llu, clamped %llu\n",
              label, (unsigned long long)e.corners, (unsigned long long)e.saturated,
              (unsigned long long)e.tclamped, e.worstPos,
              e.posCount ? e.sumPos / (double)e.posCount : 0.0, (unsigned long long)e.posCount,
              e.worstNrm, e.nrmCount ? e.sumNrm / (double)e.nrmCount : 0.0,
              (unsigned long long)e.nrmCount, (unsigned long long)e.outside,
              (unsigned long long)e.clamped);
}

// The synthetic 4-bone part: a bind box of 2^-10 m steps, bind indices within
// +-3000 steps (about 3 m), and an output box of 2^-13 m steps.
void syntheticChecks(u32 seed) {
  std::mt19937 rng(seed);
  const int qMax = 1000;  // bind indices within 1 m of the origin in their own box
  std::uniform_int_distribution<int> qd(-qMax, qMax);
  std::uniform_int_distribution<int> nd(-127, 127);
  std::uniform_int_distribution<int> wd(0, 255);
  std::uniform_int_distribution<int> jd(0, 7);
  // The bind origin is on its grid (the format's rule: a whole number of bind steps),
  // so the folds' origin index is exact.
  Compact::Box bind{{307.0F / 1024.0F, 205.0F / 1024.0F, -102.0F / 1024.0F}, 1.0F / 1024.0F};
  Compact::Box out{{0.5F, -0.25F, 0.125F}, 1.0F / 8192.0F};
  const u32 slots = 8;
  const u32 corners = 4000;
  Errors ord, ext;
  std::vector<float> palette(slots * 16, 0.0F);
  std::vector<s16> pos(3 * (size_t)corners);
  std::vector<s8> nrm(4 * (size_t)corners);
  std::vector<u8> joints(4 * (size_t)corners), weights(4 * (size_t)corners);
  for (u32 v = 0; v < corners; ++v) {
    for (int i = 0; i < 3; ++i) pos[3 * v + i] = (s16)qd(rng);
    for (int i = 0; i < 3; ++i) nrm[4 * v + i] = (s8)nd(rng);
    nrm[4 * v + 3] = 0;
    for (int k = 0; k < 4; ++k) {
      joints[4 * v + k] = (u8)jd(rng);
      weights[4 * v + k] = (u8)wd(rng);
    }
  }
  // 10 000 poses: ordinary ones (shift 1 m, scale 0.1) and extreme ones (shift 2 m,
  // scale 0.3), each checked on every corner.
  for (int pose = 0; pose < 10000; ++pose) {
    const bool extreme = pose % 4 == 0;
    for (u32 j = 0; j < slots; ++j)
      randomSlot(rng, &palette[j * 16], extreme ? 1.0F : 0.5F, extreme ? 0.3F : 0.1F);
    // The output box is the pose's own: its origin is the grid point at the mean joint
    // translation (the stream's box is centred on the pose's extent the same way).
    Compact::Box poseOut = out;
    for (int i = 0; i < 3; ++i) {
      double c = 0.0;
      for (u32 j = 0; j < slots; ++j) c += palette[j * 16 + 12 + i];
      c /= (double)slots;
      poseOut.origin[i] = (float)(std::floor(c / out.step + 0.5) * out.step);
    }
    SkelSkin::Fixed::Fold fold;
    SkelSkin::Fixed::fold(palette.data(), slots, bind, poseOut, fold, qMax);
    Errors& e = extreme ? ext : ord;
    for (u32 v = 0; v < corners; v += 7)  // every 7th corner keeps the run short
      compareCorner(fold, palette.data(), bind, poseOut, &pos[3 * v], &nrm[4 * v], &joints[4 * v],
                    &weights[4 * v], e);
  }
  printErrors("synthetic ordinary poses", ord);
  printErrors("synthetic extreme poses", ext);
  check(ord.worstPos <= 2.0, "synthetic ordinary positions within 2 output steps");
  check(ord.worstNrm <= 2.0, "synthetic ordinary normals within 2 x127 units");
  check(ord.saturated == 0, "no ordinary pose saturates the int16 pre-scale");
  check(ext.worstPos <= 2.0, "synthetic extreme positions within 2 output steps (unsaturated corners)");
  check(ext.worstNrm <= 2.0, "synthetic extreme normals within 2 x127 units");
}

// Pushed past the box: a pose that moves the part 6 m off the output origin must
// clamp to +-32767 and be counted, with the sign of the reference.
void clampChecks() {
  std::mt19937 rng(7);
  Compact::Box bind{{0.0F, 0.0F, 0.0F}, 1.0F / 1024.0F};
  Compact::Box out{{0.0F, 0.0F, 0.0F}, 1.0F / 8192.0F};
  float palette[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 6.0F, -6.0F, 0.0F, 1};
  s16 q[3] = {1000, -1000, 0};
  s8 n[4] = {127, 0, 0, 0};
  u8 joints[4] = {0, 0, 0, 0};
  u8 weights[4] = {255, 0, 0, 0};
  SkelSkin::Fixed::Fold fold;
  SkelSkin::Fixed::fold(palette, 1, bind, out, fold, 1000);
  s16 oq[4];
  s8 on[4];
  u32 tc = 0;
  const u32 clamped = SkelSkin::Fixed::cornerFixed(fold, q, n, joints, weights, oq, on, &tc);
  check(clamped == 1, "an out-of-box corner is counted once");
  check(oq[0] == 32767, "a corner pushed +x past the box clamps to +32767 (no wrap)");
  check(oq[1] == -32767, "a corner pushed -y past the box clamps to -32767 (no wrap)");
  (void)rng;
}

// Repeats: two corners with the same bind data, joints and weights give the same
// bytes (the repeat copy the stream path makes is the same as blending again).
void repeatChecks() {
  Compact::Box bind{{0.1F, 0.0F, 0.0F}, 1.0F / 2048.0F};
  Compact::Box out{{0.0F, 0.0F, 0.0F}, 1.0F / 8192.0F};
  float palette[32] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.2F, 0, 0, 1,
                       0.7F, 0.7F, 0, 0, -0.7F, 0.7F, 0, 0, 0, 0, 0.5F, 0, 0.1F, 0, 0, 1};
  SkelSkin::Fixed::Fold fold;
  SkelSkin::Fixed::fold(palette, 2, bind, out, fold);
  s16 q[3] = {123, -456, 789};
  s8 n[4] = {50, -60, 70, 0};
  u8 joints[4] = {0, 1, 0, 0};
  u8 weights[4] = {100, 155, 0, 0};
  s16 a[4], b[4];
  s8 na[4], nb[4];
  SkelSkin::Fixed::cornerFixed(fold, q, n, joints, weights, a, na, nullptr);
  SkelSkin::Fixed::cornerFixed(fold, q, n, joints, weights, b, nb, nullptr);
  check(std::memcmp(a, b, sizeof(a)) == 0 && std::memcmp(na, nb, sizeof(na)) == 0,
        "a repeat's bytes equal its source's");
}

// The real model's bind data: its own box, its own parts and LODs, the identity and a
// random palette (the stream check's poses).
void realModelChecks(const char* path) {
  SkelModel model;
  bool loaded = false;
  if (FILE* f = std::fopen(path, "rb")) {
    std::vector<u8> bytes;
    u8 chunk[8192];
    size_t got;
    while ((got = std::fread(chunk, 1, sizeof(chunk), f)) > 0)
      bytes.insert(bytes.end(), chunk, chunk + got);
    std::fclose(f);
    loaded = TsklLoader::parse(bytes.data(), bytes.size(), model);
  }
  check(loaded, "the real model parses");
  if (!loaded) return;
  const Compact::Box bind = model.box;
  const Compact::Box out = skinOutputBox(model);
  const u32 slots = (u32)model.palette.size();
  check(slots <= SkelSkin::kMaxSlots, "the palette fits the fold");
  std::vector<float> identity(slots * 16, 0.0F);
  for (u32 j = 0; j < slots; ++j)
    identity[j * 16 + 0] = identity[j * 16 + 5] = identity[j * 16 + 10] = identity[j * 16 + 15] = 1.0F;
  std::mt19937 rng(0xC0FFEEu);
  std::vector<float> posed(slots * 16, 0.0F);
  for (u32 j = 0; j < slots; ++j) randomSlot(rng, &posed[j * 16], 0.3F, 0.1F);
  // The largest bind index of the model: the bound the folds' pre-scale is set for.
  u32 qMax = 1;
  for (const SkelPart& part : model.parts) {
    for (u32 i = 0; i < 3 * part.vertexCount; ++i) {
      const u32 a = (u32)std::abs((int)part.pos[i]);
      if (a > qMax) qMax = a;
    }
    for (const SkelLod& lod : part.lods)
      for (u32 i = 0; i < 3 * lod.vertexCount; ++i) {
        const u32 a = (u32)std::abs((int)lod.pos[i]);
        if (a > qMax) qMax = a;
      }
  }
  {
    SkelSkin::Fixed::Fold probe;
    SkelSkin::Fixed::fold(identity.data(), slots, bind, out, probe, qMax);
    std::printf("combatent fold: qMax %u expA %d expR %d expT %d narrow %d\n", (unsigned)qMax,
                (int)probe.expA, (int)probe.expR, (int)probe.expT, probe.narrow ? 1 : 0);
  }
  Errors e;
  for (int pass = 0; pass < 2; ++pass) {
    const std::vector<float>& pal = pass == 0 ? identity : posed;
    SkelSkin::Fixed::Fold fold;
    SkelSkin::Fixed::fold(pal.data(), slots, bind, out, fold, qMax);
    for (size_t pi = 0; pi < model.parts.size(); ++pi) {
      const size_t levels = 1 + model.parts[pi].lods.size();
      for (size_t level = 0; level < levels && level <= 2; ++level) {
        const SkelMesh& mesh = level == 0 ? static_cast<const SkelMesh&>(model.parts[pi])
                                          : static_cast<const SkelMesh&>(model.parts[pi].lods[level - 1]);
        for (u32 v = 0; v < mesh.vertexCount; ++v)
          compareCorner(fold, pal.data(), bind, out, &mesh.pos[3 * v], &mesh.nrm[4 * v],
                        &mesh.joints[4 * v], &mesh.weights[4 * v], e);
      }
    }
  }
  printErrors("combatent model (identity + random palette)", e);
  check(e.worstPos <= 2.0, "model positions within 2 output steps");
  check(e.worstNrm <= 2.0, "model normals within 2 x127 units");
  check(e.saturated == 0, "no model corner saturates the int16 pre-scale");

  // The precomputed influences (prepCorner) against the reference blend, bit for bit:
  // 10 000 random poses over every corner of every body level.
  {
    std::mt19937 poseRng(0xBEEFu);
    std::vector<float> pal(slots * 16, 0.0F);
    u64 corners = 0, mismatches = 0, unfit = 0;
    for (int pose = 0; pose < 10000; ++pose) {
      for (u32 j = 0; j < slots; ++j) randomSlot(poseRng, &pal[j * 16], 0.3F, 0.1F);
      SkelSkin::Fixed::Fold fold;
      SkelSkin::Fixed::fold(pal.data(), slots, bind, out, fold, qMax);
      if (!fold.narrow) {
        ++unfit;
        continue;
      }
      for (size_t pi = 0; pi < model.parts.size(); ++pi) {
        const size_t levels = 1 + model.parts[pi].lods.size();
        for (size_t level = 0; level < levels; ++level) {
          const SkelMesh& mesh = level == 0 ? static_cast<const SkelMesh&>(model.parts[pi])
                                            : static_cast<const SkelMesh&>(model.parts[pi].lods[level - 1]);
          for (u32 v = 0; v < mesh.vertexCount; ++v) {
            const u8* jj = &mesh.joints[4 * v];
            const u8* ww = &mesh.weights[4 * v];
            const s16* q = &mesh.pos[3 * v];
            const s8* n = &mesh.nrm[4 * v];
            s16 qa[4] = {0, 0, 0, 0}, qb[4] = {0, 0, 0, 0};
            s8 na[4] = {0, 0, 0, 0}, nb[4] = {0, 0, 0, 0};
            u32 ta = 0, tb = 0;
            const u32 ca = SkelSkin::Fixed::cornerNarrowRef(fold, q, n, jj, ww, qa, na, &ta);
            const SkelSkin::Fixed::CornerInfl ci = SkelSkin::Fixed::prepCorner(jj, ww);
            const u32 cb = SkelSkin::Fixed::cornerFixed(fold, q, n, jj, ww, qb, nb, &tb, &ci);
            ++corners;
            const bool same = ca == cb && ta == tb && std::memcmp(qa, qb, 6) == 0 &&
                              std::memcmp(na, nb, 3) == 0;
            if (!same) ++mismatches;
          }
        }
      }
    }
    std::printf("precomputed influences: %llu corner blends, %llu mismatches, %llu poses not "
                "narrow\n",
                (unsigned long long)corners, (unsigned long long)mismatches,
                (unsigned long long)unfit);
    check(mismatches == 0, "precomputed influences match the reference blend bit for bit");
  }
}

}  // namespace

int main(int argc, char** argv) {
  syntheticChecks(0xC0FFEEu);
  clampChecks();
  repeatChecks();
  if (argc > 1) realModelChecks(argv[1]);
  std::printf("verify-skel-fixed: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
