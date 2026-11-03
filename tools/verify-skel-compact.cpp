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
#include <cstring>
#include <algorithm>
#include <random>
#include <vector>

#include <algorithm>
#include <cstring>

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"
#include "renderer/core/paths/path1/vif1_chain_check.hpp"
#include "loaders/3d/tskl_loader/tskl_loader.hpp"

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

// The real model: a .tskl baked by the editor (argv[1], e.g. res/models/combatent__*.tskl).
// Per part and LOD level: the model's own bind data is packed into a stream, skinned
// into it with the identity palette (the T-pose) and with a random pose, and the
// drawn stream is decoded corner by corner against the reference.
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
  const Compact::Box box = model.box;
  const Compact::Box out = skinOutputBox(model);
  const u32 slots = (u32)model.palette.size();
  check(slots <= SkelSkin::kMaxSlots, "the palette fits the fold");
  std::vector<float> identity(slots * 16, 0.0F);
  for (u32 j = 0; j < slots; ++j)
    identity[j * 16 + 0] = identity[j * 16 + 5] = identity[j * 16 + 10] = identity[j * 16 + 15] = 1.0F;
  std::mt19937 rngReal(0xC0FFEEu);
  std::vector<float> posed(slots * 16, 0.0F);
  for (u32 j = 0; j < slots; ++j) randomSlot(rngReal, &posed[j * 16], 0.3F);

  u32 corners = 0;
  float worstTruth = 0.0F, worstRef = 0.0F, worstNrm = 0.0F;
  bool inside = true;
  for (int pass = 0; pass < 2; ++pass) {
    const std::vector<float>& pal = pass == 0 ? identity : posed;
    SkelSkin::Folded folded;
    SkelSkin::foldPalette(pal.data(), slots, box, out, folded);
    for (size_t pi = 0; pi < model.parts.size(); ++pi) {
      const size_t levels = 1 + model.parts[pi].lods.size();
      for (size_t level = 0; level < levels && level <= 2; ++level) {
        const SkelMesh& mesh = level == 0 ? static_cast<const SkelMesh&>(model.parts[pi])
                                          : static_cast<const SkelMesh&>(model.parts[pi].lods[level - 1]);
        const u32 n = mesh.vertexCount;
        if (n == 0) continue;
        const bool textured = !mesh.uv.empty();
        // skinSource: the first corner with the same bind data.
        std::vector<u32> src(n);
        for (u32 v = 0; v < n; ++v) {
          src[v] = v;
          for (u32 u = 0; u < v; ++u)
            if (std::memcmp(&mesh.pos[3 * u], &mesh.pos[3 * v], 6) == 0 &&
                std::memcmp(&mesh.nrm[4 * u], &mesh.nrm[4 * v], 4) == 0 &&
                std::memcmp(&mesh.joints[4 * u], &mesh.joints[4 * v], 4) == 0 &&
                std::memcmp(&mesh.weights[4 * u], &mesh.weights[4 * v], 4) == 0) {
              src[v] = u;
              break;
            }
        }
        // The model's own floats: the truth the stream must hold.
        std::vector<float> tp(3 * (size_t)n), tn(3 * (size_t)n), tuv(textured ? 2 * (size_t)n : 0);
        decodeMesh(box, mesh, tp.data(), tn.data(), textured ? tuv.data() : nullptr);
        std::vector<float> spos(4 * (size_t)n), snrm(4 * (size_t)n), sst(4 * (size_t)n, 0.0F);
        for (u32 v = 0; v < n; ++v) {
          for (int i = 0; i < 3; ++i) {
            spos[4 * v + i] = tp[3 * v + i];
            snrm[4 * v + i] = tn[3 * v + i];
          }
          spos[4 * v + 3] = 1.0F;
          sst[4 * v + 0] = textured ? tuv[2 * v] : 0.0F;
          sst[4 * v + 1] = textured ? tuv[2 * v + 1] : 0.0F;
          sst[4 * v + 2] = 1.0F;
        }
        const StaPipCompactLayout layout =
            StaPipCompactLayout::forClass(textured, false, true, false, 48, false);
        const StaPipCompactSource source{spos.data(), sst.data(), nullptr, snrm.data(), n};
        StaPipCompactStream stream;
        check(Compact::packArrays(layout, source, out, stream), "a real part packs into a stream");
        const SkelSkin::SkelSkinPart part{mesh.pos.data(), mesh.nrm.data(), mesh.joints.data(),
                                          mesh.weights.data(), src.data(), n};
        const u32 clamped = SkelSkin::skinIntoStream(folded, part, stream);
        check(clamped != SkelSkin::kSkinRefused, "a real part skins into its stream");
        const Compact::Box sb = stream.box();
        const StaPipCompactView view(stream);
        check(view.count() == n, "the stream counts the part's corners");
        for (u32 v = 0; v < n; ++v) {
          s16 q4[4] = {mesh.pos[3 * v], mesh.pos[3 * v + 1], mesh.pos[3 * v + 2], 0};
          s8 n4[4] = {mesh.nrm[4 * v], mesh.nrm[4 * v + 1], mesh.nrm[4 * v + 2], 0};
          u8 j4[4] = {mesh.joints[4 * v], mesh.joints[4 * v + 1], mesh.joints[4 * v + 2],
                      mesh.joints[4 * v + 3]};
          u8 w4[4] = {mesh.weights[4 * v], mesh.weights[4 * v + 1], mesh.weights[4 * v + 2],
                      mesh.weights[4 * v + 3]};
          s16 refQ[4];
          s8 refN[4];
          bool refClamped = false;
          referenceCorner(pal.data(), box, sb, q4, n4, j4, w4, refQ, refN, &refClamped);
          float p4[4], nr[3];
          view.position(v, p4);
          view.normal(v, nr);
          for (int i = 0; i < 3; ++i) {
            const float world = p4[i];
            const float idx = world / sb.step - sb.origin[i] / sb.step;
            worstRef = std::max(worstRef, std::fabs(idx - (float)refQ[i]));
            worstNrm = std::max(worstNrm, std::fabs(nr[i] * 127.0F - (float)refN[i]));
            if (pass == 0) worstTruth = std::max(worstTruth, std::fabs(world - tp[3 * v + i]));
            if (world < sb.origin[i] + sb.step * (-32767.0F) ||
                world > sb.origin[i] + sb.step * 32767.0F)
              inside = false;
          }
          ++corners;
        }
        check(clamped == 0 || pass == 1, "the T-pose stays inside its box");
      }
    }
  }
  check(corners > 0, "the real model has corners");
  check(worstRef <= 1.0F, "every real corner decodes within one LSB of the reference (T-pose and posed)");
  check(worstTruth <= (float)std::ldexp(1.0, -13) + 1e-6F,
        "the T-pose stream holds the model's own positions within one step");
  check(worstNrm <= 1.0F + 1e-3F, "real normals within one int8 step of the reference");
  check(inside, "every decoded real position lies inside its box");
  std::printf("  real model: %u corners, worst index error %.2f LSB, worst T-pose error %.6f m, "
              "worst normal %.2f\n",
              corners, worstRef, worstTruth, worstNrm);
}

}  // namespace

int main(int argc, char** argv) {
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
  std::vector<u32> skinSource(kCorners, 0);
  for (u32 c = 0; c < kCorners; ++c) {
    skinSource[c] = c;
    // A third of the corners past the first hundred repeat an earlier corner
    // (the bind data's skinSource): the same data, a copied element.
    if (c >= 100 && (rng() % 3) == 0) {
      const u32 src = rng() % c;
      skinSource[c] = skinSource[src];
      for (int i = 0; i < 4; ++i) {
        q[c * 4 + i] = q[src * 4 + i];
        n[c * 4 + i] = n[src * 4 + i];
        joints[c * 4 + i] = joints[src * 4 + i];
        weights[c * 4 + i] = weights[src * 4 + i];
      }
      continue;
    }
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
  const s32 zeroOi[3] = {0, 0, 0};  // the origin index of a box at 0
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
                                                  zeroOi, gotQ, gotN);
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
      const u32 clamped = SkelSkin::skinCorner(f, cq, cn, j0, w0, zeroOi, gq, gn);
      check(clamped == 1, "a corner twice past the box is counted once");
      check(gq[0] == (sign > 0 ? 32767 : -32767),
            "a corner twice past the box clamps to +-32767 (no wrap)");
      // Index 0 decodes to the bind origin (0.25 and -0.5 in y and z): 2048 and
      // -4096 output steps of 2^-13 m.
      check(gq[1] == 2048 && gq[2] == -4096,
            "the other axes of a clamped corner keep their value");
    }
  }

  // Skinning into a stream: the same corners, two poses into ONE stream. The
  // stream is packed once from the bind data (its ST and colour runs are the
  // model's); each pose rewrites the positions and normals only.
  {
    const u32 kPackage = 48;
    const StaPipCompactLayout layout =
        StaPipCompactLayout::forClass(true, false, true, false, kPackage, false);
    std::vector<float> spos(kCorners * 4, 0.0F), snrm(kCorners * 4, 0.0F),
        sst(kCorners * 4, 0.0F);
    std::uniform_real_distribution<float> uvd(0.0F, 1.0F);
    for (u32 c = 0; c < kCorners; ++c) {
      for (int i = 0; i < 3; ++i) {
        spos[c * 4 + i] = bind.origin[i] + bind.step * (float)q[c * 4 + i];
        snrm[c * 4 + i] = (float)n[c * 4 + i] / 127.0F;
      }
      sst[c * 4 + 0] = uvd(rng);
      sst[c * 4 + 1] = uvd(rng);
      sst[c * 4 + 2] = 1.0F;
    }
    const StaPipCompactSource source{spos.data(), sst.data(), nullptr, snrm.data(), kCorners};
    StaPipCompactStream stream;
    check(Compact::packArrays(layout, source, out, stream), "the stream packs from the bind data");
    std::vector<s16> q3(kCorners * 3);  // the core reads bind positions 3 a corner
    for (u32 c = 0; c < kCorners; ++c)
      for (int i = 0; i < 3; ++i) q3[c * 3 + i] = q[c * 4 + i];
    const SkelSkin::SkelSkinPart part{q3.data(), n.data(), joints.data(), weights.data(),
                                      skinSource.data(), kCorners};
    const u8 normalSlot = (u8)stream.layout().indexOf(Compact::Attr::Normal);

    // The stream's bytes, the block and everything in it, without the position
    // and normal payloads: those are the only bytes a pose may change.
    auto maskedBytes = [&](const StaPipCompactStream& s) {
      std::vector<u8> masked((size_t)s.qwords() * 16, 0);
      for (u32 p = 0; p < s.packageCount(); ++p)
        for (u8 a : {(u8)0, normalSlot}) {
          const u8* base = (const u8*)s.data();
          const u8* run = s.run(p, a);
          const size_t at = (size_t)(run - base);
          for (u32 i = 0; i < s.runQwords(p, a) * 16; ++i) masked[at + i] = 1;
        }
      std::vector<u8> bytes((const u8*)s.data(), (const u8*)s.data() + (size_t)s.qwords() * 16);
      for (size_t i = 0; i < bytes.size(); ++i)
        if (masked[i]) bytes[i] = 0;
      return bytes;
    };

    // Walks the whole block through the VIF1 model: the bag setup, then per package
    // the entry, every run (header, its data words, its padding) and the kick.
    auto walkBlock = [&](StaPipCompactStream& s) {
      Vif1ChainCheck::Stream chain;
      u32 setup[Compact::kBagSetupWords];
      Compact::bagSetup(s.box(), setup);
      for (u32 w : setup) chain.word(w);
      if (!chain.result || chain.result.pending != 0 || chain.cl != 1 || chain.wl != 1)
        return false;
      for (u32 p = 0; p < s.packageCount(); ++p) {
        const u32 n = s.packageVertices(p);
        const u32* entry = (const u32*)s.entry(p);
        for (u32 w = 0; w < 12; ++w) chain.word(entry[w]);
        if (!chain.result || chain.result.pending != 0) return false;
        for (u8 a = 0; a < s.layout().attrCount; ++a) {
          u32 header[4];
          std::memcpy(header, s.runHeader(p, a), sizeof(header));
          chain.word(header[0]), chain.word(header[1]), chain.word(header[2]);
          chain.word(header[3]);
          const u32 dataWords = (n * Compact::elementBytes(s.layout().attrs[a]) + 3) / 4;
          if (!chain.result || chain.result.pending != dataWords) return false;
          const u32* payload = (const u32*)s.run(p, a);
          for (u32 w = 0; w < dataWords; ++w) chain.word(payload[w]);
          if (!chain.result || chain.result.pending != 0) return false;
          for (u32 w = dataWords; w < s.runQwords(p, a) * 4; ++w) chain.word(payload[w]);
          if (!chain.result || chain.result.pending != 0 || chain.cl != 1 || chain.wl != 1)
            return false;
        }
        const u32* kick = (const u32*)s.kick(p);
        for (u32 w = 0; w < 4; ++w) chain.word(kick[w]);
        if (!chain.result || chain.result.pending != 0) return false;
      }
      return true;
    };

    u32 clampedRef = 0, clampedGot = 0, streamPosDiff = 0, streamNrmDiff = 0;
    bool decodeOk = true;
    std::vector<u8> before;
    for (int pose = 0; pose < 2; ++pose) {
      for (u32 j = 0; j < kSlots; ++j) randomSlot(rng, &palette[j * 16], 0.3F);
      SkelSkin::Folded folded;
      SkelSkin::foldPalette(palette.data(), kSlots, bind, out, folded);
      const u32 versionBefore = stream.contentVersion();
      clampedGot = SkelSkin::skinIntoStream(folded, part, stream);
      check(clampedGot != SkelSkin::kSkinRefused, "the stream takes the part");
      if (pose == 1) {
        check(stream.contentVersion() != versionBefore, "a second pose bumps the content version");
        check(maskedBytes(stream) == before,
              "a pose changes only the position and normal payloads (ST, colour, headers, entries, kicks)");
      }
      clampedRef = 0;
      const StaPipCompactView view(stream);
      check(view.count() == kCorners, "the view counts the corners");
      for (u32 c = 0; c < kCorners; ++c) {
        s16 refQ[4];
        s8 refN[4];
        bool refClamped = false;
        referenceCorner(palette.data(), bind, stream.box(), &q[c * 4], &n[c * 4],
                        &joints[c * 4], &weights[c * 4], refQ, refN, &refClamped);
        if (refClamped) ++clampedRef;
        float pos[4], nrm[3];
        view.position(c, pos);
        view.normal(c, nrm);
        const Compact::Box sb = stream.box();
        for (int i = 0; i < 3; ++i) {
          const float idx = pos[i] / sb.step - sb.origin[i] / sb.step;
          if (std::fabs(idx - (float)refQ[i]) > 1.0F) decodeOk = false;
          const float dn = std::fabs(nrm[i] * 127.0F - (float)refN[i]);
          if (dn > 1.0F + 1e-3F) decodeOk = false;
          streamNrmDiff = std::max(streamNrmDiff, (u32)std::ceil(dn));
          streamPosDiff = std::max(streamPosDiff, (u32)std::ceil(std::fabs(idx - (float)refQ[i])));
        }
      }
      check(decodeOk, "every decoded corner within one LSB of the reference (positions and normals)");
      check(clampedGot == clampedRef, "out-of-box corners counted as the reference counts them");
      // A decoded position lies inside its part's box (the boxes the stream
      // finished with).
      bool inside = true;
      for (u32 c = 0; c < kCorners; ++c) {
        float mn[3], mx[3];
        stream.partBox(c / (kPackage / 3), mn, mx);
        float pos[4];
        view.position(c, pos);
        for (int i = 0; i < 3; ++i)
          if (pos[i] < mn[i] || pos[i] > mx[i]) inside = false;
      }
      check(inside, "every decoded position lies inside its part box");
      check(walkBlock(stream), "the stream walks as VIF1 data: entries, runs, kicks");
      if (pose == 0) before = maskedBytes(stream);
    }

    // A body translated 5 m off the model origin (a ragdoll fling) is boxed by its
    // own pose: no corner is out of its box and each decodes within one LSB of the
    // reference in the box it was written in.
    {
      std::vector<float> moved(kSlots * 16);
      for (u32 j = 0; j < kSlots; ++j) randomSlot(rng, &moved[j * 16], 0.3F);
      for (u32 j = 0; j < kSlots; ++j) moved[j * 16 + 12] += 5.0F;
      SkelSkin::Folded folded;
      SkelSkin::foldPalette(moved.data(), kSlots, bind, out, folded);
      const u32 got = SkelSkin::skinIntoStream(folded, part, stream);
      check(got == 0, "a body translated 5 m off the origin has no out-of-box corner");
      const Compact::Box sb = stream.box();
      const StaPipCompactView view(stream);
      bool close = true;
      for (u32 c = 0; c < kCorners; ++c) {
        s16 refQ[4];
        s8 refN[4];
        bool refClamped = false;
        referenceCorner(moved.data(), bind, sb, &q[c * 4], &n[c * 4], &joints[c * 4],
                        &weights[c * 4], refQ, refN, &refClamped);
        float pos[4];
        view.position(c, pos);
        for (int i = 0; i < 3; ++i)
          if (std::fabs(pos[i] / sb.step - sb.origin[i] / sb.step - (float)refQ[i]) > 1.0F)
            close = false;
      }
      check(close, "a translated pose decodes within one LSB of the reference in its own box");
    }

    // A pose spanning more than 8 m: its corners past the box clamp, are counted,
    // and decode to the box edge - never wrap.
    {
      std::vector<float> wide(kSlots * 16);
      for (u32 j = 0; j < kSlots; ++j) randomSlot(rng, &wide[j * 16], 0.0F);
      for (u32 j = 0; j < kSlots; ++j) wide[j * 16 + 12] += (j % 2 == 0) ? -6.0F : 6.0F;
      SkelSkin::Folded folded;
      SkelSkin::foldPalette(wide.data(), kSlots, bind, out, folded);
      const u32 got = SkelSkin::skinIntoStream(folded, part, stream);
      check(got > 0, "a pose spanning more than 8 m counts its out-of-box corners");
      const Compact::Box sb = stream.box();
      const StaPipCompactView view(stream);
      bool edge = true;
      for (u32 c = 0; c < kCorners; ++c) {
        s16 refQ[4];
        s8 refN[4];
        bool refClamped = false;
        referenceCorner(wide.data(), bind, sb, &q[c * 4], &n[c * 4], &joints[c * 4],
                        &weights[c * 4], refQ, refN, &refClamped);
        float pos[4];
        view.position(c, pos);
        for (int i = 0; i < 3; ++i)
          if (std::fabs(pos[i] / sb.step - sb.origin[i] / sb.step - (float)refQ[i]) > 1.0F)
            edge = false;
      }
      check(edge, "clamped corners decode to the box edge, never wrap");
    }
  }

  if (argc > 1) realModelChecks(argv[1]);
  std::printf("verify-skel-compact: %d checks, %d failed (positions max %u LSB, normals max %u)\n",
              checks, failures, maxPosDiff, maxNrmDiff);
  return failures == 0 ? 0 : 1;
}
