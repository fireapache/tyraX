// Host verifier for the batch skin kernel (renderer/3d/mesh/dynamic/skel_batch.hpp).
//
// For every part and LOD of the real combatent model (argv[1], a .tskl), over 10 000 random
// poses (every 1000th with a 2000 m joint shift, which the noSat fold refuses):
//  (a) the lane model (runPose + finishPose over the bind-time Part) is BIT-EQUAL to the
//      integer definition cornerBatchRef on every unique corner: outputs, flags, box;
//  (b) against the double-precision reference (the same one verify-skel-fixed uses),
//      positions within 2 output steps and normals within 2 x127 units (worst and mean);
//  (c) out-of-box uniques clamp to +-32767 with the sign of the reference and are counted.
// It also counts the MMI instructions the kernel issues per block and per unique (the
// sequence the lane model models; counted from the same steps, not measured on the EE).
//
// Build and run from the TyraX root (the runner passes the .tskl):
//   build/verify-skel-batch res/models/combatent__ovrb22b.tskl
//   (prints "verify-skel-batch: <n> checks, 0 failed")

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <vector>

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_batch.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

using Tyra::SkelSkin::Fixed::Fold;

namespace {

using namespace Tyra;
using namespace Tyra::SkelSkin;

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

// The double-precision reference of one corner (as verify-skel-fixed's referenceCorner).
void referenceCorner(const float* palette, const Compact::Box& bind, const Compact::Box& out,
                     const s16* q, const s8* n, const u8* joints, const u8* weights, double pos[3],
                     double nrm[3]) {
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
  for (int i = 0; i < 3; ++i) pos[i] = pp[i] / out.step - roundAway(out.origin[i] / out.step);
  const double len = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
  for (int i = 0; i < 3; ++i) nrm[i] = len > 0.0 ? nn[i] / len * 127.0 : 0.0;
}

// A column-major palette slot: rigid rotation, scale near 1, translation up to maxShift.
void randomSlot(std::mt19937& rng, float* m, float maxShift, float scaleSpread) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  double qv[4] = {gauss(rng), gauss(rng), gauss(rng), gauss(rng)};
  const double len = std::sqrt(qv[0] * qv[0] + qv[1] * qv[1] + qv[2] * qv[2] + qv[3] * qv[3]);
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

struct Tally {
  u64 uniques = 0;
  u64 meshes = 0;
  u64 compared = 0;         // uniques held to the bit-equal and the float checks
  u64 skippedPoses = 0;     // (mesh, pose) pairs not lane-safe (the scalar path takes them)
  u64 bitMismatch = 0;      // lane model vs cornerBatchRef
  u64 scalarMismatch = 0;   // batch uniques vs cornerFixed (the scalar blend they replace)
  u64 satSeen = 0;          // ref joint terms outside int16 (must be 0 on compared poses)
  u64 truncations = 0;      // lane-model values outside int16 (must be 0 on compared poses)
  u64 wideFlag = 0;         // uniques taking the scalar fallback (short normal)
  u64 clamped = 0;          // uniques with a clamped axis
  u64 outsideBad = 0;       // clamped uniques whose reference is not outside, or sign differs
  double worstPos = 0.0, sumPos = 0.0, worstNrm = 0.0, sumNrm = 0.0;
  u64 posCount = 0, nrmCount = 0;
  u64 entries = 0, blocks = 0, groups = 0, padding = 0;
  u64 boxMismatch = 0;
};

void checkMesh(const Fold& fold, const float* pal, const Compact::Box& bind,
               const Compact::Box& out, const Batch::Part& part, Batch::Products& prod,
               std::vector<Batch::Image>& img, std::vector<s16>& opos, std::vector<s8>& onrm,
               std::vector<u32>& oflags, std::vector<s16>& boxRef, Tally& t, const SkelMesh& mesh) {
  const bool lanesFit = fold.noSat && fold.narrow && fold.narrowN;
  if (!lanesFit) {
    ++t.skippedPoses;
    return;
  }
  boxRef.assign(4 * part.unique, 0);
  Batch::buildImages(fold, part, img);
  if (!Batch::runPoseFor(fold, part, img, prod)) {
    ++t.skippedPoses;  // a shift outside the kernel's instantiations: the scalar blend takes it
    return;
  }
  opos.assign(4 * part.unique, 0);
  onrm.assign(4 * part.unique, 0);
  oflags.assign(part.unique, 0);
  const u32 clampedU = Batch::finishPose(fold, part, prod, opos.data(), onrm.data(), oflags.data());
  (void)clampedU;
  t.truncations += prod.truncations;
  for (u32 u = 0; u < part.unique; ++u) {
    const u32 c = part.uniqueCorner[u];
    const s16* q = &mesh.pos[3 * c];
    const s8* n = &mesh.nrm[4 * c];
    const u8* jj = &mesh.joints[4 * c];
    const u8* ww = &mesh.weights[4 * c];
    s16 rp[4] = {0, 0, 0, 0};
    s8 rn[4] = {0, 0, 0, 0};
    u32 sat = 0;
    const u32 rf = Batch::cornerBatchRef(fold, q, n, jj, ww, rp, rn, &sat);
    ++t.uniques;
    t.satSeen += sat;
    // boxRef[4u+3] marks a unique that is not a lane-model case (saturated or short normal).
    boxRef[4 * u + 3] = (sat != 0 || (rf & 2)) ? 1 : 0;
    for (int i = 0; i < 3; ++i) boxRef[4 * u + i] = rp[i];
    if (sat != 0) continue;  // a term outside int16 is not a lane-model case
    const u32 lf = oflags[u];
    const bool flagsEqual = lf == rf;
    bool same = flagsEqual;
    if (same && (rf & 2) == 0) {
      same = std::memcmp(&opos[4 * u], rp, 6) == 0;
      if (same && (rf & 1) == 0) same = std::memcmp(&onrm[4 * u], rn, 3) == 0;
    }
    if (!same) ++t.bitMismatch;
    // The EE path writes these uniques in place of the scalar blend (cornerFixed): they must
    // be the same bytes and the same clamp.
    if (!(rf & 2)) {
      s16 fa[4] = {0, 0, 0, 0};
      s8 fb[4] = {0, 0, 0, 0};
      const u32 cf = Fixed::cornerFixed(fold, q, n, jj, ww, fa, fb, nullptr, nullptr);
      if ((cf & 1) != (rf & 1) || std::memcmp(fa, rp, 6) != 0 || std::memcmp(fb, rn, 3) != 0)
        ++t.scalarMismatch;
    }
    if (rf & 2) {
      ++t.wideFlag;
      continue;
    }
    if (rf & 1) ++t.clamped;
    // Float reference (only where the reference is inside the box, or clamps to the sign).
    double fp[3], fn[3];
    referenceCorner(pal, bind, out, q, n, jj, ww, fp, fn);
    bool outside = false;
    for (int i = 0; i < 3; ++i) {
      if (std::fabs(fp[i]) > 32767.5) {  // beyond the rounding to 32767: outside
        outside = true;
        const s16 want = fp[i] > 0 ? 32767 : -32767;
        if (rp[i] != want) ++t.outsideBad;
        continue;
      }
      const double err = std::fabs((double)rp[i] - fp[i]);
      if (err > t.worstPos) t.worstPos = err;
      t.sumPos += err;
      ++t.posCount;
    }
    if (outside && !(rf & 1)) ++t.outsideBad;
    for (int i = 0; i < 3; ++i) {
      const double err = std::fabs((double)rn[i] - fn[i]);
      if (err > t.worstNrm) t.worstNrm = err;
      t.sumNrm += err;
      ++t.nrmCount;
    }
    ++t.compared;
  }
  // Box: the min and max of the written positions over the corners (the lane model's
  // outputs, through each corner's unique), against the same over the reference's uniques.
  // Only when every unique of the mesh is compared (no fallback, no saturation).
  bool allValid = true;
  for (u32 u = 0; u < part.unique; ++u)
    if (boxRef[4 * u + 3] != 0) allValid = false;
  if (allValid) {
    s32 la[3] = {32767, 32767, 32767}, ha[3] = {-32768, -32768, -32768};
    s32 lb[3] = {32767, 32767, 32767}, hb[3] = {-32768, -32768, -32768};
    for (u32 c = 0; c < part.corners; ++c) {
      const u32 u = part.cornerUnique[c];
      for (int i = 0; i < 3; ++i) {
        const s32 v = opos[4 * u + i];
        if (v < la[i]) la[i] = v;
        if (v > ha[i]) ha[i] = v;
      }
    }
    for (u32 u = 0; u < part.unique; ++u)
      for (int i = 0; i < 3; ++i) {
        const s32 v = boxRef[4 * u + i];
        if (v < lb[i]) lb[i] = v;
        if (v > hb[i]) hb[i] = v;
      }
    for (int i = 0; i < 3; ++i)
      if (la[i] != lb[i] || ha[i] != hb[i]) ++t.boxMismatch;
  }
  ++t.meshes;
  t.entries += part.entries;
  t.groups += part.groups.size();
  t.blocks += part.entries / Batch::kBlock;
  t.padding += part.padding;
}

void printTally(const char* label, const Tally& t) {
  std::printf("%s: %llu meshes, %llu uniques, compared %llu, skipped (not lane-safe) %llu; "
              "bit mismatches %llu, vs cornerFixed %llu, sat %llu, truncations %llu, scalar-fallback uniques %llu, "
              "clamped %llu, outside/sign bad %llu, box mismatches %llu\n",
              label, (unsigned long long)t.meshes, (unsigned long long)t.uniques,
              (unsigned long long)t.compared, (unsigned long long)t.skippedPoses,
              (unsigned long long)t.bitMismatch, (unsigned long long)t.scalarMismatch, (unsigned long long)t.satSeen,
              (unsigned long long)t.truncations, (unsigned long long)t.wideFlag,
              (unsigned long long)t.clamped, (unsigned long long)t.outsideBad,
              (unsigned long long)t.boxMismatch);
  std::printf("%s: positions worst %.3f mean %.4f steps (%llu axes); normals worst %.3f mean "
              "%.4f x127 (%llu axes)\n",
              label, t.worstPos, t.posCount ? t.sumPos / (double)t.posCount : 0.0,
              (unsigned long long)t.posCount, t.worstNrm,
              t.nrmCount ? t.sumNrm / (double)t.nrmCount : 0.0, (unsigned long long)t.nrmCount);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: verify-skel-batch <model.tskl>\n");
    return 2;
  }
  SkelModel model;
  bool loaded = false;
  if (FILE* f = std::fopen(argv[1], "rb")) {
    std::vector<u8> bytes;
    u8 chunk[8192];
    size_t got;
    while ((got = std::fread(chunk, 1, sizeof(chunk), f)) > 0)
      bytes.insert(bytes.end(), chunk, chunk + got);
    std::fclose(f);
    loaded = TsklLoader::parse(bytes.data(), bytes.size(), model);
  }
  check(loaded, "the real model parses");
  if (!loaded) {
    std::printf("verify-skel-batch: %d checks, %d failed\n", checks, failures);
    return 1;
  }
  const Compact::Box bind = model.box;
  const Compact::Box out = skinOutputBox(model);
  const u32 slots = (u32)model.palette.size();
  check(slots <= kMaxSlots, "the palette fits the fold");

  // Bind-time: one Part per mesh (every part and LOD level), the first identical corner of each.
  struct Level {
    const SkelMesh* mesh;
    Batch::Part part;
    std::vector<u32> skinSource;
  };
  std::vector<Level> levels;
  u32 qMax = 1;
  for (const SkelPart& part : model.parts) {
    std::vector<const SkelMesh*> meshes;
    meshes.push_back(&part);
    for (const SkelLod& lod : part.lods) meshes.push_back(&lod);
    for (const SkelMesh* mesh : meshes) {
      for (u32 i = 0; i < 3 * mesh->vertexCount; ++i) {
        const u32 a = (u32)std::abs((int)mesh->pos[i]);
        if (a > qMax) qMax = a;
      }
      Level lv;
      lv.mesh = mesh;
      lv.skinSource.resize(mesh->vertexCount);
      std::map<std::array<int, 15>, u32> first;
      for (u32 v = 0; v < mesh->vertexCount; ++v) {
        // pos (3), nrm (4), joints (4), weights (4): the corner's bind data, as skinSource's key.
        std::array<int, 15> key{};
        for (int i = 0; i < 3; ++i) key[i] = mesh->pos[3 * v + i];
        for (int i = 0; i < 4; ++i) key[3 + i] = mesh->nrm[4 * v + i];
        for (int i = 0; i < 4; ++i) key[7 + i] = mesh->joints[4 * v + i];
        for (int i = 0; i < 4; ++i) key[11 + i] = mesh->weights[4 * v + i];
        const auto it = first.find(key);
        if (it == first.end()) {
          first[key] = v;
          lv.skinSource[v] = v;
        } else {
          lv.skinSource[v] = it->second;
        }
      }
      Batch::buildPart(mesh->pos.data(), mesh->nrm.data(), mesh->joints.data(), mesh->weights.data(),
                       lv.skinSource.data(), mesh->vertexCount, lv.part);
      levels.push_back(lv);
    }
  }

  // The fixed-point folds' per-model pre-scale bound (as verify-skel-fixed sets it).
  std::vector<float> identity(slots * 16, 0.0F);
  for (u32 j = 0; j < slots; ++j)
    identity[j * 16 + 0] = identity[j * 16 + 5] = identity[j * 16 + 10] = identity[j * 16 + 15] = 1.0F;

  Tally total;
  Batch::Products prod;
  std::vector<Batch::Image> img;
  std::vector<s16> opos;
  std::vector<s8> onrm;
  std::vector<u32> oflags;
  std::vector<s16> boxRef;
  std::mt19937 rng(0xBA7C4u);
  std::vector<float> pal(slots * 16, 0.0F);
  for (int pose = 0; pose < 10000; ++pose) {
    if (pose == 0)
      pal = identity;
    else
      for (u32 j = 0; j < slots; ++j) randomSlot(rng, &pal[j * 16], 0.3F, 0.1F);
    if (pose % 1000 == 999)
      for (u32 j = 0; j < slots; ++j)
        for (int c = 0; c < 3; ++c) pal[j * 16 + 12 + c] += 2000.0F;
    Fold fold;
    SkelSkin::Fixed::fold(pal.data(), slots, bind, out, fold, qMax);
    for (Level& lv : levels)
      checkMesh(fold, pal.data(), bind, out, lv.part, prod, img, opos, onrm, oflags, boxRef, total,
                *lv.mesh);
  }
  printTally("batch lane model vs reference, 10000 poses", total);

  // Out-of-box: the same poses with the output origin moved 3 m, so some uniques land outside
  // the +-32767 steps (4 m at the 2^-13 m step): they must clamp, be counted and keep their sign.
  Compact::Box moved = out;
  moved.origin[0] += 3.0F;
  moved.origin[1] -= 3.0F;
  Tally clip;
  std::mt19937 rng2(0xC1D2u);
  for (int pose = 0; pose < 200; ++pose) {
    for (u32 j = 0; j < slots; ++j) randomSlot(rng2, &pal[j * 16], 0.3F, 0.1F);
    Fold fold;
    SkelSkin::Fixed::fold(pal.data(), slots, bind, moved, fold, qMax);
    for (Level& lv : levels)
      checkMesh(fold, pal.data(), bind, moved, lv.part, prod, img, opos, onrm, oflags, boxRef, clip,
                *lv.mesh);
  }
  printTally("batch lane model, output box moved 3 m (200 poses)", clip);

  // The MMI instruction count per block and per unique, from the steps the lane model runs:
  // 7 loads of the block (qx, qy, qz, nx, ny, nz, w); per output row, positions: 3 PMADDH/PMULTH,
  // 2 PMFHL, per half (even, odd): PSRAVW + PADDW(b) + PADDW(round) + PSRAW = 4 (8 for both),
  // interleave 3 (PSLLW, PAND, POR), PMULTH(weight) 1, 2 stores = 19; normals: 3 + 2 PMFHL,
  // PSLLW x2 (x16), PSRAVW x2, interleave 3, PMULTH 1, 2 stores = 15. Per row pair 34; per block
  // 7 + 3 x 34 = 109. Per joint group: 21 quadword loads of the images + 3 splats of b.
  const u64 blockInstr = 7 + 3 * 34;
  const u64 groupInstr = 21 + 3;
  const double perUnique = (double)(total.blocks * blockInstr + total.groups * groupInstr) /
                           (double)(total.uniques ? total.uniques : 1);
  std::printf("batch kernel per pose: %llu blocks, %llu joint groups, %llu padding entries "
              "(%.1f %% of %llu entries); MMI-class instructions %.0f per unique (%llu per block)\n",
              (unsigned long long)total.blocks, (unsigned long long)total.groups,
              (unsigned long long)total.padding,
              total.entries ? 100.0 * (double)total.padding / (double)total.entries : 0.0,
              (unsigned long long)total.entries, perUnique, (unsigned long long)blockInstr);

  check(total.bitMismatch == 0, "the lane model is bit-equal to cornerBatchRef on every unique");
  check(total.scalarMismatch == 0, "batch uniques equal cornerFixed (the scalar blend they replace)");
  check(total.truncations == 0, "no lane value leaves int16 on the lane-safe poses");
  check(total.outsideBad == 0, "an out-of-box unique clamps to +-32767 with the reference's sign");
  check(total.boxMismatch == 0, "the part boxes equal the reference boxes");
  check(total.compared > 0, "some uniques are compared");
  check(clip.bitMismatch == 0, "moved box: the lane model is bit-equal to cornerBatchRef");
  check(clip.clamped > 0, "moved box: out-of-box uniques occur and clamp");
  check(clip.outsideBad == 0, "moved box: clamps keep the reference's sign at +-32767");
  check(total.worstPos <= 2.0, "batch positions within 2 output steps of the float reference");
  check(total.worstNrm <= 2.0, "batch normals within 2 x127 units of the float reference");
  std::printf("verify-skel-batch: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
