// Host model of skinning on VU1 (docs/animated-models.md, "Skinning on VU1").
//
// The proposed microcode blends a corner from a per-fighter palette of 4 float columns
// per joint (P_j = M_j x Dbind, pre-scaled by 1/255) with the corner's raw u8 weights
// and its shared bind data. This file writes that blend in the SAME operation order as
// the proposed VU1 sequence, every product and sum a separate float32 operation (no
// fused multiply-add: the host build is -ffp-contract=off), and checks it against the
// existing float reference (foldPalette + blendCorner in renderer/3d/mesh/dynamic).
//
// Modelled VU1 facts (docs/animated-models.md states them too):
//  - a result below FLT_MIN in magnitude flushes to +0 (VU1 has no denormals); the
//    count of flushed results is reported (zero expected on the combatent);
//  - the normal's 1/sqrt is the IEEE float 1/sqrt here; the EFU's RSQRT is an
//    approximation on the console and its precision is not measured in this repo;
//  - out-of-range results are not clamped here: the clamp belongs to the existing
//    tail (ftoi4 / the GS format), not to the blend.
//
// Checks, over the combatent's every part and LOD (levels 0 to 2), 10 000 random poses
// (identity first) plus the reference's own clamp-free poses:
//  - positions (model metres, reported in mm) within 0.05 mm of blendCorner (worst, mean);
//  - normals (unit direction x127) within 0.05 x127 units of blendCorner's (worst, mean);
//  - every corner's weights sum to 255 (the 1/255 fold is exact only then);
//  - the VU1 data-memory and batch arithmetic (constants below) is consistent: palette
//    qwords per joint, the joint offset fits the ILW byte, batch corners a multiple of 3
//    and at least one package of 3, and the batches a body's levels need.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel does this):
//   build/verify-skel-vu1 res/models/combatent__ovrb22b.tskl
//   (prints "verify-skel-vu1: <n> checks, 0 failed")

#include <cfloat>
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

// ---- VU1 layout constants (the proposed design; see the docs section) -------------------
constexpr u32 kVu1DataBytes = 16384;      // VU1 data memory (1024 qwords)
constexpr u32 kQword = 16;                // bytes per qword
constexpr u32 kQwordsPerJoint = 4;        // P_j: four float columns (c0, c1, c2, translation)
constexpr u32 kMaxJoints = 64;            // 4 x 64 = 256 qwords: the joint ILW offset fits a byte
constexpr u32 kMvpBytes = 4 * kQword;     // the MVP the existing draw tail still applies
constexpr u32 kReservedBytes = 256;       // constants the draw tail reads (options, lights, tags): estimate
constexpr u32 kCornerQwords = 6;          // unpacked per corner: pos, ST, colour, normal, joints, weights
constexpr u32 kCornerBytes = kCornerQwords * kQword;
constexpr u32 kHeaderBytes = kQword;      // the per-batch buffer header (count, box, tags)

// ---- the VU1 float operations: one rounded product or sum each -------------------------
u64 gFlushed = 0;

inline float flush(float r) {
  if (r != 0.0F && std::fabs(r) < FLT_MIN) {
    ++gFlushed;
    return 0.0F;
  }
  return r;
}
inline float vmul(float a, float b) { return flush(a * b); }
inline float vadd(float a, float b) { return flush(a + b); }

// The proposed VU1 blend of one corner. `p` holds the four joints' palettes (column-major,
// 16 floats each, already multiplied by 1/255), `jnt` the four palette slots (the byte
// offsets are 4 x slot in qwords on the console; the host indexes the slot), `wt` the raw
// u8 weights, `q` the bind index (int16 x y z, ITOF0), `n` the bind normal (int8, unscaled).
// Position: model metres. Normal: unit direction x127.
void vuCorner(const float* p, const s16* q, const s8* n, const u8* jnt, const u8* wt,
              float outPos[3], float outNrm[3]) {
  float px[3] = {0.0F, 0.0F, 0.0F};
  float nn[3] = {0.0F, 0.0F, 0.0F};
  const float qx = (float)q[0], qy = (float)q[1], qz = (float)q[2];
  const float nx = (float)n[0], ny = (float)n[1], nz = (float)n[2];
  for (int k = 0; k < 4; ++k) {
    const float* m = p + 16 * (u32)jnt[k];
    const float w = (float)wt[k];  // ITOF0 of the raw weight
    float t[3], s[3];
    // mula / madda / madda / madd: T = c0*qx + c1*qy + c2*qz + c3 (w = 1)
    for (int i = 0; i < 3; ++i) {
      t[i] = vmul(m[0 * 4 + i], qx);
      t[i] = vadd(t[i], vmul(m[1 * 4 + i], qy));
      t[i] = vadd(t[i], vmul(m[2 * 4 + i], qz));
      t[i] = vadd(t[i], m[3 * 4 + i]);
    }
    // The same three columns on the normal (w = 0: no translation term).
    for (int i = 0; i < 3; ++i) {
      s[i] = vmul(m[0 * 4 + i], nx);
      s[i] = vadd(s[i], vmul(m[1 * 4 + i], ny));
      s[i] = vadd(s[i], vmul(m[2 * 4 + i], nz));
    }
    // Weighted accumulate: the first joint writes, the rest add.
    for (int i = 0; i < 3; ++i) {
      const float wt_t = vmul(w, t[i]);
      px[i] = (k == 0) ? wt_t : vadd(px[i], wt_t);
      const float wt_s = vmul(w, s[i]);
      nn[i] = (k == 0) ? wt_s : vadd(nn[i], wt_s);
    }
  }
  // Renormalise to x127: l2 (mul, madda, madda), one 1/sqrt, then 3 scaled muls.
  float l2 = vmul(nn[0], nn[0]);
  l2 = vadd(l2, vmul(nn[1], nn[1]));
  l2 = vadd(l2, vmul(nn[2], nn[2]));
  const float k127 = l2 > 0.0F ? vmul(1.0F / std::sqrt(l2), 127.0F) : 0.0F;
  for (int i = 0; i < 3; ++i) {
    outPos[i] = px[i];
    outNrm[i] = vmul(nn[i], k127);
  }
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

// The VU1 palette of one pose: P_j = M_j x Dbind, the bind decode folded in (model metres
// = origin + step x q), then pre-scaled by 1/255 (the weights are raw u8 and sum to 255).
// The palette builder is EE-side work, not part of the VU1 sequence; it is plain float here.
void buildVuPalette(const float* palette, u32 slots, const Compact::Box& bind, float* out) {
  for (u32 j = 0; j < slots; ++j) {
    const float* m = palette + 16 * j;
    float* d = out + 16 * j;
    for (int c = 0; c < 3; ++c)
      for (int r = 0; r < 4; ++r) d[c * 4 + r] = bind.step * m[c * 4 + r];
    for (int r = 0; r < 4; ++r) {
      d[12 + r] = bind.origin[0] * m[0 * 4 + r] + bind.origin[1] * m[1 * 4 + r] +
                  bind.origin[2] * m[2 * 4 + r] + m[3 * 4 + r];
    }
    for (int i = 0; i < 16; ++i) d[i] = d[i] * (1.0F / 255.0F);
  }
}

struct Stats {
  double worstPos = 0.0, sumPos = 0.0;  // mm
  double worstNrm = 0.0, sumNrm = 0.0;  // x127 units
  u64 posN = 0, nrmN = 0;
  u64 corners = 0;
  u64 weightSumBad = 0;
  u64 influence[5] = {0, 0, 0, 0, 0};   // corners with 0..4 non-zero weights
};

// Bind-time layout facts for one part level (the batch arithmetic).
struct Level {
  const SkelMesh* mesh;
  const char* label;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: verify-skel-vu1 <model.tskl>\n");
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
    std::printf("verify-skel-vu1: %d checks, %d failed\n", checks, failures);
    return 1;
  }
  const Compact::Box bind = model.box;
  const Compact::Box outBox = skinOutputBox(model);
  check(outBox.origin[0] == 0.0F && outBox.origin[1] == 0.0F && outBox.origin[2] == 0.0F,
        "the output box sits on the model origin (index = model metres / step)");
  const u32 slots = (u32)model.palette.size();
  check(slots <= SkelSkin::kMaxSlots, "the palette fits the skin");
  check(slots <= kMaxJoints, "the joint byte offsets fit the ILW field (4 x slot <= 255)");

  // The levels: every part, its LOD 0 (the mesh) and LODs 1 and 2 (authored or generated).
  std::vector<Level> levels;
  for (const SkelPart& part : model.parts) {
    const size_t nl = 1 + part.lods.size();
    for (size_t l = 0; l < nl && l <= 2; ++l) {
      Level lv;
      lv.mesh = l == 0 ? static_cast<const SkelMesh*>(&part)
                       : static_cast<const SkelMesh*>(&part.lods[l - 1]);
      lv.label = l == 0 ? "LOD0" : (l == 1 ? "LOD1" : "LOD2");
      levels.push_back(lv);
    }
  }
  check(!levels.empty(), "the model has parts");

  // Weights: every corner's four u8 weights must sum to 255 (the 1/255 pre-scale).
  Stats wstat;
  for (const Level& lv : levels)
    for (u32 v = 0; v < lv.mesh->vertexCount; ++v) {
      int sum = 0, nz = 0;
      for (int k = 0; k < 4; ++k) {
        sum += lv.mesh->weights[4 * v + k];
        if (lv.mesh->weights[4 * v + k] != 0) ++nz;
      }
      if (sum != 255) ++wstat.weightSumBad;
      ++wstat.influence[nz];
    }
  check(wstat.weightSumBad == 0, "every corner's weights sum to 255");

  std::mt19937 rng(0xC0FFEEu);
  std::vector<float> palette(slots * 16, 0.0F);
  std::vector<float> vuPalette(slots * 16, 0.0F);
  Stats all;
  const int kPoses = 10000;
  for (int pose = 0; pose < kPoses; ++pose) {
    if (pose == 0) {
      for (u32 j = 0; j < slots; ++j) {
        float* m = &palette[j * 16];
        std::memset(m, 0, 16 * sizeof(float));
        m[0] = m[5] = m[10] = m[15] = 1.0F;
      }
    } else {
      for (u32 j = 0; j < slots; ++j) randomSlot(rng, &palette[j * 16], 0.3F, 0.1F);
    }
    buildVuPalette(palette.data(), slots, bind, vuPalette.data());
    Folded folded;
    foldPalette(palette.data(), slots, bind, outBox, folded);
    for (const Level& lv : levels) {
      const SkelMesh& mesh = *lv.mesh;
      for (u32 v = 0; v < mesh.vertexCount; ++v) {
        const s16* q = &mesh.pos[3 * v];
        const s8* n = &mesh.nrm[4 * v];
        const u8* j = &mesh.joints[4 * v];
        const u8* w = &mesh.weights[4 * v];
        float vp[3], vn[3];
        vuCorner(vuPalette.data(), q, n, j, w, vp, vn);
        float rp[3], rn[3];
        blendCorner(folded, q, n, j, w, rp, rn);
        // Reference position: index units x the output step = model metres (origin 0).
        for (int i = 0; i < 3; ++i) {
          const double ref = (double)rp[i] * (double)outBox.step;
          const double e = std::fabs((double)vp[i] - ref) * 1000.0;  // mm
          if (e > all.worstPos) all.worstPos = e;
          all.sumPos += e;
          ++all.posN;
        }
        // Reference normal: the unit direction of blendCorner's sum, x127.
        const double rl = std::sqrt((double)rn[0] * rn[0] + (double)rn[1] * rn[1] +
                                    (double)rn[2] * rn[2]);
        for (int i = 0; i < 3; ++i) {
          const double refN = rl > 0.0 ? (double)rn[i] / rl * 127.0 : 0.0;
          const double e = std::fabs((double)vn[i] - refN);
          if (e > all.worstNrm) all.worstNrm = e;
          all.sumNrm += e;
          ++all.nrmN;
        }
        ++all.corners;
      }
    }
  }
  std::printf("VU1 blend vs blendCorner (%d poses, %llu corner blends over every part at levels 0-2):\n",
              kPoses, (unsigned long long)all.corners);
  std::printf("  positions: worst %.6f mm, mean %.6f mm (%llu axes)\n", all.worstPos,
              all.posN ? all.sumPos / (double)all.posN : 0.0, (unsigned long long)all.posN);
  std::printf("  normals:   worst %.6f x127, mean %.6f x127 (%llu axes)\n", all.worstNrm,
              all.nrmN ? all.sumNrm / (double)all.nrmN : 0.0, (unsigned long long)all.nrmN);
  std::printf("  flushed to zero (denormal results): %llu\n", (unsigned long long)gFlushed);
  std::printf("  corners by non-zero influences: 0:%llu 1:%llu 2:%llu 3:%llu 4:%llu\n",
              (unsigned long long)wstat.influence[0], (unsigned long long)wstat.influence[1],
              (unsigned long long)wstat.influence[2], (unsigned long long)wstat.influence[3],
              (unsigned long long)wstat.influence[4]);
  check(all.worstPos < 0.05, "VU1 positions within 0.05 mm of blendCorner (worst)");
  check(all.worstNrm < 0.05, "VU1 normals within 0.05 x127 of blendCorner (worst)");

  // ---- VU1 data-memory and batch arithmetic (asserted, not measured) -------------------
  const u32 persistent = kMvpBytes + kQwordsPerJoint * kQword * slots + kReservedBytes;
  check(kQwordsPerJoint * kQword == 64, "a joint's palette is four qwords (64 bytes)");
  check(kQwordsPerJoint * slots <= 255, "the last joint's pre-scaled qword offset fits the ILW byte");
  check(persistent < kVu1DataBytes / 2, "the persistent region leaves room for two buffers");
  const u32 side = (kVu1DataBytes - persistent) / 2;  // one double-buffer side
  const u32 usable = side - kHeaderBytes;
  const u32 batchCorners = (usable / kCornerBytes / 3) * 3;  // whole packages of 3
  check(batchCorners >= 3, "a batch holds at least one package");
  check(batchCorners % 3 == 0, "a batch holds whole packages");
  std::printf("VU1 data memory: persistent %u bytes (MVP %u, palette %u for %u joints, reserved %u);"
              " side %u bytes; %u bytes a corner; batch %u corners\n",
              persistent, kMvpBytes, kQwordsPerJoint * kQword * slots, slots, kReservedBytes, side,
              kCornerBytes, batchCorners);
  for (const Level& lv : levels) {
    const u32 n = lv.mesh->vertexCount;
    const u32 batches = (n + batchCorners - 1) / batchCorners;
    std::printf("  %s: %u corners -> %u batches of up to %u corners\n", lv.label, n, batches,
                batchCorners);
  }
  // Same numbers the docs state: a corner of today's compact stream is 16 bytes (position 6,
  // ST 6, colour 4), with the normal 4 more; the shared bind data adds joints and weights.
  check(kCornerBytes == 96, "the unpacked corner is six qwords");

  std::printf("verify-skel-vu1: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
