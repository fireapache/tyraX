// Host verifier for the unique-vertex skin (renderer/3d/mesh/dynamic/skel_unique.hpp).
// The unique corners of every part and LOD of the soldier model are blended, boxed and
// quantized once, then scattered to every destination corner; the result is checked against
// the double-precision reference of verify-skel-fixed.cpp (the same definition, in the
// same output box).
//
// Checks, over 10 000 random poses (every level of every part, identity and random palettes
// alike), plus clamp poses whose joints spread past the box:
//  - positions within 2 output steps and normals within 2 x127 units of the reference;
//  - a corner whose reference is outside the box clamps to +-32767 with the reference's sign,
//    is counted (one per destination), and never wraps;
//  - a repeat's bytes equal its source's bytes;
//  - the part box (min and max of the written positions) equals the min and max over the
//    unique corners, and every corner is covered by the scatter.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel does this):
//   build/verify-skel-unique <model>.tskl

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "renderer/3d/mesh/dynamic/skel_unique.hpp"

namespace {

using namespace Tyra;
using SkelSkin::SkinScatter;
using SkelSkin::buildScatter;

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

struct Truth {
  double pos[3];
  double nrm[3];
};

// The double-precision reference of one corner in the output box `out` (as
// verify-skel-fixed.cpp states it): output index in steps, unit normal x127.
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

// A column-major palette slot: rigid rotation, scale near 1, translation up to maxShift.
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

// The first identical corner of each corner (as repackBind keys them: exact bind bytes).
std::vector<u32> firstIdentical(const SkelMesh& mesh) {
  std::vector<u32> src(mesh.vertexCount);
  std::unordered_map<std::string, u32> first;
  first.reserve(mesh.vertexCount);
  for (u32 v = 0; v < mesh.vertexCount; ++v) {
    char key[32];
    std::memcpy(key, &mesh.pos[3 * v], 6);
    std::memcpy(key + 6, &mesh.nrm[4 * v], 3);
    std::memcpy(key + 9, &mesh.joints[4 * v], 4);
    std::memcpy(key + 13, &mesh.weights[4 * v], 4);
    const auto ins = first.emplace(std::string(key, 17), v);
    src[v] = ins.first->second;
  }
  return src;
}

struct Stats {
  double worstPos = 0.0, sumPos = 0.0;
  double worstNrm = 0.0, sumNrm = 0.0;
  u64 posN = 0, nrmN = 0;
  u64 corners = 0, uniques = 0;
  u64 clampedDst = 0;   // destinations clamped (the scatter's count)
  u64 clampedRef = 0;   // corners whose reference is out of the box
  u64 signWrong = 0;    // clamped corners whose sign differs from the reference
  u64 repeatWrong = 0;  // repeats whose bytes differ from their source
  u64 boxWrong = 0;     // part boxes not equal to the unique extent
  u64 uncovered = 0;    // corners the scatter did not write
};

void meshPoses(const SkelMesh& mesh, const float* palette, const Compact::Box& bind,
               float outStep, const std::vector<u32>& src,
               const SkelSkin::Unique::UniqueBind& ub, const SkinScatter& map, u32 size,
               std::vector<float>& posU, std::vector<float>& nrmU, std::vector<s16>& qPos,
               std::vector<s8>& qNrm, std::vector<u8>& clamp, Stats& st) {
  const u32 n = mesh.vertexCount;
  posU.assign((size_t)ub.count * 4, 0.0F);
  nrmU.assign((size_t)ub.count * 4, 0.0F);
  qPos.assign((size_t)ub.count * 4, 0);
  qNrm.assign((size_t)ub.count * 4, 0);
  clamp.assign(ub.count, 0);
  SkelSkin::Unique::blend(ub, palette, posU.data(), nrmU.data());
  s32 oi[3];
  SkelSkin::Unique::poseOrigin(posU.data(), ub.count, bind.step, oi);
  Compact::Box out;
  out.step = outStep;
  for (int i = 0; i < 3; ++i) out.origin[i] = (float)oi[i] * out.step;
  SkelSkin::Unique::quantize(posU.data(), nrmU.data(), ub.count, out.step, oi, qPos.data(),
                             qNrm.data(), clamp.data());
  std::vector<s16> outPos((size_t)n * 4, 0);
  std::vector<s8> outNrm((size_t)n * 4, 0);
  st.clampedDst += SkelSkin::Unique::scatterFlat(map, size, ub, qPos.data(), qNrm.data(),
                                                 clamp.data(), n, outPos.data(), outNrm.data());
  st.uniques += ub.count;
  st.corners += n;
  for (u32 v = 0; v < n; ++v) {
    const Truth t = referenceCorner(palette, bind, out, &mesh.pos[3 * v], &mesh.nrm[4 * v],
                                    &mesh.joints[4 * v], &mesh.weights[4 * v]);
    for (int i = 0; i < 3; ++i) {
      const double w = outPos[(size_t)v * 4 + i];
      if (std::fabs(t.pos[i]) > 32767.5) {
        ++st.clampedRef;
        if (std::fabs(w) != 32767.0 || (w > 0) != (t.pos[i] > 0)) ++st.signWrong;
        continue;
      }
      const double e = std::fabs(w - t.pos[i]);
      if (e > st.worstPos) st.worstPos = e;
      st.sumPos += e;
      ++st.posN;
    }
    for (int i = 0; i < 3; ++i) {
      const double e = std::fabs((double)outNrm[(size_t)v * 4 + i] - t.nrm[i]);
      if (e > st.worstNrm) st.worstNrm = e;
      st.sumNrm += e;
      ++st.nrmN;
    }
    if (src[v] != v &&
        std::memcmp(&outPos[(size_t)v * 4], &outPos[(size_t)src[v] * 4], 6) != 0)
      ++st.repeatWrong;
  }
  // The part box: min and max of the written positions, against the unique extent.
  s32 bmn[3] = {32767, 32767, 32767}, bmx[3] = {-32768, -32768, -32768};
  s32 umn[3] = {32767, 32767, 32767}, umx[3] = {-32768, -32768, -32768};
  for (u32 v = 0; v < n; ++v)
    for (int i = 0; i < 3; ++i) {
      const s32 w = outPos[(size_t)v * 4 + i];
      if (w < bmn[i]) bmn[i] = w;
      if (w > bmx[i]) bmx[i] = w;
    }
  for (u32 u = 0; u < ub.count; ++u)
    for (int i = 0; i < 3; ++i) {
      const s32 w = qPos[(size_t)u * 4 + i];
      if (w < umn[i]) umn[i] = w;
      if (w > umx[i]) umx[i] = w;
    }
  for (int i = 0; i < 3; ++i)
    if (n > 0 && (bmn[i] != umn[i] || bmx[i] != umx[i])) ++st.boxWrong;
  // Coverage: every corner is a destination of exactly one unique.
  std::vector<u8> seen(n, 0);
  for (u32 u = 0; u < ub.count; ++u) {
    const u32 s0 = ub.corner[u];
    for (u32 d = map.dstOff[s0]; d < map.dstOff[s0 + 1]; ++d) {
      const u32 dv = map.dst[d];
      const u32 v = (dv >> 16) * size + (dv & 0xFFFFu);
      if (v < n) seen[v] = 1;
    }
  }
  for (u32 v = 0; v < n; ++v)
    if (!seen[v]) ++st.uncovered;
}

void printStats(const char* label, const Stats& s) {
  std::printf("%s: corners %llu, uniques %llu; positions worst %.3f steps mean %.4f (%llu axes); "
              "normals worst %.3f mean %.4f (x127 units); clamped destinations %llu (reference "
              "out of box %llu, sign wrong %llu); repeats wrong %llu, box wrong %llu, uncovered "
              "%llu\n",
              label, (unsigned long long)s.corners, (unsigned long long)s.uniques, s.worstPos,
              s.posN ? s.sumPos / (double)s.posN : 0.0, (unsigned long long)s.posN, s.worstNrm,
              s.nrmN ? s.sumNrm / (double)s.nrmN : 0.0, (unsigned long long)s.clampedDst,
              (unsigned long long)s.clampedRef, (unsigned long long)s.signWrong,
              (unsigned long long)s.repeatWrong, (unsigned long long)s.boxWrong,
              (unsigned long long)s.uncovered);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: verify-skel-unique <model.tskl>\n");
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
    std::printf("verify-skel-unique: %d checks, %d failed\n", checks, failures);
    return 1;
  }
  const Compact::Box bind = model.box;
  const Compact::Box outBox = skinOutputBox(model);
  const u32 slots = (u32)model.palette.size();
  check(slots <= SkelSkin::kMaxSlots, "the palette fits the skin");

  // One unique-bind and scatter map per part level, built once (as the bind cache would).
  struct Level {
    const SkelMesh* mesh;
    std::vector<u32> src;
    SkelSkin::Unique::UniqueBind ub;
    SkinScatter map;
    const char* name;
    char label[48];
  };
  const u32 kPackage = 3;  // the stream package size of the scatter (flat index decode)
  std::vector<Level> levels;
  for (size_t pi = 0; pi < model.parts.size(); ++pi) {
    const size_t nl = 1 + model.parts[pi].lods.size();
    for (size_t l = 0; l < nl && l <= 2; ++l) {
      Level lv;
      lv.mesh = l == 0 ? static_cast<const SkelMesh*>(&model.parts[pi])
                       : static_cast<const SkelMesh*>(&model.parts[pi].lods[l - 1]);
      lv.src = firstIdentical(*lv.mesh);
      SkelSkin::Unique::buildUnique(lv.mesh->pos.data(), lv.mesh->nrm.data(),
                                    lv.mesh->joints.data(), lv.mesh->weights.data(),
                                    lv.src.data(), lv.mesh->vertexCount, bind, lv.ub);
      buildScatter(lv.src.data(), lv.mesh->vertexCount, kPackage, lv.map);
      std::snprintf(lv.label, sizeof(lv.label), "part %zu level %zu", pi, l);
      levels.push_back(std::move(lv));
    }
  }
  check(!levels.empty(), "the model has parts");

  std::mt19937 rng(0xC0FFEEu);
  std::vector<float> palette(slots * 16, 0.0F);
  Stats all;
  std::vector<float> posU, nrmU;
  std::vector<s16> qPos;
  std::vector<s8> qNrm;
  std::vector<u8> clamp;
  // 10 000 poses in the model's box: identity, then random palettes. The bind box and
  // output box are the model's own (the output box is scaled by the pose's origin).
  for (int pose = 0; pose < 10000; ++pose) {
    if (pose == 0) {
      for (u32 j = 0; j < slots; ++j) {
        float* m = &palette[j * 16];
        std::memset(m, 0, 16 * sizeof(float));
        m[0] = m[5] = m[10] = m[15] = 1.0F;
      }
    } else {
      for (u32 j = 0; j < slots; ++j) randomSlot(rng, &palette[j * 16], 0.3F, 0.1F);
    }
    for (Level& lv : levels) {
      Stats st;
      meshPoses(*lv.mesh, palette.data(), bind, outBox.step, lv.src, lv.ub, lv.map, kPackage, posU,
                nrmU, qPos, qNrm, clamp, st);
      all.worstPos = st.worstPos > all.worstPos ? st.worstPos : all.worstPos;
      all.sumPos += st.sumPos;
      all.posN += st.posN;
      all.worstNrm = st.worstNrm > all.worstNrm ? st.worstNrm : all.worstNrm;
      all.sumNrm += st.sumNrm;
      all.nrmN += st.nrmN;
      all.corners += st.corners;
      all.uniques += st.uniques;
      all.clampedDst += st.clampedDst;
      all.clampedRef += st.clampedRef;
      all.signWrong += st.signWrong;
      all.repeatWrong += st.repeatWrong;
      all.boxWrong += st.boxWrong;
      all.uncovered += st.uncovered;
    }
  }
  printStats("random poses (every part and LOD, 10 000 poses)", all);
  check(all.worstPos <= 2.0, "unique positions within 2 output steps of the reference");
  check(all.worstNrm <= 2.0, "unique normals within 2 x127 units of the reference");
  check(all.signWrong == 0, "out-of-box corners clamp with the reference's sign");
  check(all.repeatWrong == 0, "repeats equal their source's bytes");
  check(all.boxWrong == 0, "part boxes equal the unique extent");
  check(all.uncovered == 0, "the scatter covers every corner");

  // Clamp poses: joints thrown up to 50 m apart, so the pose spans past the box.
  Stats clampAll;
  for (int pose = 0; pose < 200; ++pose) {
    for (u32 j = 0; j < slots; ++j) randomSlot(rng, &palette[j * 16], 50.0F, 0.1F);
    for (Level& lv : levels) {
      Stats st;
      meshPoses(*lv.mesh, palette.data(), bind, outBox.step, lv.src, lv.ub, lv.map, kPackage, posU,
                nrmU, qPos, qNrm, clamp, st);
      clampAll.clampedDst += st.clampedDst;
      clampAll.clampedRef += st.clampedRef;
      clampAll.signWrong += st.signWrong;
      clampAll.repeatWrong += st.repeatWrong;
      clampAll.uncovered += st.uncovered;
      clampAll.corners += st.corners;
    }
  }
  printStats("clamp poses (50 m joint spread, 200 poses)", clampAll);
  check(clampAll.clampedRef > 0, "the clamp poses reach out of the box");
  check(clampAll.signWrong == 0, "clamped corners keep the reference's sign and value");
  check(clampAll.repeatWrong == 0, "clamp poses: repeats equal their source's bytes");

  std::printf("verify-skel-unique: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
