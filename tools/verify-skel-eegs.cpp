// Host verifier for skinning straight to the GS, the EE half (renderer/3d/mesh/dynamic/
// skel_eegs.hpp): the pose pass's unique vertices mapped to the GS in fixed point.
//
// For every part and LOD of the real combatent model (argv[1], a .tskl), over 2000 random poses,
// each seen from a random view (a perspective projection as the engine's: the screen edge at
// about 0.1 of w, depth -1..1 over 0.1..500 m, GS x/y 12.4 around 2048, a 24-bit depth range):
//  (a) the unique vertices come from SkelSkin::skinIntoUniques (the real pose pass);
//  (b) each unique's GS position (1/16 px), depth (the 24-bit range), Q (= 1/w) and fog F
//      against a double-precision reference chain from the bind data (the palette applied in
//      double, then the model-view-projection, divide, screen scale - the arithmetic of the VU1
//      program without its float rounding); the error is taken against the reference
//      TRUNCATED as the VU1's ftoi4 truncates, so what is left is the fixed-point path's own;
//  (c) the box classification: a part wholly Inside never has a unique outside the GS window,
//      a depth outside the range or a non-positive w; a part classified Outside has every
//      unique outside the view's guard band or depth range (a culled part really is out);
//  (d) the lighting of the uniques' normals against the VU1 program's float arithmetic (the
//      light matrix, the three directions, clamp to [floor, 1], the three colours and the
//      ambient, truncated to 8 bits), with random lights, in both the plain and the signed
//      SH forms: the error in colour levels.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel does this):
//   build/verify-skel-eegs res/models/combatent__ovrb22b.tskl

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
#include "renderer/3d/mesh/dynamic/skel_eegs.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

namespace {

using namespace Tyra;
using namespace Tyra::SkelSkin;
using namespace Tyra::SkelSkin::EeGs;

int checks = 0;
int failures = 0;

void check(bool ok, const char* what) {
  ++checks;
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
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

// A random rotation as a 3x3 (rows), from a quaternion.
void randomRotation(std::mt19937& rng, double r[3][3]) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  double q[4] = {gauss(rng), gauss(rng), gauss(rng), gauss(rng)};
  const double len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (double& c : q) c /= len;
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double m[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                          {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                          {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  std::memcpy(r, m, sizeof(m));
}

struct Stat {
  double worst = 0.0, sum = 0.0;
  u64 n = 0;
  void add(double e) {
    const double a = std::fabs(e);
    if (a > worst) worst = a;
    sum += a;
    ++n;
  }
  double mean() const { return n ? sum / (double)n : 0.0; }
};

// The double-precision reference of a unique: its model position (metres) and the normal the
// blend should hold (unit, x127 not applied), from the bind data.
void referenceUnique(const float* palette, const Compact::Box& bind, const s16* q, const s8* n,
                     const u8* joints, const u8* weights, double pos[3]) {
  (void)n;
  double sum = 0.0;
  for (int k = 0; k < 4; ++k) sum += weights[k];
  double p[3];
  for (int i = 0; i < 3; ++i) p[i] = (double)bind.origin[i] + (double)bind.step * q[i];
  double pp[3] = {0.0, 0.0, 0.0};
  for (int k = 0; k < 4; ++k) {
    const double wk = sum > 0.0 ? weights[k] / sum : 0.0;
    const float* m = palette + 16 * joints[k];
    for (int r = 0; r < 3; ++r) {
      const double a = (double)m[0 * 4 + r] * p[0] + (double)m[1 * 4 + r] * p[1] +
                       (double)m[2 * 4 + r] * p[2] + (double)m[3 * 4 + r];
      pp[r] += wk * a;
    }
  }
  for (int i = 0; i < 3; ++i) pos[i] = pp[i];
}

struct Level {
  const SkelMesh* mesh;
  Batch::Part batch;
  std::vector<u32> skinSource;
  SkelSkinExtent extent;
  std::vector<float> st;  // two floats a corner
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: verify-skel-eegs <model.tskl>\n");
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
    std::printf("verify-skel-eegs: %d checks, %d failed\n", checks, failures);
    return 1;
  }
  const Compact::Box bind = model.box;
  const Compact::Box outBox = skinOutputBox(model);
  const u32 slots = (u32)model.palette.size();

  std::vector<Level> levels;
  for (const SkelPart& part : model.parts) {
    std::vector<const SkelMesh*> meshes;
    meshes.push_back(&part);
    for (const SkelLod& lod : part.lods) meshes.push_back(&lod);
    for (const SkelMesh* mesh : meshes) {
      Level lv;
      lv.mesh = mesh;
      lv.skinSource.resize(mesh->vertexCount);
      std::map<std::array<int, 15>, u32> first;
      for (u32 v = 0; v < mesh->vertexCount; ++v) {
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
      bindExtent(mesh->pos.data(), mesh->joints.data(), mesh->weights.data(), mesh->vertexCount,
                 lv.extent);
      lv.st.assign((size_t)mesh->vertexCount * 2, 0.0F);
      levels.push_back(std::move(lv));
    }
  }

  std::mt19937 rng(0xEE65u);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  std::vector<float> pal(slots * 16, 0.0F);
  Stat posErr, depthErr, qErr, fogErr, lightErr, depthMm, textelErr;
  Stat xRaw;
  // Per band of the vertex's w (metres): < 0.5, 0.5 - 2, 2 - 10, >= 10.
  Stat bandPos[4], bandDepthMm[4], bandDepthZ[4];
  double worstPos = 0.0, worstPosD = 0.0, worstPosW = 0.0;
  u64 uniquesCompared = 0, partsInside = 0, partsCrossing = 0, partsOutside = 0, projectFails = 0,
      insideBad = 0, outsideBad = 0, clampedPoses = 0, lightCases = 0;
  const int kPoses = 2000;
  for (int pose = 0; pose < kPoses; ++pose) {
    for (u32 j = 0; j < slots; ++j) randomSlot(rng, &pal[j * 16], 0.3F, 0.1F);
    Folded folded;
    foldPalette(pal.data(), slots, bind, outBox, folded);
    for (Level& lv : levels) {
      const SkelMesh& mesh = *lv.mesh;
      std::vector<s16> pos;
      std::vector<s8> nrm;
      Result res;
      const u32 uniqueCount = [&] {
        u32 n = 0;
        for (u32 v = 0; v < mesh.vertexCount; ++v)
          if (lv.skinSource[v] == v) ++n;
        return n;
      }();
      pos.assign((size_t)uniqueCount * 4, 0);
      nrm.assign((size_t)uniqueCount * 4, 0);
      res.pos = pos.data();
      res.nrm = nrm.data();
      res.capacity = uniqueCount;
      const SkelSkinPart part{mesh.pos.data(),    mesh.nrm.data(),    mesh.joints.data(),
                              mesh.weights.data(), lv.skinSource.data(), mesh.vertexCount,
                              &lv.extent,          nullptr,            nullptr,
                              &lv.batch};
      const u32 r = skinIntoUniques(folded, pal.data(), slots, bind, part, res);
      if (r == kSkinRefused) {
        check(false, "the pose pass refuses a part");
        continue;
      }
      if (res.clamped) ++clampedPoses;

      // A random view: the body at distance d in front, rotated at random, off-axis within
      // the screen. clip = P x (R model + t); P maps the view-space z to w.
      const double d = std::exp(std::log(1.5) + uni(rng) * (std::log(80.0) - std::log(1.5)));
      double R[3][3];
      randomRotation(rng, R);
      // One view in thirty is thrown wide of the screen, so the culled class occurs.
      const double wide = (pose % 30 == 7) ? 200.0 : 1.0;
      const double t[3] = {(uni(rng) * 2 - 1) * 0.06 * d * wide, (uni(rng) * 2 - 1) * 0.08 * d, d};
      const double px = 0.1, py = 0.14, nearP = 0.1, farP = 500.0;
      const double a = (farP + nearP) / (farP - nearP), b = -2 * farP * nearP / (farP - nearP);
      double M[4][4];  // row-major clip = M x (model, 1)
      for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 4; ++c) M[i][c] = c < 3 ? R[i][c] : t[i];
      double C[4][4];
      for (int c = 0; c < 4; ++c) {
        C[0][c] = px * M[0][c];
        C[1][c] = py * M[1][c];
        C[2][c] = a * M[2][c] + (c == 3 ? b : 0.0);
        C[3][c] = M[2][c];
      }
      float mvp[16];  // column-major
      for (int c = 0; c < 4; ++c)
        for (int rr = 0; rr < 4; ++rr) mvp[4 * c + rr] = (float)C[rr][c];
      // The reference takes the same float matrix the engine hands the VU1 and the EE.
      for (int c = 0; c < 4; ++c)
        for (int rr = 0; rr < 4; ++rr) C[rr][c] = (double)mvp[4 * c + rr];

      Screen scr;
      const bool fog = (pose & 1) != 0;
      scr.fog = fog;
      scr.fogScale = -255.0F / 55.0F;
      scr.fogOffset = 255.0F * 60.0F / 55.0F;
      const Box cls = classifyBox(mvp, res.step, res.oi, res.lo, res.hi, 0.9F);

      // The reference chain per unique.
      std::vector<double> sx(uniqueCount), sy(uniqueCount), sz(uniqueCount), wv(uniqueCount);
      std::vector<u8> ok(uniqueCount, 1);
      u32 uq = 0;
      bool anyOutsideView = false, anyInView = false;
      for (u32 v = 0; v < mesh.vertexCount; ++v) {
        if (lv.skinSource[v] != v) continue;
        double p[3];
        referenceUnique(pal.data(), bind, mesh.pos.data() + 3 * v, mesh.nrm.data() + 4 * v,
                        mesh.joints.data() + 4 * v, mesh.weights.data() + 4 * v, p);
        double clip[4];
        for (int rr = 0; rr < 4; ++rr)
          clip[rr] = C[rr][0] * p[0] + C[rr][1] * p[1] + C[rr][2] * p[2] + C[rr][3];
        wv[uq] = clip[3];
        if (clip[3] > 0.0) {
          sx[uq] = 16.0 * (clip[0] / clip[3] * 2048.0 + 2048.0);
          sy[uq] = 16.0 * (clip[1] / clip[3] * 2048.0 + 2048.0);
          sz[uq] = clip[2] / clip[3] * scr.sz + scr.sz;
        }
        const bool inGuard = clip[3] > 0.0 && std::fabs(clip[0]) <= 0.9 * clip[3] &&
                             std::fabs(clip[1]) <= 0.9 * clip[3] && clip[2] <= clip[3] &&
                             clip[2] >= -clip[3];
        if (!inGuard) anyOutsideView = true; else anyInView = true;
        ++uq;
      }
      if (cls == Box::Outside) {
        ++partsOutside;
        // Culled: no unique may be inside the guard band and depth range.
        if (anyInView) ++outsideBad;
        continue;
      }
      if (cls == Box::Crossing) {
        ++partsCrossing;
        continue;
      }
      ++partsInside;
      if (anyOutsideView) ++insideBad;
      Xform xf;
      if (!buildXform(mvp, res.step, res.oi, res.lo, res.hi, scr, xf)) {
        check(false, "buildXform failed on an Inside part");
        continue;
      }

      // Lights for this view.
      float lm[12], dirs[12], colors[16];
      {
        double Rl[3][3];
        randomRotation(rng, Rl);
        for (int c = 0; c < 3; ++c)
          for (int rr = 0; rr < 3; ++rr) lm[4 * c + rr] = (float)Rl[rr][c];
        for (int c = 0; c < 3; ++c) lm[4 * c + 3] = 0.0F;
        for (int c = 0; c < 3; ++c) {
          double v[3] = {uni(rng) * 2 - 1, uni(rng) * 2 - 1, uni(rng) * 2 - 1};
          const double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) + 1e-9;
          for (int i = 0; i < 3; ++i) dirs[4 * c + i] = (float)(v[i] / l);
          dirs[4 * c + 3] = 0.0F;
        }
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) colors[4 * i + j] = (float)(uni(rng) * 160.0);
          colors[4 * i + 3] = 0.0F;
        }
        for (int j = 0; j < 3; ++j) colors[12 + j] = (float)(uni(rng) * 100.0);
        colors[15] = 0.0F;
      }
      const bool signedSH = (pose % 3) == 0;
      Light lt;
      buildLight(lm, dirs, colors, signedSH, lt);
      ++lightCases;

      uq = 0;
      std::uniform_real_distribution<double> stDist(-4.0, 4.0);
      for (u32 v = 0; v < mesh.vertexCount; ++v) {
        if (lv.skinSource[v] != v) continue;
        Out o;
        if (!project(xf, res.pos + 4 * (size_t)uq, o)) {
          ++projectFails;
          ++uq;
          continue;
        }
        const double ox = (double)(o.xy & 0xFFFFu), oy = (double)(o.xy >> 16);
        const double oz = (double)(o.zf & 0xFFFFFFu);
        const double of = (double)(o.zf >> 24);
        // Against the truncated reference (the VU1's ftoi4).
        {
          const double ex = std::fabs(ox - std::floor(sx[uq])), ey = std::fabs(oy - std::floor(sy[uq]));
          const double em = ex > ey ? ex : ey;
          if (em > worstPos) {
            worstPos = em;
            worstPosD = d;
            worstPosW = wv[uq];
          }
        }
        const int band = wv[uq] < 0.5 ? 0 : wv[uq] < 2.0 ? 1 : wv[uq] < 10.0 ? 2 : 3;
        bandPos[band].add(ox - std::floor(sx[uq]));
        bandPos[band].add(oy - std::floor(sy[uq]));
        posErr.add(ox - std::floor(sx[uq]));
        posErr.add(oy - std::floor(sy[uq]));
        xRaw.add(ox - sx[uq]);
        xRaw.add(oy - sy[uq]);
        const double dz = oz - std::floor(sz[uq]);
        depthErr.add(dz);
        // Equivalent view depth: dZ/dz = sz x 2 f n / ((f - n) z^2).
        const double dZdz = scr.sz * 2.0 * farP * nearP / ((farP - nearP) * wv[uq] * wv[uq]);
        depthMm.add(dz / dZdz * 1000.0);
        bandDepthMm[band].add(dz / dZdz * 1000.0);
        bandDepthZ[band].add(dz);
        qErr.add((double)o.q * wv[uq] - 1.0);
        {
          // A corner's S = s x Q: texel error for |s| <= 4 on a 256-texel texture.
          const double s = stDist(rng);
          const double ours = (double)(float)(s * (double)o.q);
          const double truth = s / wv[uq];
          textelErr.add((ours - truth) * 256.0);
        }
        if (fog) {
          double ff = wv[uq] * (double)scr.fogScale + (double)scr.fogOffset;
          if (ff < 0.0) ff = 0.0;
          if (ff > 255.0) ff = 255.0;
          fogErr.add(of - std::floor(ff));
        }
        // The light.
        {
          const s8* n = res.nrm + 4 * (size_t)uq;
          double nw[3];
          for (int cp = 0; cp < 3; ++cp) {
            nw[cp] = 0.0;
            for (int c = 0; c < 3; ++c) nw[cp] += (double)lm[4 * c + cp] * (n[c] / 127.0);
          }
          double outv[3];
          for (int i = 0; i < 3; ++i) {
            double e = 0.0;
            for (int cp = 0; cp < 3; ++cp) e += (double)dirs[4 * cp + i] * nw[cp];
            const double fl = signedSH ? -1.0 : 0.0;
            if (e > 1.0) e = 1.0;
            if (e < fl) e = fl;
            outv[i] = e;
          }
          const u32 rgba = light(lt, n);
          for (int j = 0; j < 3; ++j) {
            double c = (double)colors[12 + j];
            for (int i = 0; i < 3; ++i) c += (double)colors[4 * i + j] * outv[i];
            if (c > 255.0) c = 255.0;
            if (c < 0.0) c = 0.0;
            lightErr.add((double)((rgba >> (8 * j)) & 0xFF) - std::floor(c));
          }
          check((rgba >> 24) == 128, "alpha is 128");
        }
        ++uniquesCompared;
        ++uq;
      }
    }
  }
  std::printf("views: %llu parts Inside, %llu Crossing (fall back), %llu Outside (culled); "
              "%llu uniques compared over %d poses of %zu part levels\n",
              (unsigned long long)partsInside, (unsigned long long)partsCrossing,
              (unsigned long long)partsOutside, (unsigned long long)uniquesCompared, kPoses,
              levels.size());
  std::printf("screen x/y vs the truncated reference: worst %.3f mean %.4f (1/16 px, %llu axes); "
              "vs the untruncated: worst %.3f mean %.4f\n",
              posErr.worst, posErr.mean(), (unsigned long long)posErr.n, xRaw.worst, xRaw.mean());
  std::printf("  worst position error %.1f/16 px at part distance %.2f m, vertex w %.3f m\n",
              worstPos, worstPosD, worstPosW);
  {
    const char* names[4] = {"w < 0.5 m", "0.5 - 2 m", "2 - 10 m", ">= 10 m"};
    for (int b = 0; b < 4; ++b)
      std::printf("  %-10s: position worst %.2f mean %.4f (1/16 px, %llu axes); depth worst %.0f "
                  "mean %.2f units = %.3f mm worst\n",
                  names[b], bandPos[b].worst, bandPos[b].mean(), (unsigned long long)bandPos[b].n,
                  bandDepthZ[b].worst, bandDepthZ[b].mean(), bandDepthMm[b].worst);
  }
  std::printf("depth: worst %.2f mean %.3f (24-bit range units); as view depth worst %.4f mm "
              "mean %.5f mm\n",
              depthErr.worst, depthErr.mean(), depthMm.worst, depthMm.mean());
  std::printf("Q = 1/w: relative error worst %.3g mean %.3g; S/T for |s| <= 4 on 256 texels: "
              "worst %.4f mean %.5f texel\n",
              qErr.worst, qErr.mean(), textelErr.worst, textelErr.mean());
  std::printf("fog F: worst %.2f mean %.3f (levels of 255, %llu values)\n", fogErr.worst,
              fogErr.mean(), (unsigned long long)fogErr.n);
  std::printf("light: worst %.2f mean %.3f (levels of 255, %llu channels, %llu light setups)\n",
              lightErr.worst, lightErr.mean(), (unsigned long long)lightErr.n,
              (unsigned long long)lightCases);

  check(partsInside > 0 && partsCrossing > 0 && partsOutside > 0, "all three classes occur");
  check(projectFails == 0, "project never fails on an Inside part");
  check(insideBad == 0, "an Inside part has no unique outside the guard band or depth range");
  check(outsideBad == 0, "an Outside part has no unique inside the guard band");
  check(bandPos[1].worst <= 3.0 && bandPos[2].worst <= 2.0 && bandPos[3].worst <= 2.0,
        "screen position within 3/16 px from 0.5 m, 2/16 px from 2 m of the truncated reference");
  check(bandDepthMm[2].worst <= 10.0 && bandDepthMm[3].worst <= 60.0,
        "depth within 10 mm of view depth at 2 - 10 m, 60 mm beyond (below the 24-bit range's own step there)");
  check(qErr.worst <= 1e-3, "Q within 1e-3 relative");
  check(lightErr.worst <= 2.0, "light within 2 colour levels");
  check(fogErr.worst <= 2.0, "fog within 2 levels");
  std::printf("verify-skel-eegs: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
