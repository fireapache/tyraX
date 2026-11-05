// Host verifier for the influence limit (loaders/3d/tskl_loader/tskl_limit.hpp,
// TYRA_SKIN_MAX_INFLUENCES). The combatent model is skinned in double precision with its own
// four-influence weights (K=4) and with the weights limited to K = 3 and K = 2, over random
// blended poses of every part and LOD. The arm reports the deviation in millimetres and in
// degrees, the share of corners whose two largest weights carry 95 % or more, and the
// deviation in screen pixels at 2, 8, 16 and 30 m for a 640 x 448 screen with a 60 degree
// vertical field of view (focal length 388 px).
//
// Checks: K = 4 is the identity (byte-equal); a limited corner keeps at most K influences and
// its weights sum to 255. The deviations are reported, not gated: the acceptance is the visual
// judgement in docs/animated-models.md.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel does this):
//   build/verify-skel-influences res/models/combatent__ovrb22b.tskl

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "loaders/3d/tskl_loader/tskl_limit.hpp"
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

constexpr double kFocalPx = 224.0 / 0.57735026918962576;  // 448 / 2 over tan(30 deg)

// Double-precision skin of one corner: position in model metres, unit normal.
void skinCorner(const float* palette, const Compact::Box& bind, const s16* q, const s8* n,
                const u8* joints, const u8* weights, double pos[3], double nrm[3]) {
  double sum = 0.0;
  for (int k = 0; k < 4; ++k) sum += weights[k];
  double p[3];
  for (int i = 0; i < 3; ++i) p[i] = (double)bind.origin[i] + (double)bind.step * q[i];
  pos[0] = pos[1] = pos[2] = 0.0;
  nrm[0] = nrm[1] = nrm[2] = 0.0;
  const double nv[3] = {n[0] / 127.0, n[1] / 127.0, n[2] / 127.0};
  for (int k = 0; k < 4; ++k) {
    const double wk = sum > 0.0 ? weights[k] / sum : 0.0;
    const float* m = palette + 16 * joints[k];
    for (int r = 0; r < 3; ++r)
      pos[r] += wk * ((double)m[0 + r] * p[0] + (double)m[4 + r] * p[1] +
                      (double)m[8 + r] * p[2] + (double)m[12 + r]);
    for (int i = 0; i < 3; ++i)
      nrm[i] += wk * ((double)m[0 + i] * nv[0] + (double)m[4 + i] * nv[1] +
                      (double)m[8 + i] * nv[2]);
  }
  const double len = std::sqrt(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]);
  for (int i = 0; i < 3; ++i) nrm[i] = len > 0.0 ? nrm[i] / len : 0.0;
}

// A joint pose within a bounded rotation (up to maxAngle radians) and translation (up to
// maxShift metres): the spread of a clip frame, not an arbitrary rotation.
void boundedSlot(std::mt19937& rng, float* m, double maxAngle, double maxShift) {
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  double ax[3] = {gauss(rng), gauss(rng), gauss(rng)};
  const double al = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
  const double th = maxAngle * unit(rng);
  const double sn = std::sin(th / 2.0) / al;
  const double x = ax[0] * sn, y = ax[1] * sn, z = ax[2] * sn, w = std::cos(th / 2.0);
  const double r[3][3] = {
      {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
      {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
      {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  for (int c = 0; c < 3; ++c)
    for (int row = 0; row < 3; ++row) m[c * 4 + row] = (float)r[row][c];
  for (int row = 0; row < 3; ++row) m[12 + row] = (float)(maxShift * uni(rng));
  m[3] = m[7] = m[11] = 0.0F;
  m[15] = 1.0F;
}

void randomSlot(std::mt19937& rng, float* m) {
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
  const double s = 1.0 + 0.1 * uni(rng);
  for (int c = 0; c < 3; ++c)
    for (int row = 0; row < 3; ++row) m[c * 4 + row] = (float)(s * r[row][c]);
  for (int row = 0; row < 3; ++row) m[12 + row] = (float)(0.3 * uni(rng));
  m[3] = m[7] = m[11] = 0.0F;
  m[15] = 1.0F;
}

// Samples of one deviation kind: a running sum and worst, and a subsample for percentiles.
struct Stat {
  std::vector<float> v;
  double sum = 0.0, worst = 0.0;
  void add(double x) {
    sum += x;
    if (x > worst) worst = x;
  }
  double pct(double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[(size_t)std::floor(p * (v.size() - 1))];
  }
};

struct Worst {
  double mm = -1.0;
  u32 part = 0, level = 0, vertex = 0;
  u8 j[4] = {0, 0, 0, 0}, w[4] = {0, 0, 0, 0};
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: verify-skel-influences <model.tskl>\n");
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
    std::printf("verify-skel-influences: %d checks, %d failed\n", checks, failures);
    return 1;
  }
  const Compact::Box bind = model.box;
  const u32 slots = (u32)model.palette.size();

  // Every part and LOD level, each with its own corners.
  struct Mesh {
    const SkelMesh* m;
    u32 part, level;
  };
  std::vector<Mesh> meshes;
  for (u32 pi = 0; pi < model.parts.size(); ++pi) {
    meshes.push_back({&model.parts[pi], pi, 0});
    for (u32 l = 0; l < model.parts[pi].lods.size(); ++l)
      meshes.push_back({&model.parts[pi].lods[l], pi, l + 1});
  }

  // (a) K = 4 is the identity: the limiter leaves every corner byte-equal.
  {
    u64 corners = 0, changed = 0;
    for (const Mesh& me : meshes)
      for (u32 v = 0; v < me.m->vertexCount; ++v) {
        u8 j[4], w[4];
        std::memcpy(j, &me.m->joints[(size_t)v * 4], 4);
        std::memcpy(w, &me.m->weights[(size_t)v * 4], 4);
        limitCornerInfluences(j, w, 4);
        if (std::memcmp(j, &me.m->joints[(size_t)v * 4], 4) != 0 ||
            std::memcmp(w, &me.m->weights[(size_t)v * 4], 4) != 0)
          ++changed;
        ++corners;
      }
    std::printf("K=4: %llu corners, %llu changed by the limiter\n", (unsigned long long)corners,
                (unsigned long long)changed);
    check(changed == 0, "K=4 leaves every corner unchanged (byte-equal)");
  }

  // (c) the influence histogram of the bind data and the share of corners whose two largest
  // weights carry >= 95 % of the sum.
  {
    u64 total = 0, multi = 0, top2 = 0, hist[5] = {0, 0, 0, 0, 0};
    for (const Mesh& me : meshes)
      for (u32 v = 0; v < me.m->vertexCount; ++v) {
        const u8* w = &me.m->weights[(size_t)v * 4];
        u32 sum = 0, nz = 0, a = 0, b = 0;
        for (int k = 0; k < 4; ++k) {
          sum += w[k];
          if (w[k]) ++nz;
          if (w[k] > a) {
            b = a;
            a = w[k];
          } else if (w[k] > b) {
            b = w[k];
          }
        }
        ++total;
        hist[nz]++;
        if (nz >= 2) {
          ++multi;
          if (sum && (a + b) * 100 >= sum * 95) ++top2;
        }
      }
    std::printf("influences of %llu corners: 1 %.1f %%, 2 %.1f %%, 3 %.1f %%, 4 %.1f %%; corners with two or more influences whose top two carry >= 95 %%: %.1f %%\n",
                (unsigned long long)total, 100.0 * hist[1] / total, 100.0 * hist[2] / total,
                100.0 * hist[3] / total, 100.0 * hist[4] / total,
                multi ? 100.0 * top2 / multi : 0.0);
  }

  // The limiter's invariants: at most K influences, the weights sum to 255 (corners with
  // more than K influences; the others are unchanged).
  {
    u64 bad = 0, limited = 0;
    for (u32 k = 2; k <= 3; ++k)
      for (const Mesh& me : meshes)
        for (u32 v = 0; v < me.m->vertexCount; ++v) {
          u8 j[4], w[4];
          std::memcpy(j, &me.m->joints[(size_t)v * 4], 4);
          std::memcpy(w, &me.m->weights[(size_t)v * 4], 4);
          u32 nz0 = 0;
          for (int s = 0; s < 4; ++s) nz0 += w[s] ? 1U : 0U;
          limitCornerInfluences(j, w, k);
          u32 nz = 0, sum = 0;
          for (int s = 0; s < 4; ++s) {
            nz += w[s] ? 1U : 0U;
            sum += w[s];
          }
          if (nz0 > k) {
            ++limited;
            if (nz > k || sum != 255) ++bad;
          }
        }
    std::printf("limiter invariants (K=2 and K=3): %llu corners limited, %llu broken\n",
                (unsigned long long)limited, (unsigned long long)bad);
    check(bad == 0, "a limited corner keeps K influences and sums to 255");
  }

  // (b) and (d): the deviation of K = 3 and K = 2 from K = 4 over random poses.
  for (int set = 0; set < 2; ++set) {
  std::mt19937 rng(0x2F1Eu + (u32)set);
  std::vector<float> palette(slots * 16, 0.0F);
  const u32 ks[2] = {3, 2};
  Stat mm[2], deg[2];
  Worst worst[2][10];
  u64 over1[2] = {0, 0}, over3[2] = {0, 0}, over10[2] = {0, 0};
  u64 corners = 0;
  const int kPoses = set == 0 ? 2000 : 2000;
  std::printf("== pose set %d: %s\n", set,
              set == 0 ? "random rotations (every joint an arbitrary rotation: a stress bound)"
                       : "bounded poses (joint rotations up to 35 deg, translations up to 5 cm)");
  for (int pose = 0; pose < kPoses; ++pose) {
    for (u32 j = 0; j < slots; ++j) {
      if (set == 0)
        randomSlot(rng, &palette[j * 16]);
      else
        boundedSlot(rng, &palette[j * 16], 0.61, 0.05);
    }
    for (const Mesh& me : meshes)
      for (u32 v = 0; v < me.m->vertexCount; ++v) {
        const s16* q = &me.m->pos[(size_t)v * 3];
        const s8* n = &me.m->nrm[(size_t)v * 4];
        const u8* j4 = &me.m->joints[(size_t)v * 4];
        const u8* w4 = &me.m->weights[(size_t)v * 4];
        double p4[3], n4[3];
        skinCorner(palette.data(), bind, q, n, j4, w4, p4, n4);
        for (int ki = 0; ki < 2; ++ki) {
          u8 j[4], w[4];
          std::memcpy(j, j4, 4);
          std::memcpy(w, w4, 4);
          limitCornerInfluences(j, w, ks[ki]);
          double pk[3], nk[3];
          skinCorner(palette.data(), bind, q, n, j, w, pk, nk);
          const double dmm = 1000.0 * std::sqrt((pk[0] - p4[0]) * (pk[0] - p4[0]) +
                                                (pk[1] - p4[1]) * (pk[1] - p4[1]) +
                                                (pk[2] - p4[2]) * (pk[2] - p4[2]));
          double dot = nk[0] * n4[0] + nk[1] * n4[1] + nk[2] * n4[2];
          dot = std::max(-1.0, std::min(1.0, dot));
          const double dDeg = std::acos(dot) * 180.0 / 3.14159265358979323846;
          mm[ki].add(dmm);
          deg[ki].add(dDeg);
          if (pose % 4 == 0) {
            mm[ki].v.push_back((float)dmm);
            deg[ki].v.push_back((float)dDeg);
          }
          if (dmm > 1.0) ++over1[ki];
          if (dmm > 3.0) ++over3[ki];
          if (dmm > 10.0) ++over10[ki];
          Worst& last = worst[ki][9];
          if (dmm > last.mm) {
            last.mm = dmm;
            last.part = me.part;
            last.level = me.level;
            last.vertex = v;
            std::memcpy(last.j, j4, 4);
            std::memcpy(last.w, w4, 4);
            std::sort(worst[ki], worst[ki] + 10,
                      [](const Worst& a, const Worst& b) { return a.mm > b.mm; });
          }
        }
        ++corners;
      }
  }
  const double dists[4] = {2.0, 8.0, 16.0, 30.0};
  for (int ki = 0; ki < 2; ++ki) {
    const double nC = (double)corners;
    const double p95 = mm[ki].pct(0.95), p99 = mm[ki].pct(0.99);
    std::printf("K=%u vs K=4 over %d poses (%llu corner samples): position mean %.2f mm, p95 %.2f, p99 %.2f, max %.2f; over 1 mm %.1f %%, over 3 mm %.1f %%, over 10 mm %.1f %%\n",
                ks[ki], kPoses, (unsigned long long)corners, mm[ki].sum / nC, p95, p99,
                mm[ki].worst, 100.0 * over1[ki] / nC, 100.0 * over3[ki] / nC,
                100.0 * over10[ki] / nC);
    std::printf("   normal: mean %.2f deg, p95 %.2f deg, max %.2f deg\n", deg[ki].sum / nC,
                deg[ki].pct(0.95), deg[ki].worst);
    for (double d : dists) {
      const double pw = mm[ki].worst * kFocalPx / (d * 1000.0);
      const double pp = p95 * kFocalPx / (d * 1000.0);
      std::printf("   at %4.0f m: p95 %.3f px, worst %.3f px\n", d, pp, pw);
    }
    std::printf("   the p95 error falls below 0.5 px beyond %.1f m\n", p95 * kFocalPx / 500.0);
    std::printf("   the ten worst vertices (K=%u):\n", ks[ki]);
    for (int i = 0; i < 10; ++i) {
      const Worst& wr = worst[ki][i];
      if (wr.mm < 0) continue;
      std::printf("     %6.2f mm  part %u level %u vertex %u  joints %u/%u/%u/%u weights %u/%u/%u/%u\n",
                  wr.mm, wr.part, wr.level, wr.vertex, wr.j[0], wr.j[1], wr.j[2], wr.j[3],
                  wr.w[0], wr.w[1], wr.w[2], wr.w[3]);
    }
    check(mm[ki].sum > 0.0, "the limited skin deviates from the four-influence skin");
  }
  }  // pose sets
  std::printf("verify-skel-influences: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
