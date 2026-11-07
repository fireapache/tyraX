// Host verifier for the influence limit (loaders/3d/tskl_loader/tskl_limit.hpp,
// TYRA_SKIN_MAX_INFLUENCES). Two arms report how far a limited skin (K = 3 and K = 2) lands
// from the four-influence skin (K = 4):
//
//  * the real-clip arm decodes every clip of the combatent .tskl as SkelInstance poses it
//    (sampleChannel / slerp / fromTrs / the parents-first walk / palette = global x ibm: the
//    same float operations), every third frame plus the first and the last, and skins every
//    part and LOD level with the engine's own skin core (SkelSkin::foldPalette and blendCorner,
//    the core the VU0 unique blend must match). It reports, per clip, the position deviation in
//    millimetres and the normal deviation in degrees, the ten worst vertices with their part
//    and joints, and the deviation in screen pixels at 2, 8, 16 and 30 m;
//  * the random-pose arm skins in double precision over random blended poses. It is an UPPER
//    BOUND (the joint rotations are far larger than any clip's), not a prediction.
//
// Screen pixels: a 640 x 448 screen with a 60 degree vertical field of view (focal length
// 388 px), error_px = deviation_mm x focal_px / distance_mm.
//
// Checks: K = 4 is the identity (byte-equal); a limited corner keeps at most K influences and
// its weights sum to 255. The deviations are reported, not gated: the acceptance is the visual
// judgement in docs/animated-models.md.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel does this):
//   build/verify-skel-influences res/models/combatent__ovrb22b.tskl

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "loaders/3d/tskl_loader/tskl_limit.hpp"
#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

#if TYRA_SKIN_MAX_INFLUENCES != 4
#error "verify-skel-influences measures K = 3 and K = 2 against the model's own K = 4 weights: build it with the default limit"
#endif

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


// ---- the clip arm: the engine's pose, mirrored from SkelInstance (skel_instance.cpp) --------

void fromTrs(float* m, const float* t, const float* q, const float* s) {
  const float x = q[0], y = q[1], z = q[2], w = q[3];
  const float x2 = x + x, y2 = y + y, z2 = z + z;
  const float xx = x * x2, xy = x * y2, xz = x * z2;
  const float yy = y * y2, yz = y * z2, zz = z * z2;
  const float wx = w * x2, wy = w * y2, wz = w * z2;
  m[0] = (1.0F - (yy + zz)) * s[0];
  m[1] = (xy + wz) * s[0];
  m[2] = (xz - wy) * s[0];
  m[3] = 0.0F;
  m[4] = (xy - wz) * s[1];
  m[5] = (1.0F - (xx + zz)) * s[1];
  m[6] = (yz + wx) * s[1];
  m[7] = 0.0F;
  m[8] = (xz + wy) * s[2];
  m[9] = (yz - wx) * s[2];
  m[10] = (1.0F - (xx + yy)) * s[2];
  m[11] = 0.0F;
  m[12] = t[0];
  m[13] = t[1];
  m[14] = t[2];
  m[15] = 1.0F;
}

// Column-major r = a * b (the VU0 sequence of mulM4Vu0: every column of b through a's columns).
void mulM4(float* r, const float* a, const float* b) {
  float out[16];
  for (int c = 0; c < 4; ++c)
    for (int row = 0; row < 4; ++row)
      out[c * 4 + row] = a[0 + row] * b[c * 4 + 0] + a[4 + row] * b[c * 4 + 1] +
                         a[8 + row] * b[c * 4 + 2] + a[12 + row] * b[c * 4 + 3];
  std::memcpy(r, out, sizeof(out));
}

void slerp(const float* a, const float* bIn, float t, float* out) {
  float b[4] = {bIn[0], bIn[1], bIn[2], bIn[3]};
  float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0.0F) {
    dot = -dot;
    for (int i = 0; i < 4; ++i) b[i] = -b[i];
  }
  float wa, wb;
  if (dot > 0.9995F) {
    wa = 1.0F - t;
    wb = t;
  } else {
    const float theta = acosf(dot > 1.0F ? 1.0F : dot);
    const float sinTheta = sinf(theta);
    wa = sinf((1.0F - t) * theta) / sinTheta;
    wb = sinf(t * theta) / sinTheta;
  }
  for (int i = 0; i < 4; ++i) out[i] = wa * a[i] + wb * b[i];
  const float len = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
  if (len > 1e-6F)
    for (int i = 0; i < 4; ++i) out[i] /= len;
  else
    out[3] = 1.0F;
}

u32 findKey(const std::vector<float>& times, float t) {
  const u32 n = (u32)times.size();
  u32 hi = 0;
  while (hi < n && times[hi] < t) ++hi;
  return hi;
}

void sampleChannel(const SkelChannel& ch, float t, float* out) {
  const u32 n = (u32)ch.times.size();
  const u32 hi = findKey(ch.times, t);
  if (ch.path == 1) {
    float a[4], b[4];
    const u32 lo = hi == 0 ? 0 : hi - 1;
    const u32 hiC = hi >= n ? n - 1 : hi;
    for (int c = 0; c < 4; ++c) {
      a[c] = ch.quat[(size_t)lo * 4 + c] * (1.0F / 32767.0F);
      b[c] = ch.quat[(size_t)hiC * 4 + c] * (1.0F / 32767.0F);
    }
    if (hi == 0 || hi >= n || lo == hiC) {
      std::memcpy(out, hi == 0 ? a : b, 4 * sizeof(float));
      const float len =
          sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
      if (len > 1e-6F)
        for (int i = 0; i < 4; ++i) out[i] /= len;
      return;
    }
    const float t0 = ch.times[lo], t1 = ch.times[hi];
    float f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0F;
    if (ch.step) f = 0.0F;
    slerp(a, b, f, out);
    return;
  }
  if (hi == 0) {
    std::memcpy(out, &ch.vec[0], 3 * sizeof(float));
    return;
  }
  if (hi >= n) {
    std::memcpy(out, &ch.vec[(size_t)(n - 1) * 3], 3 * sizeof(float));
    return;
  }
  const u32 lo = hi - 1;
  const float t0 = ch.times[lo], t1 = ch.times[hi];
  float f = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0F;
  if (ch.step) f = 0.0F;
  const float* a = &ch.vec[(size_t)lo * 3];
  const float* b = &ch.vec[(size_t)hi * 3];
  for (int c = 0; c < 3; ++c) out[c] = a[c] + (b[c] - a[c]) * f;
}

// The palette (global(node) x ibm per slot, 16 floats each) of `clip` at time `t`.
void clipPalette(const SkelModel& model, const SkelClip& clip, float t,
                 std::vector<float>& palette) {
  const size_t nodeCount = model.nodes.size();
  std::vector<float> locals(nodeCount * 10);
  std::vector<u8> animated(nodeCount, 0);
  for (size_t i = 0; i < nodeCount; ++i) {
    const SkelNode& node = model.nodes[i];
    std::memcpy(&locals[i * 10], node.t, 3 * sizeof(float));
    std::memcpy(&locals[i * 10 + 3], node.r, 4 * sizeof(float));
    std::memcpy(&locals[i * 10 + 7], node.s, 3 * sizeof(float));
  }
  for (const SkelChannel& ch : clip.channels) {
    float* l = &locals[(size_t)ch.node * 10];
    sampleChannel(ch, t, ch.path == 0 ? l : (ch.path == 1 ? l + 3 : l + 7));
    animated[ch.node] = 1;
  }
  std::vector<std::array<float, 16>> globals(nodeCount);
  for (u32 i : model.order) {
    const SkelNode& node = model.nodes[i];
    float local[16];
    const float* localPtr = local;
    if (node.hasMatrix && !animated[i]) {
      localPtr = node.matrix;
    } else {
      const float* l = &locals[(size_t)i * 10];
      fromTrs(local, l, l + 3, l + 7);
    }
    if (node.parent >= 0)
      mulM4(globals[i].data(), globals[node.parent].data(), localPtr);
    else
      std::memcpy(globals[i].data(), localPtr, 16 * sizeof(float));
  }
  palette.assign(model.palette.size() * 16, 0.0F);
  for (size_t j = 0; j < model.palette.size(); ++j)
    mulM4(&palette[j * 16], globals[model.palette[j].node].data(), model.palette[j].ibm);
}

// A fine histogram of non-negative samples: percentiles to the bin width, exact mean and worst.
struct Hist {
  double binWidth;
  std::vector<u64> bins;
  u64 n = 0;
  double sum = 0.0, worst = 0.0;
  Hist(double width, double range) : binWidth(width), bins((size_t)(range / width) + 2, 0) {}
  void add(double x) {
    ++n;
    sum += x;
    if (x > worst) worst = x;
    size_t b = (size_t)(x / binWidth);
    if (b >= bins.size()) b = bins.size() - 1;
    ++bins[b];
  }
  double mean() const { return n ? sum / (double)n : 0.0; }
  double pct(double p) const {
    if (!n) return 0.0;
    const u64 target = (u64)std::ceil(p * (double)n);
    u64 run = 0;
    for (size_t b = 0; b < bins.size(); ++b) {
      run += bins[b];
      if (run >= target && run > 0) return (double)b * binWidth;
    }
    return worst;
  }
};

// The deviation samples of one group (a clip, a LOD level or a part).
struct Dev {
  Hist mm{0.01, 400.0}, deg{0.01, 180.0};
};

// The worst deviation seen at one bind corner over all the sampled poses.
struct CornerWorst {
  float mm = 0.0F;
  u16 clip = 0, frame = 0;
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

  // The REAL-CLIP arm: every clip of the model, every third frame plus the first and the last,
  // posed as SkelInstance poses it, every part and LOD level skinned with the engine's own
  // skin core (foldPalette + blendCorner) at K = 4, 3 and 2. K = 4 must equal the unlimited
  // weights bit for bit (the default build must not move). Deviations are in the output box's
  // units converted to millimetres.
  {
    const Compact::Box outBox = skinOutputBox(model);
    const double stepMm = 1000.0 * (double)outBox.step;
    const u32 ks[3] = {4, 3, 2};
    // The limited weights of every mesh for K = 4 (the identity), 3 and 2.
    std::vector<std::vector<u8>> limJ[3], limW[3];
    for (int ki = 0; ki < 3; ++ki) {
      limJ[ki].resize(meshes.size());
      limW[ki].resize(meshes.size());
      for (size_t mi = 0; mi < meshes.size(); ++mi) {
        const SkelMesh& m = *meshes[mi].m;
        limJ[ki][mi] = m.joints;
        limW[ki][mi] = m.weights;
        for (u32 v = 0; v < m.vertexCount; ++v)
          limitCornerInfluences(&limJ[ki][mi][(size_t)v * 4], &limW[ki][mi][(size_t)v * 4], ks[ki]);
      }
    }
    const size_t clipCount = model.clips.size();
    // [ki: K = 3, 2][clip] at LOD0, [ki][level], [ki][part] at LOD0.
    std::vector<Dev> byClip[2], byPart[2];
    std::vector<Dev> byLevel[2];
    for (int ki = 0; ki < 2; ++ki) {
      byClip[ki].resize(clipCount);
      byPart[ki].resize(model.parts.size());
      byLevel[ki].resize(3);
    }
    std::vector<std::vector<CornerWorst>> cornerWorst[2];
    for (int ki = 0; ki < 2; ++ki) {
      cornerWorst[ki].resize(meshes.size());
      for (size_t mi = 0; mi < meshes.size(); ++mi)
        cornerWorst[ki][mi].resize(meshes[mi].m->vertexCount);
    }
    std::vector<u32> frameCounts(clipCount, 0);
    u64 identityCompared = 0, identityDiffers = 0;
    SkelSkin::Folded folded;
    std::vector<float> palette;
    for (size_t ci = 0; ci < clipCount; ++ci) {
      const SkelClip& clip = model.clips[ci];
      const int frames = clip.duration > 0.0F ? (int)std::lround(clip.duration * 30.0F) : 0;
      std::vector<int> sampled;
      for (int f = 0; f <= frames; f += 3) sampled.push_back(f);
      if (sampled.back() != frames) sampled.push_back(frames);
      frameCounts[ci] = (u32)sampled.size();
      for (int f : sampled) {
        clipPalette(model, clip, (float)f / 30.0F, palette);
        SkelSkin::foldPalette(palette.data(), slots, bind, outBox, folded);
        for (size_t mi = 0; mi < meshes.size(); ++mi) {
          const Mesh& me = meshes[mi];
          for (u32 v = 0; v < me.m->vertexCount; ++v) {
            const s16* q = &me.m->pos[(size_t)v * 3];
            const s8* n = &me.m->nrm[(size_t)v * 4];
            float p4[3], n4[3];
            SkelSkin::blendCorner(folded, q, n, &me.m->joints[(size_t)v * 4],
                                  &me.m->weights[(size_t)v * 4], p4, n4);
            // K = 4 through the limiter is the unlimited blend, bit for bit.
            {
              float pi[3], ni[3];
              SkelSkin::blendCorner(folded, q, n, &limJ[0][mi][(size_t)v * 4],
                                    &limW[0][mi][(size_t)v * 4], pi, ni);
              ++identityCompared;
              if (std::memcmp(pi, p4, sizeof(pi)) != 0 || std::memcmp(ni, n4, sizeof(ni)) != 0)
                ++identityDiffers;
            }
            const double l4 = std::sqrt((double)n4[0] * n4[0] + (double)n4[1] * n4[1] +
                                        (double)n4[2] * n4[2]);
            for (int ki = 0; ki < 2; ++ki) {
              float pk[3], nk[3];
              SkelSkin::blendCorner(folded, q, n, &limJ[ki + 1][mi][(size_t)v * 4],
                                    &limW[ki + 1][mi][(size_t)v * 4], pk, nk);
              const double dx = (double)pk[0] - p4[0], dy = (double)pk[1] - p4[1],
                           dz = (double)pk[2] - p4[2];
              const double dmm = stepMm * std::sqrt(dx * dx + dy * dy + dz * dz);
              const double lk = std::sqrt((double)nk[0] * nk[0] + (double)nk[1] * nk[1] +
                                          (double)nk[2] * nk[2]);
              double dot = (l4 > 0.0 && lk > 0.0)
                               ? ((double)nk[0] * n4[0] + (double)nk[1] * n4[1] +
                                  (double)nk[2] * n4[2]) / (l4 * lk)
                               : 1.0;
              dot = std::max(-1.0, std::min(1.0, dot));
              const double dDeg = std::acos(dot) * 180.0 / 3.14159265358979323846;
              byLevel[ki][me.level].mm.add(dmm);
              byLevel[ki][me.level].deg.add(dDeg);
              if (me.level == 0) {
                byClip[ki][ci].mm.add(dmm);
                byClip[ki][ci].deg.add(dDeg);
                byPart[ki][me.part].mm.add(dmm);
                byPart[ki][me.part].deg.add(dDeg);
              }
              CornerWorst& cw = cornerWorst[ki][mi][v];
              if ((float)dmm > cw.mm) {
                cw.mm = (float)dmm;
                cw.clip = (u16)ci;
                cw.frame = (u16)f;
              }
            }
          }
        }
      }
    }
    std::printf("== real clips: %zu clips, frames every 3rd + first/last at 30 fps, %zu meshes (%u parts x LODs), the engine's pose and skin core; deviation vs K=4\n",
                clipCount, meshes.size(), (unsigned)model.parts.size());
    std::printf("K=4 through the limiter: %llu corner skins compared with the unlimited weights, %llu differ\n",
                (unsigned long long)identityCompared, (unsigned long long)identityDiffers);
    check(identityDiffers == 0, "K=4 skin output is bit-identical to the unlimited weights (real clips)");
    check(identityCompared > 0, "the real-clip arm skinned corners");

    const double dists[4] = {2.0, 8.0, 16.0, 30.0};
    for (int ki = 0; ki < 2; ++ki) {
      std::printf("\n-- K=%u vs K=4, LOD0, per clip (mm: mean p95 p99 max | normal deg: mean p95 max | px p95/worst at 2, 8, 16, 30 m)\n", ks[ki + 1]);
      for (size_t ci = 0; ci < clipCount; ++ci) {
        const Dev& d = byClip[ki][ci];
        const double p95 = d.mm.pct(0.95), mx = d.mm.worst;
        std::printf("  %-18s %3u fr | %6.2f %6.2f %6.2f %7.2f | %5.2f %5.2f %6.2f |", model.clips[ci].name.c_str(),
                    frameCounts[ci], d.mm.mean(), p95, d.mm.pct(0.99), mx, d.deg.mean(),
                    d.deg.pct(0.95), d.deg.worst);
        for (double dist : dists)
          std::printf(" %.2f/%.2f", p95 * kFocalPx / (dist * 1000.0), mx * kFocalPx / (dist * 1000.0));
        std::printf("\n");
      }
      Dev all;
      for (size_t ci = 0; ci < clipCount; ++ci) {
        // merge the per-clip histograms into one
        Dev& d = byClip[ki][ci];
        for (size_t b = 0; b < d.mm.bins.size() && b < all.mm.bins.size(); ++b) all.mm.bins[b] += d.mm.bins[b];
        for (size_t b = 0; b < d.deg.bins.size() && b < all.deg.bins.size(); ++b) all.deg.bins[b] += d.deg.bins[b];
        all.mm.n += d.mm.n;
        all.mm.sum += d.mm.sum;
        all.mm.worst = std::max(all.mm.worst, d.mm.worst);
        all.deg.n += d.deg.n;
        all.deg.sum += d.deg.sum;
        all.deg.worst = std::max(all.deg.worst, d.deg.worst);
      }
      {
        const double p95 = all.mm.pct(0.95), mx = all.mm.worst;
        std::printf("  %-18s %6s | %6.2f %6.2f %6.2f %7.2f | %5.2f %5.2f %6.2f |", "ALL CLIPS", "",
                    all.mm.mean(), p95, all.mm.pct(0.99), mx, all.deg.mean(), all.deg.pct(0.95),
                    all.deg.worst);
        for (double dist : dists)
          std::printf(" %.2f/%.2f", p95 * kFocalPx / (dist * 1000.0), mx * kFocalPx / (dist * 1000.0));
        std::printf("\n  the LOD0 p95 error falls below 0.5 px beyond %.1f m (worst-case beyond %.1f m)\n",
                    p95 * kFocalPx / 500.0, mx * kFocalPx / 500.0);
      }
      std::printf("-- K=%u vs K=4, per LOD level over all clips (the level shown at the distance: LOD0 near, LOD1 from 8 m, LOD2 from 16 m)\n", ks[ki + 1]);
      for (int lv = 0; lv < 3; ++lv) {
        const Dev& d = byLevel[ki][lv];
        const double p95 = d.mm.pct(0.95), mx = d.mm.worst;
        std::printf("  LOD%d | mm mean %.2f p95 %.2f p99 %.2f max %.2f | normal mean %.2f p95 %.2f max %.2f deg |", lv,
                    d.mm.mean(), p95, d.mm.pct(0.99), mx, d.deg.mean(), d.deg.pct(0.95), d.deg.worst);
        for (double dist : dists)
          std::printf(" %.0fm %.2f/%.2f px", dist, p95 * kFocalPx / (dist * 1000.0),
                      mx * kFocalPx / (dist * 1000.0));
        std::printf("\n");
      }
      std::printf("-- K=%u vs K=4, LOD0 per part over all clips\n", ks[ki + 1]);
      for (u32 pi = 0; pi < model.parts.size(); ++pi) {
        const Dev& d = byPart[ki][pi];
        if (!d.mm.n) continue;
        std::printf("  %-16s | mm mean %.2f p95 %.2f p99 %.2f max %.2f | normal mean %.2f p95 %.2f max %.2f deg\n",
                    model.parts[pi].name.c_str(), d.mm.mean(), d.mm.pct(0.95), d.mm.pct(0.99),
                    d.mm.worst, d.deg.mean(), d.deg.pct(0.95), d.deg.worst);
      }
      // The ten worst distinct corners (a repeated corner of the same bind data counts once).
      struct Entry {
        float mm;
        size_t mi;
        u32 v;
      };
      std::vector<Entry> all10;
      for (size_t mi = 0; mi < meshes.size(); ++mi)
        for (u32 v = 0; v < meshes[mi].m->vertexCount; ++v)
          all10.push_back({cornerWorst[ki][mi][v].mm, mi, v});
      std::sort(all10.begin(), all10.end(), [](const Entry& a, const Entry& b) { return a.mm > b.mm; });
      std::printf("-- K=%u: the ten worst corners over every clip and LOD\n", ks[ki + 1]);
      int shown = 0;
      std::vector<std::vector<u8>> seen;
      for (const Entry& e : all10) {
        if (shown == 10) break;
        const SkelMesh& m = *meshes[e.mi].m;
        std::vector<u8> key(sizeof(u32) * 2 + 6 + 8);
        std::memcpy(&key[0], &meshes[e.mi].part, 4);
        std::memcpy(&key[4], &meshes[e.mi].level, 4);
        std::memcpy(&key[8], &m.pos[(size_t)e.v * 3], 6);
        std::memcpy(&key[14], &m.joints[(size_t)e.v * 4], 4);
        std::memcpy(&key[18], &m.weights[(size_t)e.v * 4], 4);
        if (std::find(seen.begin(), seen.end(), key) != seen.end()) continue;
        seen.push_back(key);
        ++shown;
        const u8* j4 = &m.joints[(size_t)e.v * 4];
        const u8* w4 = &m.weights[(size_t)e.v * 4];
        const CornerWorst& cw = cornerWorst[ki][e.mi][e.v];
        std::printf("   %6.2f mm  %-14s LOD%u v%-5u %-16s f%-3u |", e.mm,
                    model.parts[meshes[e.mi].part].name.c_str(), meshes[e.mi].level, e.v,
                    model.clips[cw.clip].name.c_str(), (unsigned)cw.frame);
        for (int s = 0; s < 4; ++s)
          if (w4[s])
            std::printf(" %s:%u", model.nodes[model.palette[j4[s]].node].name.c_str(), (unsigned)w4[s]);
        std::printf("\n");
      }
    }
    std::printf("\n");
  }

  // The random-pose arm: an UPPER BOUND (joint rotations far beyond any clip).
  // the deviation of K = 3 and K = 2 from K = 4 over random poses.
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
              set == 0 ? "random rotations (every joint an arbitrary rotation: an UPPER BOUND, not a prediction)"
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
