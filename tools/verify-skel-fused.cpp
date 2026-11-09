// Host verifier for the fused quantize and the part bounds of the unique-vertex skin
// (skel_instance.cpp: skinEmitUniques, the VU0 macro loop that quantizes as it blends, and the
// MMI part bounds; skel_unique.hpp: originFromExtent, the bounds lane model). The EE kernels run
// the sequences below on the device; this checks them against the plain reference the stream has
// always been written with (poseOrigin + quantizeOne + the scalar range loop), EXACTLY, byte for
// byte:
//
//  - the quantize: x = p / step - origin, rounded half away from zero as trunc(x +- 0.5), in the
//    EE's float arithmetic. The EE's add and subtract round to nearest on PCSX2 and truncate on
//    the console; the VU0 truncates. The fused kernel computes trunc(2x) - trunc(x) on the VU0
//    and re-quantizes a unique with the reference arithmetic when a lane is within a few ulp of
//    a tie or a position is within one step of the clamp. This checks that the kernel's bytes
//    equal the reference's under BOTH EE roundings, on ties, near ties (a dense sweep of the
//    ulps around every kind of tie, with origins that make the subtraction inexact), the clamp
//    limits and the soldier model's real blends, and counts how often the re-quantize is taken;
//  - the pose origin from the blended extent (min / max accumulated in any order);
//  - the part bounds: the packed-halfword min / max over the ranges of every unique, against the
//    scalar loop, over the soldier model's real range lists.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel-fused does this):
//   build/verify-skel-fused <model>.tskl

#include <cfloat>
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
namespace U = Tyra::SkelSkin::Unique;

int checks = 0;
int failures = 0;

void check(bool ok, const char* what) {
  ++checks;
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
  }
}

// ---- float arithmetic of the two units ----------------------------------------------------

enum class Round { Nearest, Chop };

float chopTo(double d) {
  float f = (float)d;  // nearest
  if (std::fabs((double)f) > std::fabs(d)) f = std::nextafter(f, 0.0F);
  return f;
}
float rnd(Round m, double d) { return m == Round::Chop ? chopTo(d) : (float)d; }
float opMul(Round m, float a, float b) { return rnd(m, (double)a * (double)b); }
float opAdd(Round m, float a, float b) { return rnd(m, (double)a + (double)b); }
float opSub(Round m, float a, float b) { return rnd(m, (double)a - (double)b); }

// The reference: Unique::quantizeOne's operations on an EE that rounds as `ee`.
u32 quantizeRef(Round ee, const float* p, const float* a, float invStep, const s32 oi[3], s16* qPos,
                s8* qNrm) {
  u32 clip = 0;
  for (int i = 0; i < 3; ++i) {
    float x = opSub(ee, opMul(ee, p[i], invStep), (float)oi[i]);
    if (x > 32767.0F) {
      x = 32767.0F;
      clip = 1;
    } else if (x < -32767.0F) {
      x = -32767.0F;
      clip = 1;
    }
    qPos[i] = (s16)(s32)opAdd(ee, x, x < 0.0F ? -0.5F : 0.5F);
  }
  qPos[3] = 0;
  for (int i = 0; i < 3; ++i) {
    float y = opMul(ee, a[i], 127.0F);
    if (y > 127.0F) y = 127.0F;
    else if (y < -127.0F) y = -127.0F;
    qNrm[i] = (s8)(s32)opAdd(ee, y, y < 0.0F ? -0.5F : 0.5F);
  }
  qNrm[3] = 0;
  return clip;
}

// vftoi0: truncate toward zero, saturating.
s32 tr(float f) {
  if (f >= 2147483648.0F) return 0x7FFFFFFF;
  if (f <= -2147483648.0F) return (s32)0x80000000;
  return (s32)f;
}

// The kernel (skinEmitUniques) on a VU0 that truncates, then the reference arithmetic when it
// asks for it. `slow` is set when it did.
const float kDelta = 1.0F / 1048576.0F;

// One lane's tie test: a tie (an odd integer) between |2v| and |2v| (1 + 2^-20) + 2^-20.
bool tieClose(float v2) {
  const Round vu = Round::Chop;
  const float a2 = std::fabs(v2);
  const float s = opAdd(vu, a2, opAdd(vu, opMul(vu, a2, kDelta), kDelta));
  const s32 t2 = tr(v2), t1 = tr(s);
  return (t1 & 1) && !(t2 & 1);
}

u32 quantizeFused(Round ee, const float* p, const float* n, float invStep, const s32 oi[3],
                  s16* qPos, s8* qNrm, bool& slow) {
  const Round vu = Round::Chop;
  bool danger = false;
  s16 fp[4] = {0, 0, 0, 0};
  s8 fn[4] = {0, 0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    const float x = opSub(vu, opMul(vu, p[i], invStep), (float)oi[i]);
    if (x > 32766.0F || x < -32766.0F) danger = true;
    const float z = x + x;  // exact
    fp[i] = (s16)(tr(z) - tr(x));
    if (tieClose(z)) danger = true;
  }
  for (int i = 0; i < 3; ++i) {
    const float y = opMul(vu, n[i], 127.0F);
    const float z = y + y;
    s32 q = tr(z) - tr(y);
    if (q > 127) q = 127;
    if (q < -127) q = -127;
    fn[i] = (s8)q;
    if (tieClose(z)) danger = true;
  }
  slow = danger;
  if (danger) return quantizeRef(ee, p, n, invStep, oi, qPos, qNrm);
  std::memcpy(qPos, fp, 8);
  std::memcpy(qNrm, fn, 4);
  return 0;  // no lane is near the clamp: nothing counted
}

// The one-blend route (skinBlendKeepUniques + skinQuantKeptUniques): run one stores the blended position
// (the exact floats: a copy) and quantizes the normal; run two quantizes the stored position. Each
// half asks for the reference arithmetic on its own lanes: the normal with a zero position (nothing
// can clamp), the position with a zero normal (its normal output is discarded). `slowPos` /
// `slowNrm` tell which half did.
u32 quantizeSplit(Round ee, const float* p, const float* n, float invStep, const s32 oi[3], s16* qPos,
                  s8* qNrm, bool& slowPos, bool& slowNrm) {
  const Round vu = Round::Chop;
  alignas(16) float kept[4];  // the kept row
  std::memcpy(kept, p, sizeof kept);
  // run one, the normal
  bool dn = false;
  s8 fn[4] = {0, 0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    const float y = opMul(vu, n[i], 127.0F);
    const float z = y + y;
    s32 q = tr(z) - tr(y);
    if (q > 127) q = 127;
    if (q < -127) q = -127;
    fn[i] = (s8)q;
    if (tieClose(z)) dn = true;
  }
  if (dn) {
    const float zp[4] = {0.0F, 0.0F, 0.0F, 1.0F};
    const s32 z3[3] = {0, 0, 0};
    s16 scrap[4];
    quantizeRef(ee, zp, n, 1.0F, z3, scrap, qNrm);
  } else {
    std::memcpy(qNrm, fn, 4);
  }
  slowNrm = dn;
  // run two, the position from the kept row
  bool dp = false;
  s16 fp[4] = {0, 0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    const float x = opSub(vu, opMul(vu, kept[i], invStep), (float)oi[i]);
    if (x > 32766.0F || x < -32766.0F) dp = true;
    const float z = x + x;
    fp[i] = (s16)(tr(z) - tr(x));
    if (tieClose(z)) dp = true;
  }
  slowPos = dp;
  if (dp) {
    const float zn[4] = {0.0F, 0.0F, 0.0F, 0.0F};
    s8 scrap[4];
    return quantizeRef(ee, kept, zn, invStep, oi, qPos, scrap);
  }
  std::memcpy(qPos, fp, 8);
  return 0;
}

long long compared = 0, slowCount = 0;
long long mismatchPos[2] = {0, 0}, mismatchNrm[2] = {0, 0}, mismatchClip[2] = {0, 0};
long long splitPos[2] = {0, 0}, splitNrm[2] = {0, 0}, splitClip[2] = {0, 0}, splitSlowPos = 0, splitSlowNrm = 0;

void compareOne(const float* p, const float* a, float invStep, const s32 oi[3]) {
  for (int m = 0; m < 2; ++m) {
    const Round ee = m == 0 ? Round::Nearest : Round::Chop;
    s16 rp[4], fp[4];
    s8 rn[4], fn[4];
    bool slow = false;
    const u32 rc = quantizeRef(ee, p, a, invStep, oi, rp, rn);
    const u32 fc = quantizeFused(ee, p, a, invStep, oi, fp, fn, slow);
    if (m == 0) {
      ++compared;
      if (slow) ++slowCount;
    }
    if (std::memcmp(rp, fp, 8) != 0) ++mismatchPos[m];
    if (std::memcmp(rn, fn, 4) != 0) ++mismatchNrm[m];
    if (rc != fc) ++mismatchClip[m];
    s16 sp[4];
    s8 sn[4];
    bool slowP = false, slowN = false;
    const u32 sc = quantizeSplit(ee, p, a, invStep, oi, sp, sn, slowP, slowN);
    if (m == 0) {
      if (slowP) ++splitSlowPos;
      if (slowN) ++splitSlowNrm;
    }
    if (std::memcmp(rp, sp, 8) != 0) ++splitPos[m];
    if (std::memcmp(rn, sn, 4) != 0) ++splitNrm[m];
    if (rc != sc) ++splitClip[m];
  }
}

float nextUp(float v) { return std::nextafter(v, INFINITY); }
float nextDown(float v) { return std::nextafter(v, -INFINITY); }

// Ties, near ties, zeros and limits on one axis, in a unit position and a unit normal. The
// positions are built as t - oi with a t whose low bits make the subtraction inexact.
void tieCases(std::mt19937& rng) {
  const float step = 1.0F / 8192.0F, invStep = 8192.0F;
  std::uniform_int_distribution<int> oiDist(-32000, 32000);
  std::uniform_int_distribution<int> ulpDist(-12, 12);
  std::uniform_int_distribution<int> lowBits(0, (1 << 6) - 1);
  for (int rep = 0; rep < 40; ++rep) {
    s32 oi[3] = {oiDist(rng), oiDist(rng), oiDist(rng)};
    for (int k = -66000; k <= 66000; k += 3) {
      const float target = (float)k * 0.5F;  // an integer or an exact tie
      for (int axis = 0; axis < 3; ++axis) {
        float p[4] = {0.0F, 0.0F, 0.0F, 1.0F};
        const float n[4] = {0.0F, 0.0F, 0.0F, 0.0F};
        // t = target + oi, nudged by some ulps of t and low bits: x = t - oi is then inexact
        float t = target + (float)oi[axis];
        for (int u = ulpDist(rng); u > 0; --u) t = nextUp(t);
        for (int u = -ulpDist(rng); u > 0; --u) t = nextDown(t);
        p[axis] = t * step;  // exact: step is a power of two
        compareOne(p, n, invStep, oi);
      }
    }
  }
  // normals: y = n * 127 on and around the ties of the 1/127 grid, and past the clamp
  const s32 zero[3] = {0, 0, 0};
  const float p0[4] = {0.0F, 0.0F, 0.0F, 1.0F};
  for (int k = -300; k <= 300; ++k) {
    const float y = (float)k * 0.5F;
    for (int u = -20; u <= 20; ++u) {
      float v = y;
      for (int j = 0; j < std::abs(u); ++j) v = u > 0 ? nextUp(v) : nextDown(v);
      for (int axis = 0; axis < 3; ++axis) {
        float n[4] = {0.0F, 0.0F, 0.0F, 0.0F};
        n[axis] = v * (1.0F / 127.0F);
        compareOne(p0, n, invStep, zero);
        n[axis] = v;
        compareOne(p0, n, invStep, zero);
      }
    }
  }
  // the clamp limits: at, one step either side, far past
  const float lim[] = {32765.0F, 32766.0F, 32766.5F, 32767.0F, nextUp(32767.0F), nextDown(32767.0F),
                       32767.4F, 32767.5F, 32767.6F, 40000.0F, 1e9F, 1e12F};
  for (float l : lim)
    for (float sg : {1.0F, -1.0F})
      for (int axis = 0; axis < 3; ++axis) {
        float p[4] = {0.0F, 0.0F, 0.0F, 1.0F};
        p[axis] = sg * l * step;
        const float n[4] = {0.0F, 0.0F, 0.0F, 0.0F};
        compareOne(p, n, invStep, zero);
      }
}

// ---- the bind data of the real model ------------------------------------------------------

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

// The scalar range loop of the indexed pass, as it was: s32 lo / hi per range and axis.
void boundsScalar(const std::vector<u32>& rngOff, const std::vector<u16>& rng, u32 count,
                  const s16* qp, u32 ranges, std::vector<s32>& lo, std::vector<s32>& hi) {
  lo.assign(3 * (size_t)ranges, 32767);
  hi.assign(3 * (size_t)ranges, -32768);
  for (u32 k = 0; k < count; ++k) {
    const s16* q = qp + 4 * (size_t)k;
    for (u32 j = rngOff[k]; j < rngOff[k + 1]; ++j) {
      s32* l = &lo[3 * (size_t)rng[j]];
      s32* h = &hi[3 * (size_t)rng[j]];
      for (int i = 0; i < 3; ++i) {
        if (q[i] < l[i]) l[i] = q[i];
        if (q[i] > h[i]) h[i] = q[i];
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: verify-skel-fused <model.tskl>\n");
    return 2;
  }
  std::mt19937 rng(0xFEEDu);

  // 1. The quantize on its own: ties, near ties, limits.
  tieCases(rng);
  std::printf("ties, near ties and limits: %lld inputs, %lld took the reference arithmetic; "
              "mismatches (EE nearest / EE chop): positions %lld / %lld, normals %lld / %lld, "
              "clamp counts %lld / %lld\n",
              compared, slowCount, mismatchPos[0], mismatchPos[1], mismatchNrm[0], mismatchNrm[1],
              mismatchClip[0], mismatchClip[1]);
  check(mismatchPos[0] == 0 && mismatchNrm[0] == 0 && mismatchClip[0] == 0,
        "fused == reference on ties, near ties and limits (EE rounds to nearest, as PCSX2)");
  check(mismatchPos[1] == 0 && mismatchNrm[1] == 0 && mismatchClip[1] == 0,
        "fused == reference on ties, near ties and limits (EE truncates, as the console)");
  std::printf("  one-blend route (kept position + normal in run one): %lld position and %lld normal re-quantizations; "
              "mismatches (EE nearest / chop): positions %lld / %lld, normals %lld / %lld, clamp counts %lld / %lld\n",
              splitSlowPos, splitSlowNrm, splitPos[0], splitPos[1], splitNrm[0], splitNrm[1], splitClip[0], splitClip[1]);
  check(splitPos[0] == 0 && splitNrm[0] == 0 && splitClip[0] == 0,
        "one-blend == reference on ties, near ties and limits (EE rounds to nearest, as PCSX2)");
  check(splitPos[1] == 0 && splitNrm[1] == 0 && splitClip[1] == 0,
        "one-blend == reference on ties, near ties and limits (EE truncates, as the console)");
  check(splitSlowPos > 0 && splitSlowNrm > 0, "the one-blend halves both take the reference arithmetic on ties");

  // 2. The real model.
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
    std::printf("verify-skel-fused: %d checks, %d failed\n", checks, failures);
    return 1;
  }
  const Compact::Box bind = model.box;
  const Compact::Box outBox = skinOutputBox(model);
  const float step = outBox.step, invStep = 1.0F / step;
  const u32 slots = (u32)model.palette.size();

  struct Level {
    const SkelMesh* mesh;
    std::vector<u32> src;
    U::UniqueBind ub;
    std::vector<u32> rngOff;  // the unique's stream parts (the scatter's lists)
    std::vector<u16> rng;
    u32 ranges;
  };
  const u32 kPackage = 75, kPart = kPackage / 3;
  std::vector<Level> levels;
  for (size_t pi = 0; pi < model.parts.size(); ++pi) {
    const size_t nl = 1 + model.parts[pi].lods.size();
    for (size_t l = 0; l < nl; ++l) {
      Level lv;
      lv.mesh = l == 0 ? static_cast<const SkelMesh*>(&model.parts[pi])
                       : static_cast<const SkelMesh*>(&model.parts[pi].lods[l - 1]);
      lv.src = firstIdentical(*lv.mesh);
      U::buildUnique(lv.mesh->pos.data(), lv.mesh->nrm.data(), lv.mesh->joints.data(),
                     lv.mesh->weights.data(), lv.src.data(), lv.mesh->vertexCount, bind, lv.ub);
      // the stream parts each unique's corners fall in, in corner order, no repeats
      lv.ranges = (lv.mesh->vertexCount + kPart - 1) / kPart;
      std::vector<std::vector<u16>> per(lv.ub.count);
      std::vector<u32> rank(lv.mesh->vertexCount, 0);
      for (u32 u = 0; u < lv.ub.count; ++u) rank[lv.ub.corner[u]] = u;
      for (u32 v = 0; v < lv.mesh->vertexCount; ++v) {
        auto& list = per[rank[lv.src[v]]];
        const u16 r = (u16)(v / kPart);
        if (list.empty() || list.back() != r) list.push_back(r);
      }
      lv.rngOff.assign(lv.ub.count + 1, 0);
      for (u32 u = 0; u < lv.ub.count; ++u) {
        lv.rngOff[u + 1] = lv.rngOff[u] + (u32)per[u].size();
        lv.rng.insert(lv.rng.end(), per[u].begin(), per[u].end());
      }
      levels.push_back(std::move(lv));
    }
  }
  check(!levels.empty(), "the model has parts");

  std::vector<float> palette(slots * 16, 0.0F);
  std::vector<float> posU, nrmU;
  long long uniquesN = 0, slowN = 0, uniques = 0, originMis = 0, boundsMis = 0, tiesSeen = 0, clippedUniques = 0, slow = 0;
  long long posMis[2] = {0, 0}, nrmMis[2] = {0, 0}, clipMis[2] = {0, 0};
  long long splitPosMis[2] = {0, 0}, splitNrmMis[2] = {0, 0}, splitClipMis[2] = {0, 0}, slowSplitPos = 0, slowSplitNrm = 0;
  for (int pose = 0; pose < 1500; ++pose) {
    const bool identity = pose == 0;
    const float shift = pose < 1200 ? 0.3F : 20.0F;  // the last poses throw joints past the box
    for (u32 j = 0; j < slots; ++j) {
      float* m = &palette[j * 16];
      if (identity) {
        std::memset(m, 0, 16 * sizeof(float));
        m[0] = m[5] = m[10] = m[15] = 1.0F;
      } else {
        randomSlot(rng, m, shift, 0.1F);
      }
    }
    for (Level& lv : levels) {
      const U::UniqueBind& ub = lv.ub;
      posU.assign((size_t)ub.count * 4, 0.0F);
      nrmU.assign((size_t)ub.count * 4, 0.0F);
      U::blend(ub, palette.data(), posU.data(), nrmU.data());
      // the origin: the reference scan against the extent folded in a different order
      s32 oiRef[3];
      U::poseOrigin(posU.data(), ub.count, step, oiRef);
      float mn[3] = {FLT_MAX, FLT_MAX, FLT_MAX}, mx[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
      for (u32 u = ub.count; u-- > 0;)
        for (int i = 0; i < 3; ++i) {
          const float v = posU[(size_t)u * 4 + i];
          mn[i] = v < mn[i] ? v : mn[i];
          mx[i] = v > mx[i] ? v : mx[i];
        }
      s32 oi[3];
      U::originFromExtent(mn, mx, step, oi);
      if (oi[0] != oiRef[0] || oi[1] != oiRef[1] || oi[2] != oiRef[2]) ++originMis;
      std::vector<s16> qpFus((size_t)ub.count * 4);
      std::vector<s8> qnFus((size_t)ub.count * 4);
      for (u32 u = 0; u < ub.count; ++u) {
        const float* p = &posU[(size_t)u * 4];
        const float* a = &nrmU[(size_t)u * 4];
        ++uniques;
        if (pose < 1200) ++uniquesN;
        for (int m = 0; m < 2; ++m) {
          const Round ee = m == 0 ? Round::Nearest : Round::Chop;
          s16 rp[4], fp[4];
          s8 rn[4], fn[4];
          bool wasSlow = false;
          const u32 rc = quantizeRef(ee, p, a, invStep, oiRef, rp, rn);
          const u32 fc = quantizeFused(ee, p, a, invStep, oiRef, fp, fn, wasSlow);
          if (m == 0) {
            clippedUniques += rc;
            if (wasSlow) ++slow;
            if (wasSlow && pose < 1200) ++slowN;
            std::memcpy(&qpFus[(size_t)u * 4], fp, 8);
            std::memcpy(&qnFus[(size_t)u * 4], fn, 4);
          }
          if (std::memcmp(rp, fp, 8) != 0) ++posMis[m];
          if (std::memcmp(rn, fn, 4) != 0) ++nrmMis[m];
          // a unique past the clamp is re-quantized, so its count is the reference's
          if (rc != fc) ++clipMis[m];
          s16 sp[4];
          s8 sn[4];
          bool slowP = false, slowN = false;
          const u32 sc = quantizeSplit(ee, p, a, invStep, oiRef, sp, sn, slowP, slowN);
          if (m == 0 && slowP) ++slowSplitPos;
          if (m == 0 && slowN) ++slowSplitNrm;
          if (std::memcmp(rp, sp, 8) != 0) ++splitPosMis[m];
          if (std::memcmp(rn, sn, 4) != 0) ++splitNrmMis[m];
          if (rc != sc) ++splitClipMis[m];
        }
        if (identity)
          for (int i = 0; i < 3; ++i) {
            const float x = opSub(Round::Nearest, opMul(Round::Nearest, p[i], invStep), (float)oiRef[i]);
            if (x != std::floor(x) && (x * 2.0F) == std::floor(x * 2.0F)) ++tiesSeen;
          }
      }
      // the part bounds
      std::vector<s32> loRef, hiRef;
      boundsScalar(lv.rngOff, lv.rng, ub.count, qpFus.data(), lv.ranges, loRef, hiRef);
      std::vector<s16> lo, hi;
      U::boundsReset(lv.ranges, lo, hi);
      for (u32 k = 0; k < ub.count; ++k)
        for (u32 j = lv.rngOff[k]; j < lv.rngOff[k + 1]; ++j)
          U::boundsFold(&lo[4 * (size_t)lv.rng[j]], &hi[4 * (size_t)lv.rng[j]],
                        &qpFus[(size_t)k * 4]);
      for (u32 r = 0; r < lv.ranges; ++r)
        for (int i = 0; i < 3; ++i)
          if (lo[4 * (size_t)r + i] != loRef[3 * (size_t)r + i] ||
              hi[4 * (size_t)r + i] != hiRef[3 * (size_t)r + i])
            ++boundsMis;
    }
  }
  std::printf("soldier model, every part and LOD, 1500 poses: %lld uniques (%lld re-quantized by the "
              "reference arithmetic); exact-tie coordinates in the identity pose %lld; clamped "
              "uniques %lld;\n  mismatches (EE nearest / chop): positions %lld / %lld, normals "
              "%lld / %lld, clamp counts %lld / %lld; origin %lld; part bounds %lld\n",
              uniques, slow, tiesSeen, clippedUniques, posMis[0], posMis[1], nrmMis[0], nrmMis[1],
              clipMis[0], clipMis[1], originMis, boundsMis);
  check(tiesSeen > 0, "the identity pose has exact ties (the case the rounding must keep)");
  check(clippedUniques > 0, "the throw poses clamp some uniques");
  check(posMis[0] == 0 && posMis[1] == 0, "fused positions are the reference's, byte for byte");
  check(nrmMis[0] == 0 && nrmMis[1] == 0, "fused normals are the reference's, byte for byte");
  check(clipMis[0] == 0 && clipMis[1] == 0, "the fused clamp count is the reference's");
  std::printf("  one-blend route on the soldier model: %lld position / %lld normal re-quantizations; mismatches (EE nearest / chop): "
              "positions %lld / %lld, normals %lld / %lld, clamp counts %lld / %lld\n",
              slowSplitPos, slowSplitNrm, splitPosMis[0], splitPosMis[1], splitNrmMis[0], splitNrmMis[1],
              splitClipMis[0], splitClipMis[1]);
  check(splitPosMis[0] == 0 && splitPosMis[1] == 0, "one-blend positions are the reference's, byte for byte");
  check(splitNrmMis[0] == 0 && splitNrmMis[1] == 0, "one-blend normals are the reference's, byte for byte");
  check(splitClipMis[0] == 0 && splitClipMis[1] == 0, "the one-blend clamp count is the reference's");
  check(originMis == 0, "the origin from the folded extent is poseOrigin's");
  check(boundsMis == 0, "the packed-halfword part bounds are the scalar loop's");
  std::printf("  ordinary poses (joints within 0.3 m): %lld of %lld uniques re-quantized (%.3f %%)\n",
              slowN, uniquesN, 100.0 * (double)slowN / (double)uniquesN);
  check(slowN * 100 < uniquesN, "ordinary poses re-quantize under 1 % of the uniques");

  // 3. Part bounds over random data, with ranges of any length (the lanes' pad stays 0).
  {
    std::uniform_int_distribution<int> val(-32767, 32767);
    long long mis = 0;
    for (int t = 0; t < 2000; ++t) {
      const u32 ranges = 1 + (u32)(rng() % 60), count = 1 + (u32)(rng() % 200);
      std::vector<u32> off(count + 1, 0);
      std::vector<u16> list;
      for (u32 k = 0; k < count; ++k) {
        const u32 n = 1 + (u32)(rng() % 4);
        for (u32 j = 0; j < n; ++j) list.push_back((u16)(rng() % ranges));
        off[k + 1] = (u32)list.size();
      }
      std::vector<s16> q((size_t)count * 4, 0);
      for (u32 k = 0; k < count; ++k)
        for (int i = 0; i < 3; ++i) q[(size_t)k * 4 + i] = (s16)val(rng);
      std::vector<s32> loRef, hiRef;
      boundsScalar(off, list, count, q.data(), ranges, loRef, hiRef);
      std::vector<s16> lo, hi;
      U::boundsReset(ranges, lo, hi);
      for (u32 k = 0; k < count; ++k)
        for (u32 j = off[k]; j < off[k + 1]; ++j)
          U::boundsFold(&lo[4 * (size_t)list[j]], &hi[4 * (size_t)list[j]], &q[(size_t)k * 4]);
      for (u32 r = 0; r < ranges; ++r) {
        for (int i = 0; i < 3; ++i)
          if (lo[4 * (size_t)r + i] != loRef[3 * (size_t)r + i] ||
              hi[4 * (size_t)r + i] != hiRef[3 * (size_t)r + i])
            ++mis;
        if (lo[4 * (size_t)r + 3] != 0 || hi[4 * (size_t)r + 3] != 0) ++mis;
      }
    }
    check(mis == 0, "random range lists: the lane bounds are the scalar loop's, pad lane 0");
  }

  std::printf("verify-skel-fused: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
