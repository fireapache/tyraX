// Host verifier for the compact vertex format (stapip_compact.hpp, docs/compact-vertex-format.md).
//
// It checks the words the VIF is handed - every UNPACK, STMASK and STCYCL a compact run header
// carries - against the hardware's field layout, walks every run of every attribute through
// vif1_chain_check's VIF1 model (the same decoder the chain validator uses), and pins the
// quantizers: the boxes, the seam rule (a vertex two world boxes share decodes to the same float
// in both), the model-box round trip, ST, colour and normals. The exact paths are run under every
// float rounding mode the host has, because the EE's FPU and VU0 round toward zero.
//
// Run it with `python tools/verify-tyrax2-host.py --only compact`: that runner writes the host
// `tamtypes.h` type shim and links stapip_compact.cpp. By hand, with such a shim in <shim>:
//   g++ -std=c++17 -Wall -Wextra -Werror -I<shim> -Ivendor/tyra/engine/inc
//       tools/verify-stapip-compact.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//       -o build/verify-stapip-compact
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>
#include "renderer/core/paths/path1/vif1_chain_check.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"
#include "renderer/3d/pipeline/static/core/stapip_vif_hash.hpp"

using namespace Tyra;
using namespace Tyra::Compact;

namespace {

unsigned checks = 0, failed = 0, groupChecks = 0, groupFailed = 0;
const char* group = "";

#define CHECK(x)                                                              \
  do {                                                                        \
    ++checks;                                                                 \
    if (!(x)) {                                                               \
      ++failed;                                                               \
      if (failed <= 25)                                                       \
        std::fprintf(stderr, "FAIL [%s] line %d: %s\n", group, __LINE__, #x); \
    }                                                                         \
  } while (0)

void beginGroup(const char* name) {
  group = name;
  groupChecks = checks;
  groupFailed = failed;
}
void endGroup() {
  std::printf("  %-34s %8u checks, %u failed\n", group, checks - groupChecks,
              failed - groupFailed);
}

const Attr kAttrs[] = {Attr::Position, Attr::St,       Attr::Color,
                       Attr::Normal,   Attr::EnvNormal, Attr::BbCenter,
                       Attr::BbParams, Attr::BbColor};

// The lanes a mask word fills from the ROW register (2 bits a lane, 1 = row),
// as bits x = 1, y = 2, z = 4, w = 8 - read from the first write cycle, and 0xFF
// when the four cycles' patterns differ (the format wants the same in all four).
u32 laneMask(u32 mask) {
  const u32 row0 = mask & 0xFFu;
  for (u32 cycle = 1; cycle < 4; ++cycle)
    if (((mask >> (cycle * 8)) & 0xFFu) != row0) return 0xFFu;
  u32 lanes = 0;
  for (u32 lane = 0; lane < 4; ++lane) {
    const u32 m = (row0 >> (lane * 2)) & 3u;
    if (m == 1u) lanes |= 1u << lane;
    else if (m != 0u) return 0xFFu;  // column or write-protect: never ours
  }
  return lanes;
}

// The four VIFcode words of a run header quadword.
struct Words {
  u32 w[4];
  bool operator==(const Words& o) const {
    return std::memcmp(w, o.w, sizeof(w)) == 0;
  }
};
Words headerWords(const void* qw) {
  Words out;
  std::memcpy(out.w, qw, sizeof(out.w));
  return out;
}

u32 floatBits(float f) {
  u32 u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}
bool sameBits(float a, float b) { return floatBits(a) == floatBits(b); }
bool isPowerOfTwo(float f) {
  return f > 0.0f && (floatBits(f) & 0x007FFFFFu) == 0u;
}
// A whole number of steps, exactly: origin / step is an integer.
bool onGrid(float origin, float step) {
  const float k = origin / step;
  return k == std::trunc(k) && std::fabs(k) < 16777216.0f;
}

void formatTable() {
  beginGroup("formats_and_sizes");
  CHECK(static_cast<u8>(Format::V2_16) == 0x5 && static_cast<u8>(Format::V3_16) == 0x9);
  CHECK(static_cast<u8>(Format::V4_16) == 0xD && static_cast<u8>(Format::V4_8) == 0xE);
  CHECK(kPositionFormat == Format::V3_16);
  CHECK(formatOf(Attr::Position) == kPositionFormat && formatOf(Attr::BbCenter) == kPositionFormat);
  CHECK(formatOf(Attr::St) == Format::V2_16);
  CHECK(formatOf(Attr::Color) == Format::V4_8 && formatOf(Attr::BbColor) == Format::V4_8);
  CHECK(formatOf(Attr::Normal) == Format::V4_8 && formatOf(Attr::EnvNormal) == Format::V4_8);
  CHECK(formatOf(Attr::BbParams) == Format::V4_16);
  CHECK(isSigned(Attr::Position) && isSigned(Attr::St) && isSigned(Attr::Normal));
  CHECK(isSigned(Attr::EnvNormal) && isSigned(Attr::BbCenter) && isSigned(Attr::BbParams));
  CHECK(!isSigned(Attr::Color) && !isSigned(Attr::BbColor));
  CHECK(masked(Attr::Position) && masked(Attr::BbCenter) && masked(Attr::St));
  CHECK(!masked(Attr::Color) && !masked(Attr::Normal) && !masked(Attr::EnvNormal));
  CHECK(!masked(Attr::BbParams) && !masked(Attr::BbColor));
  CHECK(elementBytes(Attr::Position) == 6 && elementBytes(Attr::St) == 4);
  CHECK(elementBytes(Attr::Color) == 4 && elementBytes(Attr::BbParams) == 8);
  CHECK(runBytes(Attr::Position, 75) == 450 && runQwords(Attr::Position, 75) == 29);
  CHECK(runQwords(Attr::St, 75) == 19 && runQwords(Attr::Color, 75) == 19 && runQwords(Attr::Normal, 75) == 19);
  CHECK(runQwords(Attr::Position, 8) == 3 && runQwords(Attr::Position, 9) == 4);
  CHECK(runQwords(Attr::St, 4) == 1 && runQwords(Attr::St, 5) == 2);
  CHECK(runQwords(Attr::BbParams, 2) == 1 && runQwords(Attr::BbParams, 3) == 2);
  for (Attr a : kAttrs)
    for (u32 n = 0; n <= 256; ++n) {
      CHECK(runBytes(a, n) == n * elementBytes(a));
      CHECK(runQwords(a, n) * 16 >= runBytes(a, n) && runQwords(a, n) * 16 < runBytes(a, n) + 16);
    }
  endGroup();
}

void unpackWords() {
  beginGroup("unpack_mask_row_words");
  u32 c = unpackCode(Attr::Position, 75, 2, true);
  CHECK((c >> 24) == 0x79 && ((c >> 16) & 255) == 75 && (c & 0x3FF) == 2);   // masked V3-16
  CHECK((c & (1u << 14)) == 0 && (c & (1u << 15)) != 0);                      // signed, TOPS
  CHECK(((unpackCode(Attr::Color, 75, 2, true) >> 24) == 0x6E) && (unpackCode(Attr::Color, 75, 2, true) & (1u << 14)));
  CHECK(unpackCode(Attr::Position, 75, 2, true) == 0x794B8002u);
  CHECK(unpackCode(Attr::Color, 75, 2, true) == 0x6E4BC002u);
  CHECK(unpackCode(Attr::St, 75, 77, true) == 0x754B804Du);
  CHECK(unpackCode(Attr::Normal, 4, 0, false) == 0x6E040000u);
  CHECK(unpackCode(Attr::BbParams, 1, 1023, true) == 0x6D0183FFu);
  CHECK((unpackCode(Attr::Position, 256, 0, true) >> 16 & 255) == 0);  // 256 is NUM 0
  CHECK((unpackCode(Attr::Position, 1, 1024 + 7, true) & 0x3FF) == 7);  // ADDR is 10 bits
  for (Attr a : kAttrs) {
    const u32 code = unpackCode(a, 75, 2, true);
    CHECK((code >> 29) == 3);                                        // UNPACK, no IRQ
    CHECK(((code >> 24) & 0xF) == static_cast<u32>(formatOf(a)));    // vn << 2 | vl
    CHECK((((code >> 28) & 1) != 0) == masked(a));                   // m
    CHECK((((code >> 14) & 1) == 0) == isSigned(a));                 // USN clear = signed
  }
  CHECK(laneMask(maskWord(Attr::Position)) == 0b1000 && laneMask(maskWord(Attr::St)) == 0b1100 && !masked(Attr::Color));
  CHECK(maskWord(Attr::Position) == 0x40404040u && maskWord(Attr::St) == 0x50505050u);
  CHECK(laneMask(maskWord(Attr::BbCenter)) == 0b1000);
  for (Attr a : kAttrs)
    if (!masked(a)) CHECK(maskWord(a) == 0);
  CHECK(kRow[0] == 0u && kRow[1] == 0u && kRow[2] == 0x3F800000u && kRow[3] == 0x3F800000u);
  float rowZ, rowW;
  std::memcpy(&rowZ, &kRow[2], 4);
  std::memcpy(&rowW, &kRow[3], 4);
  CHECK(rowZ == 1.0f && rowW == 1.0f);
  alignas(16) u32 header[4];
  runHeader(Attr::Position, 75, 2, header);
  CHECK(headerWords(header) == (Words{{0x20000000u, 0x40404040u, 0x01000101u, 0x794B8002u}}));
  runHeader(Attr::St, 75, 77, header);
  CHECK(headerWords(header) == (Words{{0x20000000u, 0x50505050u, 0x01000101u, 0x754B804Du}}));
  runHeader(Attr::Color, 75, 152, header);
  CHECK(headerWords(header) == (Words{{0u, 0u, 0x01000101u, unpackCode(Attr::Color, 75, 152, true)}}));
  for (Attr a : kAttrs) {
    runHeader(a, 33, 9, header);
    CHECK(header[3] == unpackCode(a, 33, 9, true));
    CHECK(header[2] == 0x01000101u);  // STCYCL cl=1 wl=1
    if (masked(a)) CHECK(header[0] == 0x20000000u && header[1] == maskWord(a));
    else CHECK(header[0] == 0u && header[1] == 0u);
  }
  CHECK(vifUnpackIsGeometry(unpackCode(Attr::Position, 75, 2, true)) && !vifUnpackIsGeometry(unpackCode(Attr::Position, 4, 0, false)));
  // The SDK's unpacks set USN (bit 14): that is not "use TOP", and must not read as geometry.
  CHECK(!vifUnpackIsGeometry(0x6C044000u) && vifUnpackIsGeometry(0x6C04C000u));
  CHECK(vifUnpackIsGeometry(unpackCode(Attr::Color, 75, 2, true)) && !vifUnpackIsGeometry(unpackCode(Attr::Color, 4, 0, false)));
  endGroup();
}

// Every run of every attribute, through vif1_chain_check's VIF1 model: the header
// quadword, then the payload padded with zero words (VIF NOPs) to whole quadwords. The
// data words the unpack eats must end exactly where the payload's data ends, so the
// padding and the word after the run are read as VIFcodes.
void streamWalk() {
  beginGroup("vif1_stream_walk");
  std::mt19937 rng(1129);
  for (Attr a : kAttrs)
    for (u32 n = 1; n <= 255; ++n) {
      alignas(16) u32 header[4];
      runHeader(a, n, 3, header);
      const u32 dataWords = (runBytes(a, n) + 3) / 4;
      const u32 payloadWords = runQwords(a, n) * 4;
      CHECK(payloadWords >= dataWords && payloadWords - dataWords < 4);
      std::vector<u32> payload(payloadWords, 0u);
      for (u32 i = 0; i < dataWords; ++i) payload[i] = rng();
      Vif1ChainCheck::Stream s;
      for (u32 w : header) s.word(w);
      CHECK(s.result && s.result.pending == dataWords);
      CHECK(s.result.command == (header[3] >> 24 & 0x7F));
      for (u32 i = 0; i < dataWords; ++i) s.word(payload[i]);
      CHECK(s.result && s.result.pending == 0);  // the next word is a VIFcode
      for (u32 i = dataWords; i < payloadWords; ++i) s.word(payload[i]);
      CHECK(s.result && s.result.pending == 0 && s.cl == 1 && s.wl == 1);
      s.word(0x17000000u);  // MSCNT: read as a code, not as data
      CHECK(s.result && s.result.command == 0x17);
    }
  endGroup();
}

void sceneSteps() {
  beginGroup("scene_step");
  CHECK(sceneStepFor(10.0f) == 1.0f / 1024 && sceneStepFor(65.0f) == 1.0f / 512 && sceneStepFor(130.0f) == 1.0f / 256);
  CHECK(sceneStepFor(0.0f) == 1.0f / 1024);  // the floor
  CHECK(sceneStepFor(127.9f) == 1.0f / 512);  // a 128 m cell is a 2^-9 scene
  for (float e = 0.01f; e < 5000.0f; e *= 1.37f) {
    const float s = sceneStepFor(e);
    CHECK(isPowerOfTwo(s) && s >= 1.0f / 1024);
    CHECK(e <= 65533.0f * s);  // a bag of this extent fits wherever it lies
    CHECK(s == 1.0f / 1024 || e > 65533.0f * (s * 0.5f));  // and it is the smallest
  }
  endGroup();
}

void worldBoxes() {
  beginGroup("world_box");
  std::mt19937 rng(77);
  std::uniform_real_distribution<float> where(-900.0f, 900.0f);
  const float steps[] = {1.0f / 1024, 1.0f / 512, 1.0f / 256};
  for (float s : steps)
    for (u32 trial = 0; trial < 2000; ++trial) {
      float mn[3], mx[3];
      // Inside 65 533 steps fits wherever the box lies (65 534 when it starts on the grid).
      const float extent = s * std::uniform_real_distribution<float>(0.0f, 65533.0f)(rng);
      for (u32 a = 0; a < 3; ++a) {
        mn[a] = where(rng);
        mx[a] = mn[a] + (a == trial % 3 ? extent : extent * 0.25f);
      }
      bool coarse = true;
      const Box box = worldBox(mn, mx, s, &coarse);
      CHECK(!coarse && box.step == s);
      for (u32 a = 0; a < 3; ++a) {
        CHECK(onGrid(box.origin[a], s));
        const s16 lo = quantize(mn[a], box.origin[a], box.step);
        const s16 hi = quantize(mx[a], box.origin[a], box.step);
        CHECK(lo > -32767 - 1 && hi < 32767 + 1);
        // nothing saturated: both ends decode within half a step
        CHECK(std::fabs(dequantize(lo, box.origin[a], box.step) - mn[a]) <= s * 0.5f);
        CHECK(std::fabs(dequantize(hi, box.origin[a], box.step) - mx[a]) <= s * 0.5f);
      }
    }
  // A box starting on the grid fits 65 534 steps exactly; one more step is coarse.
  {
    const float s = 1.0f / 512;
    float mn[3] = {-64.0f, 0.0f, 0.0f}, mx[3] = {-64.0f + 65534.0f * s, 1.0f, 1.0f};
    bool coarse = true;
    Box box = worldBox(mn, mx, s, &coarse);
    CHECK(!coarse && box.step == s && onGrid(box.origin[0], s));
    CHECK(quantize(mn[0], box.origin[0], s) == -32767 && quantize(mx[0], box.origin[0], s) == 32767);
    mx[0] = -64.0f + 65535.0f * s;
    box = worldBox(mn, mx, s, &coarse);
    CHECK(coarse && box.step == 2 * s && onGrid(box.origin[0], 2 * s));
    mx[0] = -64.0f + 65536.0f * 4 * s;  // past two doublings
    box = worldBox(mn, mx, s, &coarse);
    CHECK(coarse && box.step == 8 * s);
    CHECK(quantize(mx[0], box.origin[0], box.step) <= 32767 && quantize(mn[0], box.origin[0], box.step) >= -32767);
  }
  // A coarse box still keeps its origin on its own (coarser) grid and holds the bag.
  for (u32 trial = 0; trial < 500; ++trial) {
    const float s = 1.0f / 1024;
    float mn[3], mx[3];
    for (u32 a = 0; a < 3; ++a) {
      mn[a] = where(rng);
      mx[a] = mn[a] + std::uniform_real_distribution<float>(64.0f, 400.0f)(rng);
    }
    bool coarse = false;
    const Box box = worldBox(mn, mx, s, &coarse);
    CHECK(coarse && isPowerOfTwo(box.step) && box.step > s);
    for (u32 a = 0; a < 3; ++a) {
      CHECK(onGrid(box.origin[a], box.step));
      CHECK(std::fabs(dequantize(quantize(mn[a], box.origin[a], box.step), box.origin[a], box.step) - mn[a]) <= box.step * 0.5f);
      CHECK(std::fabs(dequantize(quantize(mx[a], box.origin[a], box.step), box.origin[a], box.step) - mx[a]) <= box.step * 0.5f);
    }
  }
  endGroup();
}

// The quantizer the seam rule rules out: subtract the origin, then round. Same rounding
// (half away from zero) and saturation; only the order of operations differs.
float naiveDecode(float p, float origin, float step) {
  float q = std::round((p - origin) / step);
  q = q > 32767.0f ? 32767.0f : (q < -32767.0f ? -32767.0f : q);
  return origin + step * q;
}

// Review Focus 1: a vertex two world boxes share decodes to the same float in both.
// The edges are random; the vertices on them include the hard cases for a quantizer
// that subtracts the origin first - a coordinate a hair below the first half step in
// boxes whose origins are metres away, an exact half step, and one an ulp from it.
// `naiveBad` counts the same vertices through naiveDecode, to show the cases bite.
u32 seamMismatches(u32 seed, u32 edges, u32* naiveBad = nullptr) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  const float steps[] = {1.0f / 1024, 1.0f / 512, 1.0f / 256};
  u32 bad = 0;
  for (u32 e = 0; e < edges; ++e) {
    const float s = steps[e % 3];
    const float cap = 60000.0f * s;  // every box fits its step
    // two boxes that meet at x = edgeX; on y and z both hold 0, from different mins
    const float edgeX = (unit(rng) - 0.5f) * 400.0f;
    float mnA[3], mxA[3], mnB[3], mxB[3];
    mnA[0] = edgeX - cap * (0.1f + 0.85f * unit(rng)); mxA[0] = edgeX;
    mnB[0] = edgeX; mxB[0] = edgeX + cap * (0.1f + 0.85f * unit(rng));
    for (u32 a = 1; a < 3; ++a) {
      mnA[a] = -cap * (0.5f + 0.45f * unit(rng)); mxA[a] = mnA[a] + cap * 0.95f;
      mnB[a] = -cap * (0.5f + 0.45f * unit(rng)); mxB[a] = mnB[a] + cap * 0.95f;
    }
    bool coarseA = true, coarseB = true;
    const Box A = worldBox(mnA, mxA, s, &coarseA);
    const Box B = worldBox(mnB, mxB, s, &coarseB);
    if (coarseA || coarseB) { ++bad; continue; }
    float p[3];
    p[0] = edgeX;  // on the shared face: the extreme of both boxes
    for (u32 a = 1; a < 3; ++a) {
      const float lo = (mnA[a] > mnB[a] ? mnA[a] : mnB[a]) + 2.0f * s;
      const float hi = (mxA[a] < mxB[a] ? mxA[a] : mxB[a]) - 2.0f * s;
      p[a] = lo + (hi - lo) * unit(rng);
    }
    const float sign = (rng() & 1) ? 1.0f : -1.0f;
    switch (e % 4) {
      case 1:  // a hair below the first half step, the origin metres away
        p[1] = sign * (0.5f * s - std::ldexp(0.5f * s, -8 - static_cast<int>(rng() % 23)));
        break;
      case 2:  // an ulp either side of a half step
        p[2] = (std::round(p[2] / s) + 0.5f) * s;
        p[2] = std::nextafter(p[2], sign * 1e9f);
        break;
      case 3:  // an exact half step
        p[1] = (std::round(p[1] / s) + 0.5f) * s;
        break;
      default: break;
    }
    for (u32 a = 0; a < 3; ++a) {
      const float da = dequantize(quantize(p[a], A.origin[a], A.step), A.origin[a], A.step);
      const float db = dequantize(quantize(p[a], B.origin[a], B.step), B.origin[a], B.step);
      if (!sameBits(da, db) || std::fabs(da - p[a]) > s * 0.5f) ++bad;
      if (naiveBad) {
        const float na = naiveDecode(p[a], A.origin[a], A.step);
        const float nb = naiveDecode(p[a], B.origin[a], B.step);
        if (!sameBits(na, nb) || std::fabs(na - p[a]) > s * 0.5f) ++*naiveBad;
      }
    }
  }
  return bad;
}

void seams() {
  beginGroup("seams_shared_edges");
  u32 naive = 0;
  CHECK(seamMismatches(4242, 10000, &naive) == 0);
  // The hard cases are real: subtracting first gets some of these vertices wrong (most
  // of them in the quarter of the edges built a hair below a half step).
  std::printf("    (subtract-then-round: %u of the 30 000 coordinates wrong)\n", naive);
  CHECK(naive > 0);
  // The case that breaks round((p - origin) / step): p = 2^-10 - 2^-34 in a box at -32 m
  // and in one at 0 m. On the grid both read 0.
  const float s = 1.0f / 512, p = std::ldexp(1.0f, -10) - std::ldexp(1.0f, -34);
  CHECK(quantize(p, -32.0f, s) == 16384 && quantize(p, 0.0f, s) == 0);
  CHECK(sameBits(dequantize(quantize(p, -32.0f, s), -32.0f, s), dequantize(quantize(p, 0.0f, s), 0.0f, s)));
  endGroup();
}

void modelBoxes() {
  beginGroup("model_box");
  {
    const float mn[3] = {-0.4f, 0.0f, -0.3f}, mx[3] = {0.4f, 2.0f, 0.3f};  // a 2 m body
    const Box box = modelBox(mn, mx);
    CHECK(box.step <= 3.06e-5f && box.step >= 2.0f / 65534.0f);
    std::mt19937 rng(5);
    float worst = 0.0f;
    for (u32 i = 0; i < 20000; ++i)
      for (u32 a = 0; a < 3; ++a) {
        const float v = mn[a] + (mx[a] - mn[a]) * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
        const float d = std::fabs(dequantize(quantize(v, box.origin[a], box.step), box.origin[a], box.step) - v);
        if (d > worst) worst = d;
      }
    for (u32 a = 0; a < 3; ++a)
      for (float v : {mn[a], mx[a]}) {
        const s16 q = quantize(v, box.origin[a], box.step);
        CHECK(q >= -32767 && q <= 32767);
        const float d = std::fabs(dequantize(q, box.origin[a], box.step) - v);
        if (d > worst) worst = d;
      }
    CHECK(worst <= box.step * 0.5f + 1e-6f);
  }
  // Boxes of every size and place: the corners never saturate, the round trip holds.
  std::mt19937 rng(91);
  for (u32 trial = 0; trial < 3000; ++trial) {
    const float size = std::ldexp(1.0f, static_cast<int>(rng() % 16) - 6);  // 1/64 m .. 512 m
    float mn[3], mx[3];
    for (u32 a = 0; a < 3; ++a) {
      mn[a] = (std::uniform_real_distribution<float>(-1.0f, 1.0f)(rng)) * size * 2.0f;
      mx[a] = mn[a] + size * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
    }
    const Box box = modelBox(mn, mx);
    CHECK(box.step > 0.0f && box.step <= 1.0001f * size / 65534.0f);
    for (u32 a = 0; a < 3; ++a)
      for (float v : {mn[a], mx[a], 0.5f * (mn[a] + mx[a])}) {
        const float d = std::fabs(dequantize(quantize(v, box.origin[a], box.step), box.origin[a], box.step) - v);
        CHECK(d <= box.step * 0.5f + 4.0f * std::ldexp(1.0f, -23) * (std::fabs(v) + 1.0f));
      }
  }
  {
    const float p[3] = {3.0f, 3.0f, 3.0f};  // a point: still a usable box
    const Box box = modelBox(p, p);
    CHECK(box.step > 0.0f && quantize(3.0f, box.origin[0], box.step) == quantize(3.0f, box.origin[1], box.step));
    CHECK(std::fabs(dequantize(quantize(3.0f, box.origin[2], box.step), box.origin[2], box.step) - 3.0f) <= box.step);
  }
  endGroup();
}

void dequantMatrix() {
  beginGroup("dequant_matrix");
  const Box box = {{-12.5f, 3.0f, 640.0f}, 1.0f / 512};
  float m[16];
  dequant(box, m);
  const float expect[16] = {box.step, 0, 0, 0, 0, box.step, 0, 0,
                            0, 0, box.step, 0, -12.5f, 3.0f, 640.0f, 1.0f};
  for (u32 i = 0; i < 16; ++i) CHECK(sameBits(m[i], expect[i]));
  // column-major: m x (q, 1) = origin + step * q, exactly the decode
  const s16 q[3] = {-32767, 1234, 32767};
  for (u32 r = 0; r < 3; ++r) {
    const float out = m[r] * q[0] + m[4 + r] * q[1] + m[8 + r] * q[2] + m[12 + r];
    CHECK(sameBits(out, dequantize(q[r], box.origin[r], box.step)));
  }
  endGroup();
}

void quantizers() {
  beginGroup("st_colour_normal");
  CHECK(quantizeSt(1.0f) == 4096 && quantizeSt(-8.0f) == -32767 && quantizeSt(9.0f) == 32767);   // symmetric saturation
  CHECK(quantizeSt(0.0f) == 0 && quantizeSt(-1.0f) == -4096);
  CHECK(quantizeSt(0.5f / 4096) == 1 && quantizeSt(-0.5f / 4096) == -1);  // half away from zero
  CHECK(quantizeSt(32767.0f / 4096) == 32767 && quantizeSt(-32767.0f / 4096) == -32767);
  // Review Focus 2: a package at u in [100.2, 101.7] stores u - 100, its fractions intact.
  {
    const float uv[2] = {100.2f, 101.7f};
    const float shift = stRebase(uv, 2);
    CHECK(shift == 100.0f);
    for (float u : uv) {
      const float r = u - shift;
      CHECK(r >= -8.0f && r < 8.0f);
      CHECK(static_cast<s32>(quantizeSt(r)) == static_cast<s32>(std::lround(static_cast<double>(u) * 4096.0)) - 409600);
    }
  }
  {
    const float inside[4] = {-8.0f, -3.25f, 0.0f, 7.99f};
    CHECK(stRebase(inside, 4) == 0.0f);
    const float negative[3] = {-20.3f, -19.1f, -19.9f};
    const float shift = stRebase(negative, 3);
    CHECK(shift == std::trunc(shift));
    for (float u : negative) CHECK(u - shift >= -8.0f && u - shift < 8.0f);
    const float wide[2] = {-5.0f, 9.5f};  // 14.5 repeats: rebased into range
    const float w = stRebase(wide, 2);
    CHECK(w == std::trunc(w) && wide[0] - w >= -8.0f && wide[1] - w < 8.0f);
    CHECK(stRebase(wide, 0) == 0.0f);
  }
  std::mt19937 rng(3);
  for (u32 i = 0; i < 2000; ++i) {  // rebased packages anywhere in +-1000 repeats
    const float base = std::uniform_real_distribution<float>(-1000.0f, 1000.0f)(rng);
    float uv[6];
    for (float& u : uv) u = base + std::uniform_real_distribution<float>(0.0f, 13.0f)(rng);
    const float shift = stRebase(uv, 6);
    CHECK(shift == std::trunc(shift));
    for (float u : uv) CHECK(u - shift >= -8.0f && u - shift < 8.0f);
  }
  // colour: 0..255 exact, 300 -> 255, -3 -> 0
  for (u32 v = 0; v <= 255; ++v) CHECK(quantizeColor(static_cast<float>(v)) == v);
  CHECK(quantizeColor(300.0f) == 255 && quantizeColor(-3.0f) == 0 && quantizeColor(127.5f) == 128);
  CHECK(quantizeColor(254.6f) == 255 && quantizeColor(0.4f) == 0 && quantizeColor(NAN) == 0);
  // normal: |n - q / 127| <= 1 / 254
  for (float n = -1.0f; n <= 1.0f; n += 1.0f / 1024) {
    const s8 q = quantizeNormal(n);
    CHECK(std::fabs(n - q / 127.0f) <= 1.0f / 254 + 1e-7f);
  }
  CHECK(quantizeNormal(1.0f) == 127 && quantizeNormal(-1.0f) == -127 && quantizeNormal(0.0f) == 0);
  CHECK(quantizeNormal(2.0f) == 127 && quantizeNormal(-2.0f) == -127 && quantizeNormal(0.5f) == 64);
  endGroup();
}

void quantizeRules() {
  beginGroup("quantize_rounding");
  const float s = 1.0f / 512;
  CHECK(quantize(0.0f, 0.0f, s) == 0);
  CHECK(quantize(s * 0.5f, 0.0f, s) == 1 && quantize(-s * 0.5f, 0.0f, s) == -1);  // half away from zero
  CHECK(quantize(s * 1.5f, 0.0f, s) == 2 && quantize(-s * 1.5f, 0.0f, s) == -2);
  // x = 0.5 - 2^-25: floor(x + 0.5) rounds to 1 under round-to-nearest; this reads 0
  CHECK(quantize((0.5f - std::ldexp(1.0f, -25)) * s, 0.0f, s) == 0);
  CHECK(quantize(100.0f, 0.0f, s) == 32767 && quantize(-100.0f, 0.0f, s) == -32767);  // saturate
  CHECK(quantize(64.0f, 0.0f, s) == 32767 && quantize(-64.0f, 0.0f, s) == -32767);
  CHECK(quantize(64.0f - s, 0.0f, s) == 32767 && quantize(-64.0f + s, 0.0f, s) == -32767);
  CHECK(quantize(1e30f, 0.0f, s) == 32767 && quantize(-1e30f, 0.0f, s) == -32767);
  CHECK(quantize(NAN, 0.0f, s) == 0);
  CHECK(dequantize(32767, 0.0f, s) == 32767.0f * s && dequantize(-5, 10.0f, s) == 10.0f - 5.0f * s);
  endGroup();
}

// The exact paths read the same under every rounding mode: the EE's FPU and VU0 round
// toward zero, the host to nearest. A mode the host cannot select is skipped (and said).
void roundingModes() {
  beginGroup("every_rounding_mode");
  const int modes[] = {FE_TOWARDZERO, FE_DOWNWARD, FE_UPWARD, FE_TONEAREST};
  const char* names[] = {"toward zero", "downward", "upward", "to nearest"};
  for (u32 m = 0; m < 4; ++m) {
    if (std::fesetround(modes[m]) != 0) {
      std::printf("    (rounding mode %s not available on this host)\n", names[m]);
      continue;
    }
    volatile float third = 1.0f, three = 3.0f;
    third = third / three;
    // the mode really changed the arithmetic (to nearest and upward agree on 1/3)
    if (modes[m] == FE_TOWARDZERO || modes[m] == FE_DOWNWARD)
      CHECK(floatBits(third) == 0x3EAAAAAAu);
    else
      CHECK(floatBits(third) == 0x3EAAAAABu);
    CHECK(selfTest() == 0);
    CHECK(seamMismatches(17 + m, 2500) == 0);
  }
  std::fesetround(FE_TONEAREST);
  endGroup();
}

}  // namespace

int main() {
  std::printf("verify-stapip-compact\n");
  formatTable();
  unpackWords();
  streamWalk();
  sceneSteps();
  worldBoxes();
  seams();
  modelBoxes();
  dequantMatrix();
  quantizers();
  quantizeRules();
  roundingModes();
  beginGroup("self_test");
  CHECK(selfTest() == 0);
  endGroup();
  if (failed != 0) {
    std::printf("verify-stapip-compact: %u checks, %u FAILED\n", checks, failed);
    return 1;
  }
  std::printf("verify-stapip-compact: ALL %u CHECKS PASSED\n", checks);
  return 0;
}
