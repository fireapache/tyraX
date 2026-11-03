// Host verifier for the compact vertex format (stapip_compact.hpp, stapip_compact_stream.hpp,
// docs/compact-vertex-format.md).
//
// It checks the words the VIF is handed - a bag's setup (STCYCL, STMOD, STROW, STCOL) and every
// STMASK, STMOD and UNPACK a compact run header carries - against the hardware's field layout,
// walks every run of every attribute through vif1_chain_check's VIF1 model (the same decoder the
// chain validator uses), and pins the quantizers: the boxes, the seam rule (a vertex two world
// boxes share reaches the VU as the same grid index, under the same matrix - the same clip-space
// position, bit for bit), the model-box round trip, ST, colour and normals. The exact paths are
// run under every float rounding mode the host has, because the EE's FPU and VU0 round toward zero.
// Then the stream: every layout written through the writer and packed from float arrays (the same
// bytes), its sizes, run headers and VIF1 walk, what the view decodes, the part boxes, the batch
// paths, ST rebasing and clamping, and allocation that is refused (nothing leaks).
//
// Run it with `python tools/verify-tyrax2-host.py --only compact`: that runner writes the host
// `tamtypes.h` type shim and links the three compact sources. By hand, with such a shim in <shim>:
//   g++ -std=c++17 -Wall -Wextra -Werror -I<shim> -Ivendor/tyra/engine/inc
//       tools/verify-stapip-compact.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact_stream.cpp
//       vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/stapip_compact_ee.cpp
//       -o build/verify-stapip-compact
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <random>
#include <utility>
#include <vector>
#include "renderer/core/paths/path1/vif1_chain_check.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"
#include "renderer/3d/pipeline/static/core/programs/stapip_vu1_shared_defines.h"
#include "renderer/3d/pipeline/static/core/stapip_vif_hash.hpp"

// Array allocations are counted, and the nothrow form can be told to refuse: the stream takes its
// block with `new (std::nothrow) u8[]`, so a refused allocate() must return false and leave the
// count where it was.
namespace {
long liveArrays = 0;
long refuseIn = -1;  // the nothrow new[] that many calls from now answers null; -1 never
}  // namespace

void* operator new[](std::size_t size) {
  void* p = std::malloc(size ? size : 1);
  if (p == nullptr) throw std::bad_alloc();
  ++liveArrays;
  return p;
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  if (refuseIn == 0) {
    refuseIn = -1;
    return nullptr;
  }
  if (refuseIn > 0) --refuseIn;
  void* p = std::malloc(size ? size : 1);
  if (p != nullptr) ++liveArrays;
  return p;
}
void operator delete[](void* p) noexcept {
  if (p == nullptr) return;
  --liveArrays;
  std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept { operator delete[](p); }

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

// An ST value quantizes without saturating: |x x 4096| rounds inside +-32767.
bool stFitsExact(float x) { return x * 4096.0f > -32767.5f && x * 4096.0f < 32767.5f; }

// Does ANY whole-repeat shift bring these values inside - every integer near them tried, not
// stRebase's three candidates.
bool anyShiftFits(const float* uv, u32 n, u32 stride) {
  float lo = uv[0], hi = uv[0];
  for (u32 i = 1; i < n; ++i) {
    lo = uv[i * stride] < lo ? uv[i * stride] : lo;
    hi = uv[i * stride] > hi ? uv[i * stride] : hi;
  }
  for (float c = std::floor(hi) - 10.0f; c <= std::ceil(lo) + 10.0f; c += 1.0f)
    if (stFitsExact(lo - c) && stFitsExact(hi - c)) return true;
  return false;
}

// stRebase's shift brings every value inside.
bool rebasedFits(const float* uv, u32 n, u32 stride) {
  const float shift = stRebase(uv, n, stride);
  for (u32 i = 0; i < n; ++i)
    if (!stFitsExact(uv[i * stride] - shift)) return false;
  return true;
}

const Attr kAttrs[] = {Attr::Position, Attr::St,       Attr::Color,
                       Attr::Normal,   Attr::EnvNormal, Attr::BbCenter,
                       Attr::BbParams, Attr::BbColor};

// The lanes a mask word fills from the COL register (2 bits a lane, 2 = column),
// as bits x = 1, y = 2, z = 4, w = 8 - read from the first write cycle, and 0xFF
// when the four cycles' patterns differ (the format wants the same in all four).
u32 laneMask(u32 mask) {
  const u32 row0 = mask & 0xFFu;
  for (u32 cycle = 1; cycle < 4; ++cycle)
    if (((mask >> (cycle * 8)) & 0xFFu) != row0) return 0xFFu;
  u32 lanes = 0;
  for (u32 lane = 0; lane < 4; ++lane) {
    const u32 m = (row0 >> (lane * 2)) & 3u;
    if (m == 2u) lanes |= 1u << lane;
    else if (m != 0u) return 0xFFu;  // row or write-protect: never ours
  }
  return lanes;
}

// What one position lane is in VU memory: the int16 sign-extended, plus the ROW word (offset
// MODE) - a 32-bit integer add, as the VIF does it.
s32 vifLane(s16 q, u32 row) {
  return static_cast<s32>(static_cast<u32>(static_cast<s32>(q)) + row);
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
  CHECK(formatOf(Attr::St) == Format::V3_16);  // S, T, Q (ruling P16)
  CHECK(formatOf(Attr::Color) == Format::V4_8 && formatOf(Attr::BbColor) == Format::V4_8);
  CHECK(formatOf(Attr::Normal) == Format::V4_8 && formatOf(Attr::EnvNormal) == Format::V4_8);
  CHECK(formatOf(Attr::BbParams) == Format::V4_16);
  CHECK(isSigned(Attr::Position) && isSigned(Attr::St) && isSigned(Attr::Normal));
  CHECK(isSigned(Attr::EnvNormal) && isSigned(Attr::BbCenter) && isSigned(Attr::BbParams));
  CHECK(!isSigned(Attr::Color) && !isSigned(Attr::BbColor));
  CHECK(masked(Attr::Position) && masked(Attr::BbCenter) && masked(Attr::St));
  CHECK(!masked(Attr::Color) && !masked(Attr::Normal) && !masked(Attr::EnvNormal));
  CHECK(!masked(Attr::BbParams) && !masked(Attr::BbColor));
  CHECK(elementBytes(Attr::Position) == 6 && elementBytes(Attr::St) == 6);
  CHECK(elementBytes(Attr::Color) == 4 && elementBytes(Attr::BbParams) == 8);
  CHECK(runBytes(Attr::Position, 75) == 450 && runQwords(Attr::Position, 75) == 29);
  CHECK(runQwords(Attr::St, 75) == 29 && runQwords(Attr::Color, 75) == 19 && runQwords(Attr::Normal, 75) == 19);
  CHECK(runQwords(Attr::Position, 8) == 3 && runQwords(Attr::Position, 9) == 4);
  CHECK(runQwords(Attr::St, 2) == 1 && runQwords(Attr::St, 3) == 2 && runQwords(Attr::St, 8) == 3);
  CHECK(runQwords(Attr::BbParams, 2) == 1 && runQwords(Attr::BbParams, 3) == 2);
  for (Attr a : kAttrs)
    for (u32 n = 0; n <= 256; ++n) {
      CHECK(runBytes(a, n) == n * elementBytes(a));
      CHECK(runQwords(a, n) * 16 >= runBytes(a, n) && runQwords(a, n) * 16 < runBytes(a, n) + 16);
    }
  endGroup();
}

void unpackWords() {
  beginGroup("unpack_mask_setup_words");
  u32 c = unpackCode(Attr::Position, 75, 2, true);
  CHECK((c >> 24) == 0x79 && ((c >> 16) & 255) == 75 && (c & 0x3FF) == 2);   // masked V3-16
  CHECK((c & (1u << 14)) == 0 && (c & (1u << 15)) != 0);                      // signed, TOPS
  CHECK(((unpackCode(Attr::Color, 75, 2, true) >> 24) == 0x6E) && (unpackCode(Attr::Color, 75, 2, true) & (1u << 14)));
  CHECK(unpackCode(Attr::Position, 75, 2, true) == 0x794B8002u);
  CHECK(unpackCode(Attr::Color, 75, 2, true) == 0x6E4BC002u);
  CHECK(unpackCode(Attr::St, 75, 77, true) == 0x794B804Du);  // masked V3-16, like a position
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
  CHECK(laneMask(maskWord(Attr::Position)) == 0b1000 && laneMask(maskWord(Attr::St)) == 0b1000 && !masked(Attr::Color));
  CHECK(maskWord(Attr::Position) == 0x80808080u && maskWord(Attr::St) == 0x80808080u);  // Q is data now
  CHECK(laneMask(maskWord(Attr::BbCenter)) == 0b1000);
  for (Attr a : kAttrs)
    if (!masked(a)) CHECK(maskWord(a) == 0);
  // Offset MODE on positions (and billboard centres) only.
  CHECK(offset(Attr::Position) && offset(Attr::BbCenter));
  for (Attr a : kAttrs)
    if (a != Attr::Position && a != Attr::BbCenter) CHECK(!offset(a));
  for (u32 i = 0; i < 4; ++i) {
    float col;
    std::memcpy(&col, &kCol[i], 4);
    CHECK(kCol[i] == 0x3F800000u && col == 1.0f);
  }
  alignas(16) u32 header[4];
  runHeader(Attr::Position, 75, 2, header);
  CHECK(headerWords(header) == (Words{{0x20000000u, 0x80808080u, 0x05000001u, 0x794B8002u}}));
  runHeader(Attr::St, 75, 77, header);
  CHECK(headerWords(header) == (Words{{0x20000000u, 0x80808080u, 0x05000000u, 0x794B804Du}}));
  runHeader(Attr::Color, 75, 152, header);
  CHECK(headerWords(header) == (Words{{0u, 0u, 0x05000000u, unpackCode(Attr::Color, 75, 152, true)}}));
  for (Attr a : kAttrs) {
    runHeader(a, 33, 9, header);
    CHECK(header[3] == unpackCode(a, 33, 9, true));
    CHECK(header[2] == (offset(a) ? 0x05000001u : 0x05000000u));  // STMOD offset / normal
    if (masked(a)) CHECK(header[0] == 0x20000000u && header[1] == maskWord(a));
    else CHECK(header[0] == 0u && header[1] == 0u);
  }
  // A bag's setup: STCYCL 1/1, STMOD normal, STROW = the origin's grid index, STCOL = 1.0f.
  {
    const Box box = {{-12.5f, 3.0f, 640.0f}, 1.0f / 512};
    u32 row[4];
    rowFor(box, row);
    CHECK(row[0] == static_cast<u32>(-6400) && row[1] == 1536u && row[2] == 327680u && row[3] == 0u);
    u32 setup[kBagSetupWords];
    bagSetup(box, setup);
    const u32 want[kBagSetupWords] = {0x01000101u, 0x05000000u, 0x30000000u, row[0], row[1], row[2], 0u,
                                      0x31000000u, 0x3F800000u, 0x3F800000u, 0x3F800000u, 0x3F800000u};
    CHECK(std::memcmp(setup, want, sizeof(want)) == 0);
    CHECK(kBagSetupWords % 4 == 0);  // whole quadwords
    // Offset MODE gives the VU the vertex's grid index: round(v / step).
    const float v = -12.5f + 1000.0f / 512;
    CHECK(vifLane(quantize(v, box.origin[0], box.step), row[0]) == -6400 + 1000);
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
      const Box box = {{-3.0f, 1.0f, 0.5f}, 1.0f / 1024};
      u32 setup[kBagSetupWords];
      bagSetup(box, setup);
      for (u32 w : setup) s.word(w);
      CHECK(s.result && s.result.pending == 0 && s.cl == 1 && s.wl == 1);
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

// A view-projection over the seam probe's world (+-400 m), column-major like M4x4.
void sceneMvp(float out[16]) {
  const double yaw = 0.52, pitch = -0.35, fy = 1.0 / std::tan(0.5), fx = fy * 0.75;
  const double n = 0.1, f = 1000.0;
  const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
  // view = R(pitch) R(yaw) T(-eye), rows
  const double r[3][3] = {{cy, 0.0, -sy}, {sy * sp, cp, cy * sp}, {sy * cp, -sp, cy * cp}};
  const double eye[3] = {-10.0, 30.0, -50.0};
  double v[4][4] = {};
  for (u32 i = 0; i < 3; ++i) {
    for (u32 k = 0; k < 3; ++k) v[i][k] = r[i][k];
    v[i][3] = -(r[i][0] * eye[0] + r[i][1] * eye[1] + r[i][2] * eye[2]);
  }
  v[3][3] = 1.0;
  const double pm[4][4] = {{fx, 0, 0, 0}, {0, fy, 0, 0},
                           {0, 0, (f + n) / (n - f), 2 * f * n / (n - f)}, {0, 0, -1, 0}};
  for (u32 row = 0; row < 4; ++row)
    for (u32 col = 0; col < 4; ++col) {
      double acc = 0.0;
      for (u32 k = 0; k < 4; ++k) acc += pm[row][k] * v[k][col];
      out[col * 4 + row] = static_cast<float>(acc);
    }
}

// MatrixMultiplyVertex in float, the VU's order: acc = c0 x; acc += c1 y; acc += c2 z;
// out = acc + c3 w (one rounding a product, one a sum).
void vuTransform(const float m[16], const float in[4], float out[4]) {
  for (u32 row = 0; row < 4; ++row) {
    float acc = m[row] * in[0];
    acc = acc + m[4 + row] * in[1];
    acc = acc + m[8 + row] * in[2];
    out[row] = acc + m[12 + row] * in[3];
  }
}

// a x b, column-major, in float.
void mul4(const float a[16], const float b[16], float out[16]) {
  for (u32 col = 0; col < 4; ++col)
    for (u32 row = 0; row < 4; ++row) {
      float acc = 0.0f;
      for (u32 k = 0; k < 4; ++k) acc = acc + a[k * 4 + row] * b[col * 4 + k];
      out[col * 4 + row] = acc;
    }
}

// Review Focus 1, where it bites: on the VU. Each shared vertex goes through what a draw does -
// quantize in its bag's box, the VIF's offset add of the bag's ROW (its origin index), ITOF0,
// then MatrixMultiplyVertex - under the ONE scene matrix MVP x D(step, 0), and the two bags'
// clip-space positions must match bit for bit. `perBagBad` counts the scheme this replaced: the
// box-local integer under a per-bag MVP x D(step, origin), whose translation rounds differently
// per bag.
u32 clipSeamMismatches(u32 seed, u32 edges, u32* perBagBad) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  const float steps[] = {1.0f / 1024, 1.0f / 512, 1.0f / 256};
  float mvp[16];
  sceneMvp(mvp);
  u32 bad = 0;
  for (u32 e = 0; e < edges; ++e) {
    const float s = steps[e % 3];
    const float cap = 60000.0f * s;
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
    p[0] = edgeX;
    for (u32 a = 1; a < 3; ++a) {
      const float lo = (mnA[a] > mnB[a] ? mnA[a] : mnB[a]) + 2.0f * s;
      const float hi = (mxA[a] < mxB[a] ? mxA[a] : mxB[a]) - 2.0f * s;
      p[a] = lo + (hi - lo) * unit(rng);
    }
    float d[16], scene[16];
    dequant(s, d);
    mul4(mvp, d, scene);  // the one matrix every world bag of the scene uploads
    u32 rowA[4], rowB[4];
    rowFor(A, rowA);
    rowFor(B, rowB);
    float inA[4], inB[4], localA[4], localB[4];
    for (u32 a = 0; a < 3; ++a) {
      const s16 qa = quantize(p[a], A.origin[a], A.step), qb = quantize(p[a], B.origin[a], B.step);
      inA[a] = static_cast<float>(vifLane(qa, rowA[a]));  // ITOF0
      inB[a] = static_cast<float>(vifLane(qb, rowB[a]));
      localA[a] = static_cast<float>(qa);
      localB[a] = static_cast<float>(qb);
    }
    inA[3] = inB[3] = localA[3] = localB[3] = 1.0f;  // w from COL
    float clipA[4], clipB[4];
    vuTransform(scene, inA, clipA);
    vuTransform(scene, inB, clipB);
    if (std::memcmp(clipA, clipB, sizeof(clipA)) != 0) ++bad;
    if (perBagBad) {
      float dA[16], dB[16], mA[16], mB[16];
      dequant(s, dA);
      dequant(s, dB);
      for (u32 a = 0; a < 3; ++a) {
        dA[12 + a] = A.origin[a];
        dB[12 + a] = B.origin[a];
      }
      mul4(mvp, dA, mA);
      mul4(mvp, dB, mB);
      vuTransform(mA, localA, clipA);
      vuTransform(mB, localB, clipB);
      if (std::memcmp(clipA, clipB, sizeof(clipA)) != 0) ++*perBagBad;
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
  // On the VU: the same grid index under the same matrix - no clip-space difference at all.
  u32 perBag = 0;
  CHECK(clipSeamMismatches(4243, 20000, &perBag) == 0);
  std::printf("    (per-bag MVP x D(origin): %u of 20 000 shared vertices' clip-space bits differ)\n", perBag);
  CHECK(perBag > 0);
  endGroup();
}

void modelBoxes() {
  beginGroup("model_box");
  {
    const float mn[3] = {-0.4f, 0.0f, -0.3f}, mx[3] = {0.4f, 2.0f, 0.3f};  // a 2 m body
    const Box box = modelBox(mn, mx);
    CHECK(box.step == 1.0f / 16384);  // the power of two at or above 2 / 65 534
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
    CHECK(worst <= box.step * 0.5f);  // a power-of-two step: every operation is exact
  }
  // Boxes of every size and place: the corners never saturate, the round trip holds.
  // The step is a power of two at or above extent / 65 534 (twice that when both ends'
  // grid points are exact half steps either side of 0), the round trip is exact to half a step,
  // and every grid index is an exact float.
  std::mt19937 rng(91);
  for (u32 trial = 0; trial < 3000; ++trial) {
    const float size = std::ldexp(1.0f, static_cast<int>(rng() % 16) - 6);  // 1/64 m .. 512 m
    float mn[3], mx[3];
    for (u32 a = 0; a < 3; ++a) {
      mn[a] = (std::uniform_real_distribution<float>(-1.0f, 1.0f)(rng)) * size * 2.0f;
      mx[a] = mn[a] + size * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
    }
    const Box box = modelBox(mn, mx);
    float extent = 0.0f;
    for (u32 a = 0; a < 3; ++a) extent = std::fmax(extent, mx[a] - mn[a]);
    const float wanted = extent / 65534.0f;
    CHECK(isPowerOfTwo(box.step) && box.step >= wanted && box.step < 4.0f * wanted);
    u32 row[4];
    rowFor(box, row);
    for (u32 a = 0; a < 3; ++a) {
      CHECK(onGrid(box.origin[a], box.step));
      for (float v : {mn[a], mx[a], 0.5f * (mn[a] + mx[a])}) {
        const s16 q = quantize(v, box.origin[a], box.step);
        CHECK(q >= -32767 && q <= 32767);
        const float d = std::fabs(dequantize(q, box.origin[a], box.step) - v);
        CHECK(d <= box.step * 0.5f);
        const s32 index = vifLane(q, row[a]);
        CHECK(std::abs(index) < 16777216 && sameBits(static_cast<float>(index) * box.step,
                                                     dequantize(q, box.origin[a], box.step)));
      }
    }
  }
  // A model far from its own origin coarsens until its indices are exact floats.
  {
    const float mn[3] = {5000.0f, 0.0f, 0.0f}, mx[3] = {5000.5f, 0.5f, 0.5f};
    const Box box = modelBox(mn, mx);
    CHECK(isPowerOfTwo(box.step) && 5000.5f / box.step < 16777216.0f);
    CHECK(box.step == 1.0f / 2048);  // 2^-16 holds the extent, but 5000 m is 2^28 of it; 2^-11 is the first under 2^24
  }
  {
    const float p[3] = {3.0f, 3.0f, 3.0f};  // a point: still a usable box
    const Box box = modelBox(p, p);
    CHECK(isPowerOfTwo(box.step) && quantize(3.0f, box.origin[0], box.step) == quantize(3.0f, box.origin[1], box.step));
    CHECK(dequantize(quantize(3.0f, box.origin[2], box.step), box.origin[2], box.step) == 3.0f);
    CHECK(3.0f / box.step < 16777216.0f);
  }
  endGroup();
}

void dequantMatrix() {
  beginGroup("dequant_matrix");
  const Box box = {{-12.5f, 3.0f, 640.0f}, 1.0f / 512};
  float m[16];
  dequant(box.step, m);
  const float expect[16] = {box.step, 0, 0, 0, 0, box.step, 0, 0,
                            0, 0, box.step, 0, 0, 0, 0, 1.0f};
  for (u32 i = 0; i < 16; ++i) CHECK(sameBits(m[i], expect[i]));
  // column-major: m x (index, 1) = step * index - the grid index the VIF hands the VU (the
  // local integer plus ROW) decodes to exactly the EE's origin + step * q
  const s16 q[3] = {-32767, 1234, 32767};
  u32 row[4];
  rowFor(box, row);
  float in[4];
  for (u32 r = 0; r < 3; ++r) in[r] = static_cast<float>(vifLane(q[r], row[r]));
  in[3] = 1.0f;
  float out[4];
  vuTransform(m, in, out);
  for (u32 r = 0; r < 3; ++r) CHECK(sameBits(out[r], dequantize(q[r], box.origin[r], box.step)));
  CHECK(out[3] == 1.0f);
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
    const float inside[4] = {-7.99f, -3.25f, 0.0f, 7.99f};
    CHECK(stRebase(inside, 4) == 0.0f);
    // -8 itself saturates (to -32767), so a package reaching it is moved.
    const float edge[2] = {-8.0f, 0.0f};
    CHECK(stRebase(edge, 2) == -4.0f && rebasedFits(edge, 2, 1));
    const float negative[3] = {-20.3f, -19.1f, -19.9f};
    CHECK(stRebase(negative, 3) == -20.0f && rebasedFits(negative, 3, 1));
    const float wide[2] = {-5.0f, 9.5f};  // 14.5 repeats: rebased into range
    CHECK(stRebase(wide, 2) == 2.0f && rebasedFits(wide, 2, 1));
    // floor(middle) = 7 leaves 8.0 at the top, which saturates: 8 fits (review M2).
    const float lopsided[2] = {0.6f, 15.0f};
    CHECK(stRebase(lopsided, 2) == 8.0f && rebasedFits(lopsided, 2, 1));
    const float under[2] = {-15.0f, -0.6f};  // the mirror image: -8
    CHECK(stRebase(under, 2) == -8.0f && rebasedFits(under, 2, 1));
    const float tooWide[2] = {0.25f, 16.5f};  // no shift fits: floor(middle), saturating
    CHECK(!anyShiftFits(tooWide, 2, 1) && stRebase(tooWide, 2) == 8.0f);
    CHECK(stRebase(wide, 0) == 0.0f);
  }
  std::mt19937 rng(3);
  for (u32 i = 0; i < 20000; ++i) {
    // Packages anywhere in +-1000 repeats, 0 to 16.5 repeats wide: whenever any shift fits,
    // stRebase finds one - always, up to 15 repeats wide.
    const float base = std::uniform_real_distribution<float>(-1000.0f, 1000.0f)(rng);
    const float span = std::uniform_real_distribution<float>(0.0f, 16.5f)(rng);
    float uv[6];
    for (float& u : uv) u = base + std::uniform_real_distribution<float>(0.0f, span)(rng);
    float lo = uv[0], hi = uv[0];
    for (float u : uv) lo = u < lo ? u : lo, hi = u > hi ? u : hi;
    const bool any = anyShiftFits(uv, 6, 1);
    CHECK(rebasedFits(uv, 6, 1) == any);
    if (hi - lo <= 15.0f) CHECK(any);
    if (any && lo * 4096.0f > -32767.5f && hi * 4096.0f < 32767.5f) CHECK(stRebase(uv, 6) == 0.0f);
  }
  // The package's ST scale (ruling P16): the largest power of two at most 1 that fits.
  CHECK(stScaleFor(0.0f) == 1.0f && stScaleFor(7.9997f) == 1.0f && stScaleFor(8.0f) == 0.5f);
  CHECK(stScaleFor(32767.0f / 4096) == 1.0f && stScaleFor(32768.0f / 4096) == 0.5f);
  CHECK(stScaleFor(16.5f) == 0.25f && stScaleFor(1e30f) == 1.0f / 16777216 && stScaleFor(INFINITY) == 1.0f / 16777216);
  CHECK(stScaleFor(std::nanf("")) == 1.0f);
  CHECK(stScaleBits(1.0f) == 0 && stScaleBits(0.5f) == 1 && stScaleBits(1.0f / 16777216) == 24);
  CHECK(stRebaseRange(100.25f, 101.5f) == 100.0f && stRebaseRange(-7.5f, 7.5f) == 0.0f);
  for (u32 i = 0; i < 20000; ++i) {
    const float largest = std::ldexp(std::uniform_real_distribution<float>(0.0f, 1.0f)(rng),
                                     static_cast<int>(rng() % 34) - 8);  // up to 2^26: inside the 2^-24 floor
    const float k = stScaleFor(largest);
    int e = 0;
    CHECK(k <= 1.0f && std::frexp(k, &e) == 0.5f);  // a power of two
    CHECK(stFitsExact(largest * k) && stFitsExact(-largest * k));
    if (k < 1.0f) CHECK(!stFitsExact(largest * k * 2.0f));  // and the largest that fits
    CHECK(stScaleBits(k) == static_cast<u32>(1 - e));
  }
  // colour: 0..255 exact, 300 -> 255, -3 -> 0
  for (u32 v = 0; v <= 255; ++v) CHECK(quantizeColor(static_cast<float>(v)) == v);
  CHECK(quantizeColor(300.0f) == 255 && quantizeColor(-3.0f) == 0 && quantizeColor(127.5f) == 127);
  // truncated, as the float programs' FixColor (ftoi0) did: an unlit colour is the GS's as before
  for (float c = -2.0f; c < 258.0f; c += 0.37f)
    CHECK(quantizeColor(c) == (c <= 0.0f ? 0 : c >= 255.0f ? 255 : static_cast<int>(std::floor(c))));
  CHECK(quantizeColor(std::nanf("")) == 0 && quantizeColor(0.999f) == 0 && quantizeColor(254.999f) == 254);
  CHECK(quantizeColor(254.6f) == 254 && quantizeColor(0.4f) == 0 && quantizeColor(NAN) == 0);
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
u32 batchMismatchesHost(u32 seed);  // below, with the stream's checks

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
    CHECK(clipSeamMismatches(29 + m, 2500, nullptr) == 0);  // VU1 rounds toward zero
    CHECK(batchMismatchesHost(41 + m) == 0);  // the batch paths too
  }
  std::fesetround(FE_TONEAREST);
  endGroup();
}


// ---------------------------------------------------------------------------
// The stream (stapip_compact_stream.hpp)

void layouts() {
  beginGroup("stream_layouts");
  using L = StaPipCompactLayout;
  const L tc = L::forClass(true, false, false, false, 75, false);
  CHECK(tc.attrCount == 3 && tc.attrs[0] == Attr::Position && tc.attrs[1] == Attr::St &&
        tc.attrs[2] == Attr::Color && !tc.stripped && !tc.billboard);
  const L tce = L::forClass(true, true, false, false, 75, false);
  CHECK(tce.attrCount == 3 && tce.attrs[1] == Attr::EnvNormal && tce.attrs[2] == Attr::Color);
  const L tceLit = L::forClass(true, true, true, false, 75, false);  // env wins: never lit
  CHECK(tceLit == tce);
  const L tceSingle = L::forClass(true, true, false, true, 75, false);
  CHECK(tceSingle.attrCount == 2 && tceSingle.attrs[1] == Attr::EnvNormal);
  const L td = L::forClass(true, false, true, false, 75, false);
  CHECK(td.attrCount == 3 && td.attrs[1] == Attr::St && td.attrs[2] == Attr::Normal);
  CHECK(L::forClass(true, false, true, true, 75, false) == td);  // lit: no colours either way
  const L d = L::forClass(false, false, true, false, 111, false);
  CHECK(d.attrCount == 2 && d.attrs[1] == Attr::Normal);
  const L c = L::forClass(false, false, false, false, 111, false);
  CHECK(c.attrCount == 2 && c.attrs[1] == Attr::Color);
  const L single = L::forClass(false, false, false, true, 150, false);
  CHECK(single.attrCount == 1 && single.attrs[0] == Attr::Position);
  const L tcSingle = L::forClass(true, false, false, true, 75, false);
  CHECK(tcSingle.attrCount == 2 && tcSingle.attrs[1] == Attr::St);
  const L bb = L::forBillboards(21);
  CHECK(bb.attrCount == 3 && bb.billboard && bb.attrs[0] == Attr::BbCenter &&
        bb.attrs[1] == Attr::BbParams && bb.attrs[2] == Attr::BbColor);
  const L strip = L::forClass(true, false, false, false, 75, true);
  CHECK(strip.stripped && strip != tc);
  CHECK(tc.indexOf(Attr::Color) == 2 && tc.indexOf(Attr::Normal) == -1 && bb.indexOf(Attr::BbParams) == 1);
  // VU1 addresses: VERT_DATA + attrIndex x n, what the generated wrappers advance by.
  CHECK(tc.dest(0, 75) == VU1_STAPIP_VERT_DATA_ADDR && tc.dest(1, 75) == VU1_STAPIP_VERT_DATA_ADDR + 75 &&
        tc.dest(2, 41) == VU1_STAPIP_VERT_DATA_ADDR + 82);
  endGroup();
}

// The quadwords a stream of `n` vertices takes: per package three entry quadwords, a header and
// the padded payload per attribute, and a kick quadword.
u32 sumOverPackages(const StaPipCompactLayout& l, u32 n) {
  u32 qw = 0;
  for (u32 first = 0; first < n; first += l.packageSize) {
    const u32 m = n - first < l.packageSize ? n - first : l.packageSize;
    qw += 3 + 1;
    for (u32 a = 0; a < l.attrCount; ++a) qw += 1 + runQwords(l.attrs[a], m);
  }
  return qw;
}

enum class StKind { St, WideSt, Stq, EnvNormal, Weights };

// n vertices, 4 floats each: positions in a 60 m region around a random centre; the ST array per
// `kind` - ST whose packages sit anywhere in +-60 repeats but span under 6 (Q = 1), the same as
// a projective STQ (S = u Q, T = v Q, Q from 0.5 to 30 - a lamp's pool), env normals of any
// length, or billboard weights; colours mostly whole 0..255, some fractional or out of range;
// normals with components in [-1, 1].
struct Vertices {
  std::vector<float> pos, st, col, nrm;
};
Vertices makeVertices(u32 n, u32 packageSize, StKind kind, std::mt19937& rng) {
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  Vertices v;
  v.pos.resize(n * 4);
  v.st.resize(n * 4);
  v.col.resize(n * 4);
  v.nrm.resize(n * 4);
  float centre[3];
  for (float& c : centre) c = (unit(rng) - 0.5f) * 400.0f;
  float base[2] = {0.0f, 0.0f};
  for (u32 i = 0; i < n; ++i) {
    for (u32 a = 0; a < 3; ++a) v.pos[i * 4 + a] = centre[a] + (unit(rng) - 0.5f) * 60.0f;
    v.pos[i * 4 + 3] = 1.0f;
    if (i % packageSize == 0)
      for (float& b : base) b = std::floor((unit(rng) - 0.5f) * 120.0f);
    switch (kind) {
      case StKind::St:
        v.st[i * 4] = base[0] + unit(rng) * 5.5f;
        v.st[i * 4 + 1] = base[1] + unit(rng) * 5.5f;
        v.st[i * 4 + 2] = 1.0f;
        break;
      case StKind::WideSt:  // every third package's u spans up to 24 repeats
        v.st[i * 4] = base[0] + unit(rng) * ((i / packageSize) % 3 == 0 ? 24.0f : 14.0f);
        v.st[i * 4 + 1] = base[1] + unit(rng) * 15.0f;
        v.st[i * 4 + 2] = 1.0f;
        break;
      case StKind::Stq: {  // projective: the GS draws S/Q, T/Q
        const float q = 0.5f + 29.5f * unit(rng);
        v.st[i * 4] = (base[0] + unit(rng) * 5.5f) * q;
        v.st[i * 4 + 1] = (base[1] + unit(rng) * 5.5f) * q;
        v.st[i * 4 + 2] = q;
        break;
      }
      case StKind::EnvNormal: {
        const float len = 0.3f + 2.2f * unit(rng);
        float d[3], l2 = 0.0f;
        for (float& x : d) {
          x = unit(rng) - 0.5f;
          l2 += x * x;
        }
        const float s = len / std::sqrt(l2 > 1e-6f ? l2 : 1e-6f);
        for (u32 a = 0; a < 3; ++a) v.st[i * 4 + a] = d[a] * s;
        break;
      }
      case StKind::Weights:
        for (u32 a = 0; a < 4; ++a) v.st[i * 4 + a] = (unit(rng) - 0.5f) * 6.0f;
        break;
    }
    for (u32 c = 0; c < 4; ++c)
      v.col[i * 4 + c] = rng() % 10 < 7 ? static_cast<float>(rng() % 256) : unit(rng) * 300.0f - 20.0f;
    for (u32 a = 0; a < 3; ++a) v.nrm[i * 4 + a] = unit(rng) * 2.0f - 1.0f;
  }
  return v;
}

// Writes every vertex through a writer; returns its stClamped(), its stScaleBits() in `bits`.
u32 writeAll(StaPipCompactStream& s, const Vertices& v, u32 n, bool envNormals, u32* bits = nullptr) {
  StaPipCompactWriter w(s);
  for (u32 i = 0; i < n; ++i) {
    // Every attribute, in a scrambled order: the layout keeps what it has. A Q of 1 goes
    // through st(u, v), any other through stq(s, t, q).
    w.color(&v.col[i * 4]);
    w.normal(envNormals ? &v.st[i * 4] : &v.nrm[i * 4]);
    w.bbParams(&v.st[i * 4]);
    if (v.st[i * 4 + 2] == 1.0f)
      w.st(v.st[i * 4], v.st[i * 4 + 1]);
    else
      w.stq(v.st[i * 4], v.st[i * 4 + 1], v.st[i * 4 + 2]);
    w.position(&v.pos[i * 4]);
    w.next();
  }
  w.end();
  if (bits != nullptr) *bits = w.stScaleBits();
  return w.stClamped();
}

float largestAbs(const std::vector<float>& values) {
  float m = 0.0f;
  for (float x : values) m = std::fabs(x) > m ? std::fabs(x) : m;
  return m;
}

struct LayoutCase {
  const char* name;
  StaPipCompactLayout layout;
  StKind kind;
  bool world;
};

// Streams whose package 0 was walked without its kick's STMOD 0 and seen to end in offset MODE.
u32 modeFailuresShown = 0;

// One layout, `n` vertices: written, walked, decoded, boxed, packed and batch-decoded.
void streamCase(const LayoutCase& lc, u32 n, std::mt19937& rng) {
  const StaPipCompactLayout& l = lc.layout;
  const u32 size = l.packageSize;
  const Vertices v = makeVertices(n, size, lc.kind, rng);
  bool coarse = true;
  const Box box = boxFor(v.pos.data(), n, lc.world, &coarse);
  CHECK(!coarse);
  StaPipCompactStream s;
  CHECK(s.allocate(l, n, box));
  if (!s.allocated()) return;
  const u32 allocatedVersion = s.contentVersion();
  if (l.billboard) s.setWeightScale(weightScaleFor(largestAbs(v.st)));
  const bool env = l.indexOf(Attr::EnvNormal) >= 0;
  u32 writerBits = 0;
  const u32 writerClamped = writeAll(s, v, n, env, &writerBits);
  CHECK(s.contentVersion() != allocatedVersion);
  // The packages no whole-repeat shift fits into 4.12, found by trying every shift: since ruling
  // P16 they are stored at a smaller scale, never saturated (review M5 counted them clamped).
  const bool hasSt = l.indexOf(Attr::St) >= 0;
  const bool projective = lc.kind == StKind::Stq;
  std::vector<u8> packageScaled((n + size - 1) / size, 0);
  {
    u32 expected = 0;
    for (u32 p = 0; hasSt && !projective && p * size < n; ++p) {
      const u32 m = n - p * size < size ? n - p * size : size;
      packageScaled[p] = !anyShiftFits(&v.st[p * size * 4], m, 4) ||
                         !anyShiftFits(&v.st[p * size * 4 + 1], m, 4);
      expected += packageScaled[p];
    }
    CHECK(writerClamped == 0);
    if (!projective) CHECK((writerBits > 0) == (expected > 0));
    if (lc.kind == StKind::WideSt && n >= size) CHECK(expected > 0);
  }

  // Sizes, run headers, the entry and kick quadwords' places.
  const u32 packages = (n + size - 1) / size;
  CHECK(s.vertexCount() == n && s.packageCount() == packages);
  CHECK(s.qwords() == sumOverPackages(l, n));
  const u8* base = static_cast<const u8*>(s.data());
  CHECK((reinterpret_cast<uintptr_t>(base) & 15) == 0);
  u32 at = 0;  // the package's first quadword
  for (u32 p = 0; p < packages; ++p) {
    const u32 m = s.packageVertices(p);
    CHECK(m == (p + 1 < packages ? size : n - p * size));
    CHECK(s.entry(p) == base + at * 16);
    for (u32 q = 0; q < 48; ++q) CHECK(s.entry(p)[q] == 0);  // NOP entries until the renderer captures
    u32 next = at + 3;
    for (u8 a = 0; a < l.attrCount; ++a) {
      u32 want[4];
      runHeader(l.attrs[a], m, l.dest(a, m), want);
      CHECK(s.runHeader(p, a) == base + next * 16 && s.run(p, a) == base + next * 16 + 16);
      CHECK(headerWords(s.runHeader(p, a)) == headerWords(want));
      CHECK(s.runQwords(p, a) == runQwords(l.attrs[a], m));
      // the padding behind the data is zero (VIF NOPs)
      for (u32 b = runBytes(l.attrs[a], m); b < runQwords(l.attrs[a], m) * 16; ++b) CHECK(s.run(p, a)[b] == 0);
      next += 1 + runQwords(l.attrs[a], m);
    }
    CHECK(s.kick(p) == base + next * 16);
    for (u32 q = 0; q < 16; ++q) CHECK(s.kick(p)[q] == 0);
    at = next + 1;
  }
  CHECK(at == s.qwords());

  // VIF1 reads the whole block behind the bag's setup: no error, every run's UNPACK expects
  // exactly its data, and nothing is pending at the end - qwords() x 4 words, no more.
  {
    std::vector<u32> pendingAfter(s.qwords() * 4, ~0u);
    for (u32 p = 0; p < packages; ++p)
      for (u8 a = 0; a < l.attrCount; ++a)
        pendingAfter[(s.runHeader(p, a) - base) / 4 + 3] = (runBytes(l.attrs[a], s.packageVertices(p)) + 3) / 4;
    Vif1ChainCheck::Stream walk;
    u32 setup[kBagSetupWords];
    bagSetup(box, setup);
    for (u32 w : setup) walk.word(w);
    const u32* words = static_cast<const u32*>(s.data());
    for (u32 i = 0; i < s.qwords() * 4; ++i) {
      walk.word(words[i]);
      if (pendingAfter[i] != ~0u) CHECK(walk.result && walk.result.pending == pendingAfter[i]);
    }
    CHECK(walk.result && walk.result.pending == 0);
    walk.word(0x17000000u);  // the next word is a VIFcode
    CHECK(walk.result && walk.result.command == 0x17);
  }

  // Ruling P12, once the renderer has filled the entries and kicks with a program's own words
  // (setPackageWords): no package leaves VIF1 in offset MODE. MODE persists, so it is read where
  // the next VIFcode lands - behind the bag setup, each entry, each run header and each kick.
  {
    std::mt19937 dataRng(n * 31u + size);
    alignas(16) u32 entryData[8];
    for (u32& w : entryData) w = dataRng();
    const u32 entryCodes[2] = {0x05000000u, 0x6C020000u};  // STMOD 0, UNPACK V4-32 x 2
    for (u32 p = 0; p < packages; ++p) {
      const u32 kickCodes[2] = {0x11000000u, p == 0 ? 0x14000000u : 0x17000000u};  // FLUSH, MSCAL / MSCNT
      s.setPackageWords(p, entryCodes, entryData, kickCodes);
      const u32* e = reinterpret_cast<const u32*>(s.entry(p));
      const u32* k = reinterpret_cast<const u32*>(s.kick(p));
      CHECK(e[0] == 0x05000000u && e[1] == 0u && e[2] == entryCodes[0] && e[3] == entryCodes[1]);
      CHECK(std::memcmp(e + 4, entryData, 32) == 0);
      CHECK(k[0] == 0x05000000u && k[1] == 0u && k[2] == kickCodes[0] && k[3] == kickCodes[1]);
    }
    // A program whose CNT carries no STMOD (a game's own writer) still gets one from the slot.
    const u32 bareCodes[2] = {0u, 0x6C020000u};
    const u32 bareKick[2] = {0x11000000u, 0x14000000u};
    s.setPackageWords(0, bareCodes, entryData, bareKick);
    CHECK(reinterpret_cast<const u32*>(s.entry(0))[0] == 0x05000000u);
    // modeAfterKick[p]: what package p's kick leaves; false = the walk itself failed
    auto walkModes = [&](std::vector<u32>& modeAfterKick) {
      Vif1ChainCheck::Stream walk;
      u32 setup[kBagSetupWords];
      bagSetup(box, setup);
      for (u32 w : setup) walk.word(w);
      bool ok = walk.result && walk.mode == 0;
      for (u32 p = 0; p < packages; ++p) {
        const u32* e = reinterpret_cast<const u32*>(s.entry(p));
        for (u32 i = 0; i < 12; ++i) walk.word(e[i]);
        ok = ok && walk.result && walk.result.pending == 0 && walk.mode == 0;
        for (u8 a = 0; a < l.attrCount; ++a) {
          const u32* h = reinterpret_cast<const u32*>(s.runHeader(p, a));
          for (u32 i = 0; i < 4; ++i) walk.word(h[i]);
          ok = ok && walk.mode == (offset(l.attrs[a]) ? 1u : 0u);  // the tracking reads STMOD
          const u32* r = reinterpret_cast<const u32*>(s.run(p, a));
          for (u32 i = 0; i < s.runQwords(p, a) * 4; ++i) walk.word(r[i]);
        }
        const u32* k = reinterpret_cast<const u32*>(s.kick(p));
        for (u32 i = 0; i < 4; ++i) walk.word(k[i]);
        ok = ok && walk.result && walk.result.pending == 0;
        modeAfterKick[p] = walk.mode;
      }
      return ok;
    };
    std::vector<u32> modes(packages, 9u);
    CHECK(walkModes(modes));
    for (u32 p = 0; p < packages; ++p) CHECK(modes[p] == 0);
    // The check can fail: a positions-only package without the kick's STMOD 0 ends in offset MODE.
    if (offset(l.attrs[l.attrCount - 1])) {
      reinterpret_cast<u32*>(s.kick(0))[0] = 0u;
      CHECK(walkModes(modes) && modes[0] == 1);
      ++modeFailuresShown;
    }
    // A package built fresh - the program's entry, a REF of its runs, the SDK's FLUSH + MSCAL
    // kick, which has no slot for an STMOD - and then a uniform upload (a bag's object data, the
    // EE clipper's other uniform forms, an instanced copy's MVP): the renderer's uniform head
    // (Compact::kUniformHeadCodes, FLUSHE + STMOD 0) puts MODE back to normal before the
    // uniform's UNPACK; FLUSHE + NOP left the origin index added to the uniform (review I1).
    if (offset(l.attrs[l.attrCount - 1])) {
      auto modeAtUniform = [&](const u32 head[2]) {
        Vif1ChainCheck::Stream walk;
        u32 setup[kBagSetupWords];
        bagSetup(box, setup);
        for (u32 w : setup) walk.word(w);
        for (u32 w : entryCodes) walk.word(w);
        for (u32 w : entryData) walk.word(w);
        for (u8 a = 0; a < l.attrCount; ++a) {
          const u32* h = reinterpret_cast<const u32*>(s.runHeader(0, a));
          for (u32 i = 0; i < 4; ++i) walk.word(h[i]);
          const u32* r = reinterpret_cast<const u32*>(s.run(0, a));
          for (u32 i = 0; i < s.runQwords(0, a) * 4; ++i) walk.word(r[i]);
        }
        walk.word(0x11000000u);  // FLUSH
        walk.word(0x14000000u);  // MSCAL
        walk.word(head[0]);
        walk.word(head[1]);
        const u32 mode = walk.mode;
        walk.word(0x6C010014u);  // UNPACK V4-32 x 1 to a uniform's address
        for (u32 i = 0; i < 4; ++i) walk.word(0x3F800000u);
        return walk.result && walk.result.pending == 0 ? mode : 9u;
      };
      CHECK(kUniformHeadCodes[0] == 0x10000000u && kUniformHeadCodes[1] == 0x05000000u);
      CHECK(modeAtUniform(kUniformHeadCodes) == 0);
      const u32 flusheNop[2] = {0x10000000u, 0u};
      CHECK(modeAtUniform(flusheNop) == 1);  // the check can fail
    }
    s.clearPackageWords();
    CHECK(!s.entriesCaptured());
    for (u32 p = 0; p < packages; ++p) {
      for (u32 q = 0; q < 48; ++q) CHECK(s.entry(p)[q] == 0);
      for (u32 q = 0; q < 16; ++q) CHECK(s.kick(p)[q] == 0);
    }
  }

  // What the view decodes.
  const StaPipCompactView view(s);
  CHECK(view.count() == n);
  std::vector<float> packageLargest(packages, 0.0f), packageK(packages, 1.0f);
  const float step = box.step;
  const float scale = s.weightScale();
  for (u32 i = 0; i < n; ++i) {
    float p4[4];
    view.position(i, p4);
    for (u32 a = 0; a < 3; ++a) CHECK(std::fabs(p4[a] - v.pos[i * 4 + a]) <= step * 0.5f + 1e-6f);
    CHECK(p4[3] == 1.0f);
    float st2[2], c4[4], n3[3], w4[4];
    view.st(i, st2);
    view.color(i, c4);
    view.normal(i, n3);
    view.bbParams(i, w4);
    if (l.indexOf(Attr::St) >= 0) {
      // Each texture coordinate rebased by the package's whole repeats (S - n Q), then S, T and Q
      // x the package's power-of-two scale k (ruling P16): the view's S/Q is the rebased u to
      // 2^m / 8192, its STQ the stored lanes.
      const u32 p = i / size, first = p * size, m = s.packageVertices(p);
      float q3[3];
      view.stq(i, q3);
      const float q = v.st[i * 4 + 2];
      const float k = std::exp2(std::round(std::log2(q3[2] / q)));  // the package's scale
      CHECK(k <= 1.0f && std::fabs(q3[2] - q * k) <= 0.5f / 4096 + 1e-7f);
      if (!projective) CHECK(q3[2] == k && (k < 1.0f) == (packageScaled[p] != 0));
      packageK[p] = k;
      for (u32 c = 0; c < 3; ++c) packageLargest[p] = std::max(packageLargest[p], std::fabs(q3[c]) * 4096.0f);
      for (u32 c = 0; c < 2; ++c) {
        CHECK(st2[c] == q3[c] / q3[2]);  // S/Q, exact for Q = k
        const float u = v.st[i * 4 + c] / q;
        const double whole = static_cast<double>(u) - st2[c];
        const double tol = (0.5 / 4096 / k) * (1.0 + std::fabs(st2[c])) / (q3[2] / k) + 1e-5;
        CHECK(std::fabs(whole - std::round(whole)) <= tol);  // u less whole repeats
        CHECK(std::fabs(q3[c]) * 4096.0f < 32767.0f + 0.5f);
        if (!projective && k == 1.0f) {
          // a package that fits is stored exactly as unscaled 4.12 (what it was before P16)
          const float shift = stRebase(&v.st[first * 4 + c], m, 4);
          CHECK(st2[c] == static_cast<float>(quantizeSt(v.st[i * 4 + c] - shift)) / 4096.0f);
        }
      }
    } else {
      CHECK(st2[0] == 0.0f && st2[1] == 0.0f);
    }
    if (l.indexOf(Attr::Color) >= 0 || l.indexOf(Attr::BbColor) >= 0) {
      for (u32 c = 0; c < 4; ++c) CHECK(c4[c] == static_cast<float>(quantizeColor(v.col[i * 4 + c])));
    }
    if (l.indexOf(Attr::Normal) >= 0) {
      for (u32 a = 0; a < 3; ++a) CHECK(std::fabs(n3[a] - v.nrm[i * 4 + a]) <= 1.0f / 254 + 1e-7f);
    } else if (env) {
      const float* e = &v.st[i * 4];
      const float len = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
      const float got = std::sqrt(n3[0] * n3[0] + n3[1] * n3[1] + n3[2] * n3[2]);
      CHECK(std::fabs(got - 1.0f) <= 1.0f / 127);  // normalised as written
      for (u32 a = 0; a < 3; ++a) CHECK(std::fabs(n3[a] - e[a] / len) <= 1.0f / 254 + 1e-6f);
    }
    if (l.billboard) {
      for (u32 a = 0; a < 4; ++a) CHECK(std::fabs(w4[a] - v.st[i * 4 + a]) <= 0.5f / scale + 1e-6f);
    }
  }

  // Colours reach the block exactly when the layout carries them: another set written over
  // the same stream changes it, unless one colour serves the bag (review M5: a check that read
  // the view's 0 for an absent attribute could not fail).
  {
    Vertices other = v;
    for (float& c : other.col) c = static_cast<float>(255 - static_cast<int>(c < 0 ? 0 : c > 255 ? 255 : c));
    StaPipCompactStream o;
    CHECK(o.allocate(l, n, box));
    if (l.billboard) o.setWeightScale(s.weightScale());
    writeAll(o, other, n, env);
    const bool colours = l.indexOf(Attr::Color) >= 0 || l.indexOf(Attr::BbColor) >= 0;
    CHECK((std::memcmp(o.data(), s.data(), s.qwords() * 16) != 0) == colours);
  }

  // The part boxes: a third of a package each, holding exactly the decoded positions.
  {
    const u32 partSize = size / 3;
    CHECK(s.partCount() == (n + partSize - 1) / partSize);
    // In place, behind the block, six floats a part (review I1: the classifier reads these).
    CHECK(s.partBoxes() != nullptr &&
          reinterpret_cast<const u8*>(s.partBoxes()) >= base + s.qwords() * 16);
    for (u32 part = 0; part < s.partCount(); ++part) {
      float mn[3], mx[3];
      s.partBox(part, mn, mx);
      CHECK(std::memcmp(mn, s.partBoxes() + part * 6, sizeof(mn)) == 0);
      CHECK(std::memcmp(mx, s.partBoxes() + part * 6 + 3, sizeof(mx)) == 0);
    }
    // The coarse groups and the whole box: exact unions of the parts, and tight.
    const u32 groups = (s.partCount() + StaPipCompactStream::kCoarseParts - 1) /
                       StaPipCompactStream::kCoarseParts;
    CHECK(s.coarseCount() == groups && s.coarseBoxes() != nullptr && s.bounds() != nullptr);
    float all[6] = {1e30f, 1e30f, 1e30f, -1e30f, -1e30f, -1e30f};
    for (u32 g = 0; g < groups; ++g) {
      float u[6] = {1e30f, 1e30f, 1e30f, -1e30f, -1e30f, -1e30f};
      for (u32 part = g * StaPipCompactStream::kCoarseParts;
           part < s.partCount() && part < (g + 1) * StaPipCompactStream::kCoarseParts; ++part)
        for (u32 a = 0; a < 3; ++a) {
          u[a] = std::fmin(u[a], s.partBoxes()[part * 6 + a]);
          u[3 + a] = std::fmax(u[3 + a], s.partBoxes()[part * 6 + 3 + a]);
        }
      CHECK(std::memcmp(u, s.coarseBoxes() + g * 6, sizeof(u)) == 0);
      for (u32 a = 0; a < 3; ++a) {
        all[a] = std::fmin(all[a], u[a]);
        all[3 + a] = std::fmax(all[3 + a], u[3 + a]);
      }
    }
    CHECK(std::memcmp(all, s.bounds(), sizeof(all)) == 0);
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (u32 i = 0; i < n; ++i) {
      float p4[4];
      view.position(i, p4);
      for (u32 a = 0; a < 3; ++a) lo[a] = std::fmin(lo[a], p4[a]), hi[a] = std::fmax(hi[a], p4[a]);
    }
    for (u32 a = 0; a < 3; ++a) CHECK(s.bounds()[a] == lo[a] && s.bounds()[3 + a] == hi[a]);
    for (u32 part = 0; part < s.partCount(); ++part) {
      float mn[3], mx[3], lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
      s.partBox(part, mn, mx);
      for (u32 i = part * partSize; i < n && i < (part + 1) * partSize; ++i) {
        float p4[4];
        view.position(i, p4);
        for (u32 a = 0; a < 3; ++a) {
          CHECK(p4[a] >= mn[a] && p4[a] <= mx[a]);
          lo[a] = p4[a] < lo[a] ? p4[a] : lo[a];
          hi[a] = p4[a] > hi[a] ? p4[a] : hi[a];
        }
      }
      for (u32 a = 0; a < 3; ++a) CHECK(mn[a] == lo[a] && mx[a] == hi[a]);  // tight
    }
  }

  // Packing the same float arrays writes the same bytes (positions through the batch path).
  {
    const bool single = l.indexOf(Attr::Color) < 0 && l.indexOf(Attr::BbColor) < 0;
    StaPipCompactSource src;
    src.positions = v.pos.data();
    src.sts = v.st.data();
    src.colors = single ? nullptr : v.col.data();
    src.normals = v.nrm.data();
    src.count = n;
    StaPipCompactStream t;
    u32 clamped = 99;
    CHECK(packArrays(l, src, box, t, &clamped));
    CHECK(t.qwords() == s.qwords() && t.weightScale() == s.weightScale());
    CHECK(std::memcmp(t.data(), s.data(), s.qwords() * 16) == 0);
    CHECK(clamped == writerClamped);
    for (u32 part = 0; part < s.partCount(); ++part) {
      float a0[3], a1[3], b0[3], b1[3];
      s.partBox(part, a0, a1);
      t.partBox(part, b0, b1);
      CHECK(std::memcmp(a0, b0, sizeof(a0)) == 0 && std::memcmp(a1, b1, sizeof(a1)) == 0);
    }
    // again, into a stream that already matches: the block is kept, the content moves
    const void* block = t.data();
    const u32 version = t.contentVersion();
    CHECK(t.matches(l, n, box));
    CHECK(packArrays(l, src, box, t));
    CHECK(t.data() == block && t.contentVersion() != version);
    CHECK(std::memcmp(t.data(), s.data(), s.qwords() * 16) == 0);
  }

  // A scaled package is at the LARGEST scale that fits: twice it would not.
  for (u32 p = 0; p < packages; ++p)
    if (packageK[p] < 1.0f) CHECK(packageLargest[p] >= 16384.0f);

  // The renderer's copy pool: a package packed run by run (packRun, from the floats) or copied
  // out of the stream (copyRun) is the stream's run, byte for byte - padding included - whether
  // the vertices come as a range, as an index list, or reversed (the same set: the same rebase).
  {
    const bool single = l.indexOf(Attr::Color) < 0 && l.indexOf(Attr::BbColor) < 0;
    StaPipCompactSource src;
    src.positions = v.pos.data();
    src.sts = v.st.data();
    src.colors = single ? nullptr : v.col.data();
    src.normals = v.nrm.data();
    src.count = n;
    std::vector<u8> a0(size * 16 + 64), a1(size * 16 + 64), b0(size * 16 + 64), b1(size * 16 + 64);
    std::vector<u32> index(size), reversed(size);
    for (u32 p = 0; p < packages; ++p) {
      const u32 m = s.packageVertices(p), first = p * size;
      for (u32 i = 0; i < m; ++i) {
        index[i] = first + i;
        reversed[i] = first + m - 1 - i;
      }
      for (u8 a = 0; a < l.attrCount; ++a) {
        const u32 bytes = s.runQwords(p, a) * 16;
        const bool saturates = false;  // a package too wide for 4.12 is scaled (P16)
        std::fill(a0.begin(), a0.end(), 0xEE);
        std::fill(a1.begin(), a1.end(), 0xEE);
        CHECK(packRun(l.attrs[a], src, nullptr, first, m, box, s.weightScale(), a0.data()) == saturates);
        CHECK(packRun(l.attrs[a], src, index.data(), 0, m, box, s.weightScale(), a1.data()) == saturates);
        CHECK(std::memcmp(a0.data(), s.run(p, a), bytes) == 0);
        CHECK(std::memcmp(a1.data(), s.run(p, a), bytes) == 0);
        CHECK(a0[bytes] == 0xEE && a1[bytes] == 0xEE);  // nothing past the padding
        copyRun(s, a, nullptr, first, m, b0.data());
        copyRun(s, a, index.data(), 0, m, b1.data());
        CHECK(std::memcmp(b0.data(), s.run(p, a), bytes) == 0);
        CHECK(std::memcmp(b1.data(), s.run(p, a), bytes) == 0);
        // reversed: the elements reversed, both ways
        packRun(l.attrs[a], src, reversed.data(), 0, m, box, s.weightScale(), a0.data());
        copyRun(s, a, reversed.data(), 0, m, b0.data());
        CHECK(std::memcmp(a0.data(), b0.data(), bytes) == 0);
      }
    }
  }

  // The batch decode equals the per-vertex one, bit for bit, wherever a range starts and ends.
  {
    std::vector<float> buffer(n * 4 + 8);
    const u32 starts[] = {0, 1, size - 1, size, n / 2, n - 1};
    for (u32 first : starts) {
      if (first >= n) continue;
      for (u32 k : {1u, 2u, size, n - first, n + 5}) {  // n + 5 runs past the end: clamped
        for (u32 shift = 0; shift < 2; ++shift) {
          float* out = buffer.data() + shift;
          std::fill(buffer.begin(), buffer.end(), -7.0f);
          view.positions(first, k, out);
          const u32 got = k < n - first ? k : n - first;
          for (u32 i = 0; i < got; ++i) {
            float p4[4];
            view.position(first + i, p4);
            CHECK(std::memcmp(p4, out + i * 4, sizeof(p4)) == 0);
          }
          CHECK(out[got * 4] == -7.0f);  // nothing written past the stream's last vertex
        }
      }
    }
  }
}

void streams() {
  beginGroup("stream_write_view_pack");
  setSceneStep(1.0f / 512);
  using L = StaPipCompactLayout;
  const LayoutCase cases[] = {
      {"TC", L::forClass(true, false, false, false, 75, false), StKind::St, true},
      {"TCE", L::forClass(true, true, false, false, 75, false), StKind::EnvNormal, false},
      {"TD", L::forClass(true, false, true, false, 75, false), StKind::St, false},
      {"D", L::forClass(false, false, true, false, 111, false), StKind::St, false},
      {"C colours", L::forClass(false, false, false, false, 111, false), StKind::St, true},
      {"C single colour", L::forClass(false, false, false, true, 150, false), StKind::St, true},
      {"TC, ST past 16 repeats", L::forClass(true, false, false, false, 75, false), StKind::WideSt, true},
      {"TC, projective STQ", L::forClass(true, false, false, false, 75, false), StKind::Stq, true},
      {"TD, projective STQ", L::forClass(true, false, true, false, 75, false), StKind::Stq, false},
      {"billboard T", L::forBillboards(21), StKind::Weights, false},
      {"TC stripped, runs of 75", L::forClass(true, false, false, false, 75, true), StKind::St, true},
  };
  std::mt19937 rng(4004);
  for (const LayoutCase& lc : cases) {
    const u32 size = lc.layout.packageSize;
    const u32 counts[] = {1, 2, 3, 20, size - 1, size, size + 1, 2 * size + 7, 1000};
    const unsigned before = failed;
    for (u32 n : counts) streamCase(lc, n, rng);
    if (failed != before) std::fprintf(stderr, "  (layout %s failed)\n", lc.name);
  }
  CHECK(modeFailuresShown > 0);  // a positions-only layout ran the MODE check's failing arm
  endGroup();
}

// Review Focus 2: a package far out in ST is rebased by whole repeats with its fractions intact;
// one with a triangle spanning 20 repeats cannot be, and is stored at half scale instead of
// saturating (ruling P16) - and so is a projective STQ, whatever its Q.
void stRanges() {
  beginGroup("stream_st_rebase_scale");
  const StaPipCompactLayout l = StaPipCompactLayout::forClass(true, false, false, false, 75, false);
  const u32 n = 75 + 3;
  std::vector<float> pos(n * 4, 1.0f), st(n * 4, 0.0f), col(n * 4, 128.0f);
  for (u32 i = 0; i < n; ++i) st[i * 4 + 2] = 1.0f;  // Q
  std::mt19937 rng(77);
  for (u32 i = 0; i < 75; ++i) {
    st[i * 4] = 100.2f + 1.5f * std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);  // u in [100.2, 101.7]
    st[i * 4 + 1] = std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
  }
  const float tri[3] = {0.0f, 10.0f, 20.0f};  // the second package: one triangle, 20 repeats wide
  for (u32 i = 0; i < 3; ++i) st[(75 + i) * 4] = tri[i];
  bool coarse = true;
  const Box box = boxFor(pos.data(), n, false, &coarse);
  StaPipCompactStream s;
  CHECK(s.allocate(l, n, box));
  StaPipCompactWriter w(s);
  for (u32 i = 0; i < n; ++i) {
    w.position(&pos[i * 4]);
    w.st(st[i * 4], st[i * 4 + 1]);
    w.color(&col[i * 4]);
    w.next();
  }
  w.end();
  const StaPipCompactView view(s);
  for (u32 i = 0; i < 75; ++i) {
    float uv[2], q3[3];
    view.st(i, uv);
    view.stq(i, q3);
    CHECK(q3[2] == 1.0f);  // fits: scale 1, Q stored as 4096
    CHECK(uv[0] == static_cast<float>(quantizeSt(st[i * 4] - 100.0f)) / 4096.0f);  // stores u - 100
    CHECK(std::fabs(uv[0] - (st[i * 4] - 100.0f)) <= 0.5f / 4096 + 1e-6f);
    CHECK(uv[1] == static_cast<float>(quantizeSt(st[i * 4 + 1])) / 4096.0f);       // v needed no shift
  }
  float uv[2], q3[3];
  view.st(75, uv);
  view.stq(75, q3);
  CHECK(uv[0] == -10.0f && q3[0] == -5.0f && q3[2] == 0.5f);  // 0 - 10, at half scale: exact
  view.st(77, uv);
  CHECK(uv[0] == 10.0f);                                     // 20 - 10
  CHECK(w.stClamped() == 0 && w.stScaleBits() == 1);
  StaPipCompactSource src = {pos.data(), st.data(), col.data(), nullptr, n};
  StaPipCompactStream t;
  u32 clamped = 7, bits = 7;
  CHECK(packArrays(l, src, box, t, &clamped, &bits) && clamped == 0 && bits == 1);
  CHECK(std::memcmp(t.data(), s.data(), s.qwords() * 16) == 0);
  // st(u, v) is stq(u, v, 1).
  {
    StaPipCompactStream a;
    CHECK(a.allocate(l, n, box));
    StaPipCompactWriter wa(a);
    for (u32 i = 0; i < n; ++i) {
      wa.position(&pos[i * 4]);
      wa.stq(st[i * 4], st[i * 4 + 1], 1.0f);
      wa.color(&col[i * 4]);
      wa.next();
    }
    wa.end();
    CHECK(std::memcmp(a.data(), s.data(), s.qwords() * 16) == 0);
  }
  // The strided rebase is the plain one over the same values.
  for (u32 trial = 0; trial < 500; ++trial) {
    float interleaved[40 * 4], plain[40];
    const u32 m = 1 + rng() % 40;
    const float centre = std::uniform_real_distribution<float>(-500.0f, 500.0f)(rng);
    for (u32 i = 0; i < m; ++i) {
      plain[i] = centre + std::uniform_real_distribution<float>(-7.0f, 7.0f)(rng);
      interleaved[i * 4 + 1] = plain[i];
    }
    CHECK(stRebase(interleaved + 1, m, 4) == stRebase(plain, m));
  }
  // A projective STQ under REPEAT is rebased by whole repeats of S/Q - S - n Q - and comes back
  // as u - n: (S - n Q) / Q = S / Q - n.
  {
    std::vector<float> pst(st);
    for (u32 i = 0; i < 75; ++i) {
      const float q = 2.0f + std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
      pst[i * 4] = st[i * 4] * q;  // u in [100.2, 101.7]
      pst[i * 4 + 1] = st[i * 4 + 1] * q;
      pst[i * 4 + 2] = q;
    }
    StaPipCompactSource psrc = src;
    psrc.sts = pst.data();
    StaPipCompactStream ps;
    CHECK(packArrays(l, psrc, box, ps, &clamped, &bits) && clamped == 0);
    const StaPipCompactView pv(ps);
    for (u32 i = 0; i < 75; ++i) {
      pv.st(i, uv);
      pv.stq(i, q3);
      CHECK(std::fabs(uv[0] - (st[i * 4] - 100.0f)) <= 2.0f / 4096);  // S/Q - 100
      CHECK(std::fabs(uv[1] - st[i * 4 + 1]) <= 2.0f / 4096);
      CHECK(std::fabs(q3[2] - pst[i * 4 + 2]) <= 0.5f / 4096);           // Q 2..3: scale 1
    }
  }
  // A texture that does not REPEAT (stFixed): no whole-repeat shift - it would move a CLAMPed
  // picture (a lamp's pool) - so the package's scale holds the values as they are: u in
  // [100.2, 101.7] at 1/16 (101.7 / 16 is inside 8), and v - still rebased - shares that scale.
  {
    StaPipCompactLayout f = l;
    f.stFixed = 1;  // u fixed, v rebased
    CHECK(f != l);  // a different layout: the cache key moves with the wrap
    for (u32 i = 0; i < 75; ++i) st[i * 4 + 1] = 40.25f + 0.5f * (i % 3);  // v in [40.25, 41.25]
    StaPipCompactStream u;
    u32 clampedFixed = 7, bitsFixed = 0;
    CHECK(packArrays(f, src, box, u, &clampedFixed, &bitsFixed));
    CHECK(clampedFixed == 0 && bitsFixed == 4);  // u kept: scale 1/16 (the triangle 1/4)
    const StaPipCompactView fv(u);
    for (u32 i = 0; i < 75; ++i) {
      fv.st(i, uv);
      fv.stq(i, q3);
      CHECK(q3[2] == 1.0f / 16);
      CHECK(std::fabs(uv[0] - st[i * 4]) <= 8.0f / 4096 + 1e-5f);              // never shifted
      CHECK(std::fabs(uv[1] - (st[i * 4 + 1] - 40.0f)) <= 8.0f / 4096 + 1e-6f);  // v still rebased
    }
    // a fixed coordinate is stored as it is where the rebase would have shifted it: u in
    // [-8, -0.5] - the REPEAT layout moves it by -4 (-8 alone leaves the range), the fixed one
    // keeps every value, at half scale for the -8
    std::vector<float> st2(st);
    for (u32 i = 0; i < 75; ++i) st2[i * 4] = i == 0 ? -8.0f : -0.5f - 7.0f * (i % 7) / 6.0f;
    StaPipCompactSource src2 = src;
    src2.sts = st2.data();
    CHECK(stRebase(st2.data(), 75, 4) != 0.0f);  // the REPEAT layout would shift it
    CHECK(packArrays(f, src2, box, u));
    const StaPipCompactView fv2(u);
    for (u32 i = 0; i < 75; ++i) {
      fv2.st(i, uv);
      if (i == 0)
        CHECK(uv[0] == -8.0f);
      else
        CHECK(std::fabs(uv[0] - st2[i * 4]) <= 1.0f / 4096 + 1e-6f);
    }
    // the copy pool packs it the same way
    std::vector<u8> runBytes(u.runQwords(0, 1) * 16 + 32, 0xEE);
    const s32 stSlot = f.indexOf(Attr::St);
    CHECK(stSlot >= 0);
    packRun(Attr::St, src2, nullptr, 0, 75, box, 4096.0f, runBytes.data(), f.stFixed);
    CHECK(std::memcmp(runBytes.data(), u.run(0, static_cast<u8>(stSlot)), u.runQwords(0, static_cast<u8>(stSlot)) * 16) == 0);
    packRun(Attr::St, src2, nullptr, 0, 75, box, 4096.0f, runBytes.data(), 0);
    CHECK(std::memcmp(runBytes.data(), u.run(0, static_cast<u8>(stSlot)), u.runQwords(0, static_cast<u8>(stSlot)) * 16) != 0);
  }
  // A lamp's pool: CLAMP on both axes, S = (0.5 + 0.4 x) Q, Q from 0.5 to 20 m - S, T and Q
  // all at the package's scale (1/4 here: 18 / 4 is inside 8), and S/Q, T/Q within what that
  // scale keeps.
  {
    StaPipCompactLayout pool = l;
    pool.stFixed = 3;
    std::vector<float> lamp(n * 4, 0.0f);
    for (u32 i = 0; i < n; ++i) {
      const float q = 0.5f + 19.5f * (i % 13) / 12.0f;
      const float x = std::uniform_real_distribution<float>(-1.0f, 1.0f)(rng);
      const float y = std::uniform_real_distribution<float>(-1.0f, 1.0f)(rng);
      lamp[i * 4] = (0.5f + 0.4f * x) * q;
      lamp[i * 4 + 1] = (0.5f + 0.4f * y) * q;
      lamp[i * 4 + 2] = q;
    }
    StaPipCompactSource lsrc = src;
    lsrc.sts = lamp.data();
    StaPipCompactStream ls;
    u32 lampBits = 0;
    CHECK(packArrays(pool, lsrc, box, ls, &clamped, &lampBits) && clamped == 0 && lampBits == 2);
    const StaPipCompactView lv(ls);
    for (u32 i = 0; i < 75; ++i) {
      lv.st(i, uv);
      lv.stq(i, q3);
      const float q = lamp[i * 4 + 2];
      CHECK(std::fabs(q3[2] - q * 0.25f) <= 0.5f / 4096);
      // S/Q: S' and Q' each within half a unit of 1/4096 at scale 1/4
      const float tol = 4.0f / 4096 * (1.0f + std::fabs(uv[0])) / (q * 0.25f) * 0.25f + 1e-5f;
      CHECK(std::fabs(uv[0] - lamp[i * 4] / q) <= tol);
      CHECK(std::fabs(uv[1] - lamp[i * 4 + 1] / q) <= tol);
    }
  }
  endGroup();
}

// allocate() is nothrow: a refused block returns false and keeps nothing; so do arguments that
// cannot make a stream. Nothing leaks across allocate, release, a move or a destructor.
void allocation() {
  beginGroup("stream_allocation");
  const StaPipCompactLayout l = StaPipCompactLayout::forClass(true, false, false, false, 75, false);
  const float mn[3] = {-10.0f, 0.0f, -10.0f}, mx[3] = {10.0f, 4.0f, 10.0f};
  const Box box = modelBox(mn, mx);
  const long before = liveArrays;
  {
    StaPipCompactStream s;
    refuseIn = 0;
    CHECK(!s.allocate(l, 100, box));
    refuseIn = -1;
    CHECK(!s.allocated() && s.data() == nullptr && s.qwords() == 0 && s.packageCount() == 0);
    CHECK(liveArrays == before);
    CHECK(s.allocate(l, 100, box) && liveArrays == before + 1);  // one allocation, boxes in it
    CHECK(s.weightScale() == 4096.0f);
    s.setWeightScale(2.0f);
    CHECK(s.allocate(l, 100, box) && s.weightScale() == 4096.0f);  // allocate resets it
    s.setWeightScale(2.0f);
    s.release();
    CHECK(s.weightScale() == 4096.0f && s.partBoxes() == nullptr && s.bounds() == nullptr);
    CHECK(s.allocate(l, 100, box) && liveArrays == before + 1);
    // contentVersion comes from one counter for every stream (review I2): two streams freshly
    // packed the same way never read the same version, nor does a stream allocated again -
    // a cache keyed on (block address, version) cannot take one for the other.
    {
      StaPipCompactStream a, b;
      CHECK(a.allocate(l, 100, box) && b.allocate(l, 100, box));
      CHECK(a.contentVersion() != b.contentVersion() && a.contentVersion() != 0);
      const u32 first = a.contentVersion();
      a.finish();
      CHECK(a.contentVersion() != first && a.contentVersion() != b.contentVersion());
      const u32 finished = a.contentVersion();
      a.release();
      CHECK(a.allocate(l, 100, box) && a.contentVersion() != finished && a.contentVersion() != first);
    }
    refuseIn = 0;  // refused while holding a block: the old one is released first
    CHECK(!s.allocate(l, 300, box));
    refuseIn = -1;
    CHECK(!s.allocated() && liveArrays == before);
    // Arguments that cannot make a stream.
    StaPipCompactLayout bad = l;
    CHECK(!s.allocate(l, 0, box));
    bad.packageSize = 2;
    CHECK(!s.allocate(bad, 100, box));
    bad.packageSize = 257;
    CHECK(!s.allocate(bad, 100, box));
    bad = l;
    bad.attrCount = 0;
    CHECK(!s.allocate(bad, 100, box));
    bad = l;
    bad.attrs[0] = Attr::Color;
    CHECK(!s.allocate(bad, 100, box));
    const Box notPow2 = {{0.0f, 0.0f, 0.0f}, 0.003f};
    const Box offGrid = {{0.001f, 0.0f, 0.0f}, 1.0f / 512};
    const Box farOrigin = {{40000.0f, 0.0f, 0.0f}, 1.0f / 512};  // index 2^24 and more
    const Box nanOrigin = {{NAN, 0.0f, 0.0f}, 1.0f / 512};
    CHECK(!s.allocate(l, 100, notPow2) && !s.allocate(l, 100, offGrid));
    CHECK(!s.allocate(l, 100, farOrigin) && !s.allocate(l, 100, nanOrigin));
    CHECK(liveArrays == before);
    // A writer and a view of a stream that holds nothing do nothing.
    StaPipCompactWriter w(s);
    const float p[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    w.position(p);
    w.st(1.0f, 2.0f);
    w.color(p);
    w.normal(p);
    w.bbParams(p);
    w.next();
    w.end();
    const StaPipCompactView view(s);
    float out[4] = {9.0f, 9.0f, 9.0f, 9.0f};
    CHECK(view.count() == 0);
    view.positions(0, 4, out);
    CHECK(out[0] == 9.0f);
    view.position(0, out);
    CHECK(out[0] == 0.0f && out[3] == 1.0f);
    // packArrays refused: false, nothing kept
    std::vector<float> pos(400, 1.0f);
    StaPipCompactSource src = {pos.data(), nullptr, nullptr, nullptr, 100};
    refuseIn = 0;
    CHECK(!packArrays(l, src, box, s));
    refuseIn = -1;
    CHECK(!s.allocated() && liveArrays == before);
  }
  {
    StaPipCompactStream a;
    CHECK(a.allocate(l, 160, box));
    const void* block = a.data();
    StaPipCompactStream b(std::move(a));
    CHECK(!a.allocated() && b.allocated() && b.data() == block && b.vertexCount() == 160);
    StaPipCompactStream c;
    CHECK(c.allocate(l, 10, box));
    c = std::move(b);
    CHECK(!b.allocated() && c.data() == block && liveArrays == before + 1);
    c.release();
    c.release();
    CHECK(liveArrays == before);
    CHECK(c.allocate(l, 75, box));
  }
  CHECK(liveArrays == before);  // the destructor released the last one
  endGroup();
}

// Boxes for bags and the scene step (decision 3).
void bagBoxes() {
  beginGroup("stream_boxes_scene_step");
  setSceneStep(1.0f / 512);
  CHECK(sceneStep() == 1.0f / 512);
  setSceneStep(0.003f);
  CHECK(sceneStep() == 1.0f / 256);  // the power of two at or above
  setSceneStep(1e-6f);
  CHECK(sceneStep() == 1.0f / 1024);  // the floor
  setSceneStep(-1.0f);
  setSceneStep(NAN);
  CHECK(sceneStep() == 1.0f / 1024);  // ignored
  setSceneStep(1.0f / 512);
  std::mt19937 rng(31);
  for (u32 trial = 0; trial < 300; ++trial) {
    const u32 n = 1 + rng() % 200;
    const float extent = trial % 3 == 0 ? 200.0f : 100.0f;  // past and inside 2^-9's 128 m
    std::vector<float> pos(n * 4);
    float mn[3] = {1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f, -1e30f, -1e30f};
    const float centre = std::uniform_real_distribution<float>(-300.0f, 300.0f)(rng);
    for (u32 i = 0; i < n; ++i)
      for (u32 a = 0; a < 4; ++a) {
        const float x = a == 3 ? 1.0f : centre + std::uniform_real_distribution<float>(0.0f, extent)(rng);
        pos[i * 4 + a] = x;
        if (a < 3) {
          mn[a] = x < mn[a] ? x : mn[a];
          mx[a] = x > mx[a] ? x : mx[a];
        }
      }
    if (n > 3) pos[4 * (n / 2)] = NAN;  // skipped by the bounds
    if (n > 3) {
      mn[0] = 1e30f;
      mx[0] = -1e30f;
      for (u32 i = 0; i < n; ++i)
        if (i != n / 2) {
          mn[0] = pos[i * 4] < mn[0] ? pos[i * 4] : mn[0];
          mx[0] = pos[i * 4] > mx[0] ? pos[i * 4] : mx[0];
        }
    }
    bool coarse = false, want = false;
    const Box world = boxFor(pos.data(), n, true, &coarse);
    const Box expect = worldBox(mn, mx, 1.0f / 512, &want);
    CHECK(coarse == want && std::memcmp(&world, &expect, sizeof(Box)) == 0);
    if (!coarse) CHECK(world.step == 1.0f / 512);
    for (u32 a = 0; a < 3; ++a) CHECK(onGrid(world.origin[a], world.step));
    coarse = true;
    const Box model = boxFor(pos.data(), n, false, &coarse);
    const Box modelWant = modelBox(mn, mx);
    CHECK(!coarse && std::memcmp(&model, &modelWant, sizeof(Box)) == 0);
  }
  endGroup();
}

// The batch paths on this host (the C++ arm of stapip_compact_ee.cpp), NaN and infinity
// included: quantizePositions equals quantize, decodePositions equals dequantize.
u32 batchMismatchesHost(u32 seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  u32 bad = 0;
  for (u32 trial = 0; trial < 60; ++trial) {
    float mn[3], mx[3];
    for (u32 a = 0; a < 3; ++a) {
      mn[a] = (unit(rng) - 0.5f) * 600.0f;
      mx[a] = mn[a] + unit(rng) * (trial % 2 ? 120.0f : 3.0f);
    }
    bool coarse = false;
    const Box box = trial % 2 ? worldBox(mn, mx, 1.0f / 512, &coarse) : modelBox(mn, mx);
    const u32 n = 1 + rng() % 90;
    std::vector<float> in(n * 4);
    for (u32 i = 0; i < n; ++i)
      for (u32 a = 0; a < 4; ++a) {
        float x = mn[a % 3] + (mx[a % 3] - mn[a % 3]) * (unit(rng) * 1.4f - 0.2f);
        switch (rng() % 16) {
          case 0: x = NAN; break;
          case 1: x = INFINITY; break;
          case 2: x = -INFINITY; break;
          case 3: x = (std::round(x / box.step) + 0.5f) * box.step; break;  // a half step
          case 4: x = 1e30f * (unit(rng) - 0.5f); break;
          default: break;
        }
        in[i * 4 + a] = x;
      }
    const u32 stride = trial % 3 == 0 ? 8 : 6;
    std::vector<u8> out(n * stride + 2, 0x5A);
    quantizePositions(in.data(), n, box, out.data(), stride);
    for (u32 i = 0; i < n; ++i) {
      s16 q[3];
      std::memcpy(q, out.data() + i * stride, sizeof(q));
      for (u32 a = 0; a < 3; ++a)
        if (q[a] != quantize(in[i * 4 + a], box.origin[a], box.step)) ++bad;
      if (stride == 8 && (out[i * 8 + 6] != 0x5A || out[i * 8 + 7] != 0x5A)) ++bad;
    }
    // Decoded: step x the grid index bit for bit (what the VU computes), dequantize's value.
    std::vector<float> dec(n * 4);
    decodePositions(out.data(), stride, n, box, dec.data());
    u32 row[4];
    rowFor(box, row);
    for (u32 i = 0; i < n; ++i) {
      s16 q[3];
      std::memcpy(q, out.data() + i * stride, sizeof(q));
      for (u32 a = 0; a < 3; ++a) {
        const float grid = box.step * static_cast<float>(static_cast<s32>(row[a]) + q[a]);
        if (!sameBits(dec[i * 4 + a], grid)) ++bad;
        if (dec[i * 4 + a] != dequantize(q[a], box.origin[a], box.step)) ++bad;
      }
      if (dec[i * 4 + 3] != 1.0f) ++bad;
    }
  }
  return bad;
}

void batchPaths() {
  beginGroup("batch_paths");
  CHECK(batchMismatchesHost(808) == 0);
  // The weights' scale: the largest power of two that keeps them inside +-32767.
  std::mt19937 rng(9);
  for (u32 i = 0; i < 2000; ++i) {
    const float w = std::ldexp(std::uniform_real_distribution<float>(0.5f, 1.0f)(rng),
                               static_cast<int>(rng() % 30) - 10);
    const float s = weightScaleFor(w);
    CHECK(isPowerOfTwo(s) && w * s <= 32767.0f);
    CHECK(s == 16777216.0f || w * s * 2.0f > 32767.0f);
    CHECK(quantizeWeight(w, s) == static_cast<s16>(std::lround(static_cast<double>(w) * s)));
    CHECK(quantizeWeight(-w, s) == -quantizeWeight(w, s));
  }
  CHECK(weightScaleFor(0.0f) == 4096.0f && weightScaleFor(NAN) == 4096.0f);
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
  layouts();
  streams();
  stRanges();
  allocation();
  bagBoxes();
  batchPaths();
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
