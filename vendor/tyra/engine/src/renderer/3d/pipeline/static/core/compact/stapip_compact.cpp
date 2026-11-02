/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# TyraX
#-----------------------------------------------------------------------
*/

// Added by TyraX: the compact vertex format (stapip_compact.hpp,
// docs/compact-vertex-format.md). Portable C++ - no PS2 header - so the editor
// and tools/verify-stapip-compact.cpp run exactly this code.

#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

#include <cmath>

namespace Tyra {
namespace Compact {
namespace {

constexpr u32 kVifNop = 0x00000000u;
constexpr u32 kVifStmask = 0x20000000u;
constexpr u32 kVifStcycl11 = 0x01000101u;  // STCYCL cl=1 wl=1
constexpr u32 kUsn = 1u << 14;  // UNPACK immediate: unsigned (zero extension)
constexpr u32 kFlg = 1u << 15;  // UNPACK immediate: address relative to TOPS
constexpr u32 kMaskOn = 0x10u;  // UNPACK command: masked (m)

constexpr s32 kMaxQ = 32767;      // symmetric: -32768 is never written
constexpr s32 kBoxSteps = 65534;  // a box spans -32767 .. 32767
constexpr float kMinSceneStep = 1.0f / 1024;
// roundHalfAway's range: 2x must stay inside s32.
constexpr float kRoundLimit = 536870912.0f;  // 2^29
// A world box's origin index must be an exact float for origin = k * step.
constexpr s32 kExactGrid = 16777216;  // 2^24

/**
 * Round half away from zero, with exact operations only: trunc(2x) - trunc(x).
 * 2x is exact (a power-of-two scale) and a truncation is exact, so the result
 * is the same under every rounding mode. (floor(x + 0.5) is not: under
 * round-to-nearest, x = 0.5 - 2^-25 adds up to exactly 1.0 and reads 1.)
 * Saturates at +-2^29; NaN reads 0.
 */
s32 roundHalfAway(float x) {
  if (!(x == x)) return 0;
  if (x >= kRoundLimit) return static_cast<s32>(kRoundLimit);
  if (x <= -kRoundLimit) return -static_cast<s32>(kRoundLimit);
  return static_cast<s32>(x + x) - static_cast<s32>(x);
}

s32 clampQ(s32 q) { return q > kMaxQ ? kMaxQ : (q < -kMaxQ ? -kMaxQ : q); }

/**
 * The box of step `step` over [mn, mx]: per axis, the grid points of the two
 * ends, and an origin index halfway between them, so both ends land inside
 * +-32767. False when an axis spans more than 65 534 steps, or an end lies
 * `gridLimit` steps or more from 0.
 */
bool fit(const float mn[3], const float mx[3], float step, s32 gridLimit,
         Box* out) {
  const float inv = 1.0f / step;
  s32 k[3];
  for (u32 a = 0; a < 3; ++a) {
    float lo = mn[a] * inv, hi = mx[a] * inv;
    if (hi < lo) {
      const float t = lo;
      lo = hi;
      hi = t;
    }
    const float limit = static_cast<float>(gridLimit);
    if (!(lo > -limit && hi < limit)) return false;  // NaN fails too
    const s32 glo = roundHalfAway(lo), ghi = roundHalfAway(hi);
    if (ghi - glo > kBoxSteps) return false;
    k[a] = glo + (ghi - glo) / 2;
  }
  out->step = step;
  for (u32 a = 0; a < 3; ++a) out->origin[a] = static_cast<float>(k[a]) * step;
  return true;
}

}  // namespace

Format formatOf(Attr a) {
  switch (a) {
    case Attr::Position:
    case Attr::BbCenter:
      return kPositionFormat;
    case Attr::St:
      return Format::V2_16;
    case Attr::BbParams:
      return Format::V4_16;
    case Attr::Color:
    case Attr::Normal:
    case Attr::EnvNormal:
    case Attr::BbColor:
      break;
  }
  return Format::V4_8;
}

bool isSigned(Attr a) { return a != Attr::Color && a != Attr::BbColor; }

bool masked(Attr a) {
  return a == Attr::Position || a == Attr::BbCenter || a == Attr::St;
}

u32 elementBytes(Attr a) {
  switch (formatOf(a)) {
    case Format::V2_16:
      return 4;
    case Format::V3_16:
      return 6;
    case Format::V4_16:
      return 8;
    case Format::V4_8:
      break;
  }
  return 4;
}

u32 runBytes(Attr a, u32 n) { return n * elementBytes(a); }

u32 runQwords(Attr a, u32 n) { return (runBytes(a, n) + 15u) / 16u; }

u32 maskWord(Attr a) {
  // One write cycle's lanes, x in the low bits: w from ROW is 01 in bits 7:6,
  // z and w 0101 in bits 7:4. The same byte for all four cycles.
  if (a == Attr::St) return 0x50505050u;
  if (masked(a)) return 0x40404040u;
  return 0u;
}

u32 unpackCode(Attr a, u32 num, u32 dest, bool useTop) {
  const u32 cmd =
      0x60u | (masked(a) ? kMaskOn : 0u) | static_cast<u32>(formatOf(a));
  const u32 imm = (dest & 0x3FFu) | (isSigned(a) ? 0u : kUsn) |
                  (useTop ? kFlg : 0u);
  return (cmd << 24) | ((num & 0xFFu) << 16) | imm;
}

void runHeader(Attr a, u32 num, u32 dest, u32 out[4]) {
  if (masked(a)) {
    out[0] = kVifStmask;
    out[1] = maskWord(a);
  } else {
    out[0] = kVifNop;
    out[1] = kVifNop;
  }
  out[2] = kVifStcycl11;
  out[3] = unpackCode(a, num, dest, true);
}

float sceneStepFor(float largestWorldExtent) {
  float step = kMinSceneStep;
  // 65 533 * step is exact: a power-of-two scale. 2^20 m ends a NaN or infinity.
  for (u32 i = 0; i < 30 && !(largestWorldExtent <= 65533.0f * step); ++i)
    step *= 2.0f;
  return step;
}

Box worldBox(const float mn[3], const float mx[3], float sceneStep,
             bool* coarse) {
  float step = sceneStep > 0.0f ? sceneStep : kMinSceneStep;
  Box box = {{0.0f, 0.0f, 0.0f}, step};
  bool wider = false;
  for (u32 i = 0; i < 40; ++i) {
    if (fit(mn, mx, step, kExactGrid, &box)) break;
    step *= 2.0f;
    wider = true;
  }
  if (coarse) *coarse = wider;
  return box;
}

Box modelBox(const float mn[3], const float mx[3]) {
  float extent = 0.0f;
  for (u32 a = 0; a < 3; ++a) {
    const float e = std::fabs(mx[a] - mn[a]);
    if (e > extent) extent = e;
  }
  float step = extent > 0.0f && extent < 1e30f ? extent / static_cast<float>(kBoxSteps)
                                               : 1.0f / static_cast<float>(kBoxSteps);
  Box box = {{0.0f, 0.0f, 0.0f}, step};
  // The ends' grid points can sit 65 535 steps apart when the box does not
  // start on its grid; one step more across makes 65 534 hold it. A model 2^29
  // steps from its own origin takes coarser steps until it fits.
  for (u32 i = 0; i < 40; ++i) {
    if (fit(mn, mx, step, static_cast<s32>(kRoundLimit), &box)) break;
    step *= i == 0 ? static_cast<float>(kBoxSteps + 1) / kBoxSteps : 2.0f;
  }
  return box;
}

void dequant(const Box& box, float colMajor[16]) {
  for (u32 i = 0; i < 16; ++i) colMajor[i] = 0.0f;
  colMajor[0] = box.step;
  colMajor[5] = box.step;
  colMajor[10] = box.step;
  colMajor[12] = box.origin[0];
  colMajor[13] = box.origin[1];
  colMajor[14] = box.origin[2];
  colMajor[15] = 1.0f;
}

s16 quantize(float v, float origin, float step) {
  if (!(v == v)) return 0;
  const float inv = 1.0f / step;
  // Both within +-2^29, so the difference cannot overflow.
  const s32 q = roundHalfAway(v * inv) - roundHalfAway(origin * inv);
  return static_cast<s16>(clampQ(q));
}

float dequantize(s16 q, float origin, float step) {
  return origin + step * static_cast<float>(q);
}

s16 quantizeSt(float v) {
  return static_cast<s16>(clampQ(roundHalfAway(v * 4096.0f)));
}

float stRebase(const float* uv, u32 n) {
  if (n == 0) return 0.0f;
  float lo = uv[0], hi = uv[0];
  for (u32 i = 1; i < n; ++i) {
    if (uv[i] < lo) lo = uv[i];
    if (uv[i] > hi) hi = uv[i];
  }
  if (!(lo >= -8.0f && hi < 8.0f) && lo == lo && hi == hi)
    return std::floor(lo * 0.5f + hi * 0.5f);
  return 0.0f;
}

u8 quantizeColor(float v) {
  const s32 q = roundHalfAway(v);
  return static_cast<u8>(q < 0 ? 0 : (q > 255 ? 255 : q));
}

s8 quantizeNormal(float v) {
  const s32 q = roundHalfAway(v * 127.0f);
  return static_cast<s8>(q < -127 ? -127 : (q > 127 ? 127 : q));
}

namespace {

// Known answers. Every input is chosen so each operation on it is exact (power
// of two scales, values that fit 24 bits), so the answers hold under every
// float rounding mode - the EE's toward zero as much as a host's to nearest.
struct QuantizeCase {
  float v, origin, step;
  s16 q;
};
const QuantizeCase kQuantize[] = {
    {0.0f, 0.0f, 0x1p-9f, 0},
    {0x1p-10f, 0.0f, 0x1p-9f, 1},     // a half step: away from zero
    {-0x1p-10f, 0.0f, 0x1p-9f, -1},
    {0x1p-10f - 0x1p-34f, 0.0f, 0x1p-9f, 0},  // 0.5 - 2^-25 steps: not a tie
    {0x1p-10f - 0x1p-34f, -32.0f, 0x1p-9f, 16384},  // same point, origin 32 m off
    {12.3456789f, 10.0f, 0x1p-10f, 2402},
    {-3.75f, -64.0f, 0x1p-8f, 15424},
    {100.0f, 0.0f, 0x1p-9f, 32767},    // saturated, symmetric
    {-100.0f, 0.0f, 0x1p-9f, -32767},
    {64.0f, 0.0f, 0x1p-9f, 32767},     // 32 768 steps: saturated
    {1e30f, 0.0f, 0x1p-10f, 32767},
};

}  // namespace

u32 selfTest() {
  u32 bad = 0;
  for (const QuantizeCase& c : kQuantize)
    if (quantize(c.v, c.origin, c.step) != c.q) ++bad;
  // The seam rule: one point in two boxes on one grid decodes to one float.
  {
    const float p = 0x1p-10f - 0x1p-34f, s = 0x1p-9f;
    const float a = dequantize(quantize(p, -32.0f, s), -32.0f, s);
    const float b = dequantize(quantize(p, 0.0f, s), 0.0f, s);
    if (!(a == b && a == 0.0f)) ++bad;
  }
  if (dequantize(-32767, 8.0f, 0x1p-9f) != -55.998046875f) ++bad;
  // ST, colour, normal.
  if (quantizeSt(1.0f) != 4096 || quantizeSt(-8.0f) != -32767 ||
      quantizeSt(9.0f) != 32767 || quantizeSt(0x1p-13f) != 1 ||
      quantizeSt(-0x1p-13f) != -1 || quantizeSt(0x1p-13f - 0x1p-37f) != 0)
    ++bad;
  {
    const float uv[2] = {100.25f, 101.5f};
    const float inside[2] = {-8.0f, 7.5f};
    if (stRebase(uv, 2) != 100.0f || stRebase(inside, 2) != 0.0f) ++bad;
  }
  if (quantizeColor(0.0f) != 0 || quantizeColor(255.0f) != 255 ||
      quantizeColor(127.5f) != 128 || quantizeColor(300.0f) != 255 ||
      quantizeColor(-3.0f) != 0)
    ++bad;
  if (quantizeNormal(1.0f) != 127 || quantizeNormal(-1.0f) != -127 ||
      quantizeNormal(0.5f) != 64 || quantizeNormal(2.0f) != 127)
    ++bad;
  // The VIF words.
  if (unpackCode(Attr::Position, 75, 2, true) != 0x794B8002u ||
      unpackCode(Attr::Color, 75, 2, true) != 0x6E4BC002u ||
      unpackCode(Attr::St, 75, 77, true) != 0x754B804Du ||
      maskWord(Attr::Position) != 0x40404040u ||
      maskWord(Attr::St) != 0x50505050u || maskWord(Attr::Normal) != 0u)
    ++bad;
  if (runQwords(Attr::Position, 75) != 29 || runQwords(Attr::St, 75) != 19 ||
      runQwords(Attr::Color, 75) != 19 || runQwords(Attr::BbParams, 75) != 38)
    ++bad;
  // The boxes.
  if (sceneStepFor(10.0f) != 0x1p-10f || sceneStepFor(65.0f) != 0x1p-9f ||
      sceneStepFor(130.0f) != 0x1p-8f)
    ++bad;
  {
    const float mn[3] = {-64.0f, 0.5f, 3.0f};
    const float mx[3] = {-64.0f + 65534.0f * 0x1p-9f, 2.0f, 3.0f};
    bool coarse = true;
    const Box box = worldBox(mn, mx, 0x1p-9f, &coarse);
    if (coarse || box.step != 0x1p-9f || box.origin[0] != 0.0f - 0x1p-9f ||
        quantize(mn[0], box.origin[0], box.step) != -32767 ||
        quantize(mx[0], box.origin[0], box.step) != 32767)
      ++bad;
    const float wide[3] = {-64.0f + 65535.0f * 0x1p-9f, 2.0f, 3.0f};
    const Box wider = worldBox(mn, wide, 0x1p-9f, &coarse);
    if (!coarse || wider.step != 0x1p-8f) ++bad;
  }
  return bad;
}

}  // namespace Compact
}  // namespace Tyra
