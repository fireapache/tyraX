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

// Added by TyraX: the compact format's batch paths (stapip_compact.hpp,
// docs/compact-vertex-format.md "The stream"): positions quantized and decoded
// a vertex a VU0 / MMI pass on the EE, the same steps in C++ elsewhere - and
// selfTest, which holds them to the per-element functions on the platform it
// runs on. Every step is exact (a power-of-two step, integers under 2^24), so
// the FPU, VU0 and a host agree under any rounding mode.

#include <cstdint>
#include <cstring>
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

namespace Tyra {
namespace Compact {
namespace {

constexpr float kLimit = 536870912.0f;  // 2^29: quantize's rounding range
constexpr s32 kMaxQ = 32767;

/**
 * The batch quantizer's steps on one coordinate, as VU0 and MMI take them:
 * x = v / step (exact: a power-of-two step), clamped to +-2^29 (VMINI / VMAX -
 * FTOI0 saturates at 2^31, and the PSUBW after it would wrap), half away from
 * zero as trunc(2x) - trunc(x) (two FTOI0, PSUBW), less the origin's index
 * (PSUBW), clamped to +-32767 (PMINW / PMAXW). A host's NaN reads 0 as
 * quantize's does; on the EE the FPU and VU0 have no NaN, both take the
 * pattern as a large number.
 */
s16 quantizeLane(float v, float inv, s32 origin) {
  if (!(v == v)) return 0;
  float x = v * inv;
  x = x < kLimit ? x : kLimit;
  x = x > -kLimit ? x : -kLimit;
  s32 q = static_cast<s32>(x + x) - static_cast<s32>(x) - origin;
  q = q < kMaxQ ? q : kMaxQ;
  q = q > -kMaxQ ? q : -kMaxQ;
  return static_cast<s16>(q);
}

}  // namespace

void quantizePositions(const float* in, u32 n, const Box& box, u8* out,
                       u32 outStride) {
  if (n == 0) return;
  u32 row[4];
  rowFor(box, row);  // round(origin / step) - the index quantize subtracts
  const float inv = 1.0f / box.step;
#if defined(_EE)
  // The asm loads with lqc2 (16-byte aligned input) and stores halfwords with
  // sh, which faults on an odd address: an odd `out` or stride takes C++.
  if ((reinterpret_cast<uintptr_t>(in) & 15) == 0 &&
      ((reinterpret_cast<uintptr_t>(out) | outStride) & 1) == 0) {
    alignas(16) const float consts[4] = {inv, kLimit, -kLimit, 0.0f};
    alignas(16) const s32 ints[12] = {
        static_cast<s32>(row[0]), static_cast<s32>(row[1]),
        static_cast<s32>(row[2]), 0, kMaxQ, kMaxQ, kMaxQ, kMaxQ, -kMaxQ, -kMaxQ,
        -kMaxQ, -kMaxQ};
    const float* src = in;
    u8* dst = out;
    u32 left = n;
    asm volatile(
        "lqc2       $vf1, 0x00(%[c])        \n\t"  // 1/step, 2^29, -2^29
        "lq         $8, 0x00(%[k])          \n\t"  // the origin's index
        "lq         $9, 0x10(%[k])          \n\t"  // 32767 x 4
        "lq         $10, 0x20(%[k])         \n\t"  // -32767 x 4
        "1:                                 \n\t"
        "lqc2       $vf2, 0x00(%[src])      \n\t"
        "vmulx.xyz  $vf2, $vf2, $vf1        \n\t"  // x = v / step, exact
        "vminiy.xyz $vf2, $vf2, $vf1        \n\t"  // at most 2^29
        "vmaxz.xyz  $vf2, $vf2, $vf1        \n\t"  // at least -2^29
        "vadd.xyz   $vf3, $vf2, $vf2        \n\t"  // 2x, exact
        "vftoi0.xyz $vf2, $vf2              \n\t"  // trunc(x)
        "vftoi0.xyz $vf3, $vf3              \n\t"  // trunc(2x)
        "qmfc2      $11, $vf3               \n\t"
        "qmfc2      $12, $vf2               \n\t"
        "psubw      $11, $11, $12           \n\t"  // half away from zero
        "psubw      $11, $11, $8            \n\t"  // less the origin's index
        "pminw      $11, $11, $9            \n\t"
        "pmaxw      $11, $11, $10           \n\t"
        "ppach      $11, $0, $11            \n\t"  // x, y, z halfwords
        "sh         $11, 0x0(%[dst])        \n\t"
        "dsrl       $12, $11, 16            \n\t"
        "sh         $12, 0x2(%[dst])        \n\t"
        "dsrl32     $12, $11, 0             \n\t"
        "sh         $12, 0x4(%[dst])        \n\t"
        "addiu      %[src], %[src], 16      \n\t"
        "addu       %[dst], %[dst], %[stride] \n\t"
        "addiu      %[left], %[left], -1    \n\t"
        "bne        %[left], $0, 1b         \n\t"
        : [src] "+r"(src), [dst] "+r"(dst), [left] "+r"(left)
        : [c] "r"(consts), [k] "r"(ints), [stride] "r"(outStride)
        : "$8", "$9", "$10", "$11", "$12", "memory");
    return;
  }
#endif
  for (u32 i = 0; i < n; ++i, in += 4, out += outStride) {
    const s16 q[3] = {quantizeLane(in[0], inv, static_cast<s32>(row[0])),
                      quantizeLane(in[1], inv, static_cast<s32>(row[1])),
                      quantizeLane(in[2], inv, static_cast<s32>(row[2]))};
    std::memcpy(out, q, sizeof(q));
  }
}

void decodePositions(const u8* in, u32 inStride, u32 n, const Box& box,
                     float* out4) {
  if (n == 0) return;
  u32 row[4];
  rowFor(box, row);
  const float step = box.step;
#if defined(_EE)
  // Every vertex but the last: an unaligned doubleword load reads one
  // halfword past z, which for the last one could be past the data.
  if ((reinterpret_cast<uintptr_t>(out4) & 15) == 0 && n > 1) {
    alignas(16) const float consts[4] = {step, 0.0f, 0.0f, 0.0f};
    alignas(16) const s32 ints[4] = {static_cast<s32>(row[0]),
                                     static_cast<s32>(row[1]),
                                     static_cast<s32>(row[2]), 0};
    const u8* src = in;
    float* dst = out4;
    u32 left = n - 1;
    asm volatile(
        "lqc2       $vf1, 0x00(%[c])        \n\t"  // the step
        "lq         $8, 0x00(%[k])          \n\t"  // the origin's index
        "1:                                 \n\t"
        "ldr        $9, 0x0(%[src])         \n\t"  // x, y, z (and a halfword
        "ldl        $9, 0x7(%[src])         \n\t"  //   more), unaligned
        "pextlh     $9, $9, $0              \n\t"  // each to a word's top half
        "psraw      $9, $9, 16              \n\t"  // sign-extended
        "paddw      $9, $9, $8              \n\t"  // + origin: the grid index
        "qmtc2      $9, $vf2                \n\t"
        "vitof0.xyz $vf2, $vf2              \n\t"  // exact: under 2^24
        "vmulx.xyz  $vf2, $vf2, $vf1        \n\t"  // x step, exact
        "vmove.w    $vf2, $vf0              \n\t"  // w = 1
        "sqc2       $vf2, 0x0(%[dst])       \n\t"
        "addu       %[src], %[src], %[stride] \n\t"
        "addiu      %[dst], %[dst], 16      \n\t"
        "addiu      %[left], %[left], -1    \n\t"
        "bne        %[left], $0, 1b         \n\t"
        : [src] "+r"(src), [dst] "+r"(dst), [left] "+r"(left)
        : [c] "r"(consts), [k] "r"(ints), [stride] "r"(inStride)
        : "$8", "$9", "memory");
    in += (n - 1) * inStride;
    out4 += (n - 1) * 4;
    n = 1;
  }
#endif
  for (u32 i = 0; i < n; ++i, in += inStride, out4 += 4) {
    s16 q[3];
    std::memcpy(q, in, sizeof(q));
    for (u32 a = 0; a < 3; ++a)
      out4[a] = step * static_cast<float>(static_cast<s32>(row[a]) + q[a]);
    out4[3] = 1.0f;
  }
}

namespace {

constexpr u32 kEdges = 24, kSweep = 72, kRows = kEdges + kSweep;

/**
 * Positions for one box: the rounding edges (a half step either way, 2^-25
 * of a step below one, a step and a half), saturation (+-32 768 steps out,
 * 1e30), -0, a denormal - then a sweep of quarter steps. Numbers on every
 * platform only: the EE's FPU and VU0 have no NaN or infinity, and an
 * emulator's clamping decides what such a pattern becomes.
 */
void fillPositions(const Box& box, float rows[kRows][4]) {
  const float s = box.step;
  const float edges[kEdges] = {0.0f,
                               0.5f * s,
                               -0.5f * s,
                               (0.5f - 0x1p-25f) * s,
                               -(0.5f - 0x1p-25f) * s,
                               1.5f * s,
                               -1.5f * s,
                               2.5f * s,
                               box.origin[0] + 32767.0f * s,
                               box.origin[0] - 32767.0f * s,
                               box.origin[1] + 32768.0f * s,
                               box.origin[1] - 32768.0f * s,
                               box.origin[2] + 32767.5f * s,
                               box.origin[2] - 32767.5f * s,
                               box.origin[0] + (0.5f - 0x1p-25f) * s,
                               box.origin[1] - 0.5f * s,
                               1e30f,
                               -1e30f,
                               -0.0f,
                               1e-40f,
                               -1e-40f,
                               box.origin[2],
                               1000.0f,
                               -1000.0f};
  for (u32 i = 0; i < kRows; ++i) {
    for (u32 a = 0; a < 3; ++a) {
      if (i < kEdges) {
        rows[i][a] = edges[(i + a * 7) % kEdges];
      } else {
        // Quarter steps across the box, exact: integers under 2^24 x s / 4.
        const s32 quarter = static_cast<s32>(((i * 7919u + a * 104729u) % 262139u)) - 131069;
        rows[i][a] = box.origin[a] + static_cast<float>(quarter) * (0.25f * s);
      }
    }
    rows[i][3] = 1.0f;
  }
}

u32 batchMismatches() {
  const Box boxes[3] = {{{-32.0f, 8.0f, 0.0f}, 0x1p-9f},
                        {{0.25f, -1.0f, 3.0f}, 0x1p-14f},
                        {{1000.0f, 0.0f, -1000.0f}, 0x1p-6f}};
  u32 bad = 0;
  for (const Box& box : boxes) {
    alignas(16) float rows[kRows][4];
    fillPositions(box, rows);
    // The quantizer: a V3-16 run, a V4-16 one (the halfword after z stays as
    // it was), and an input off the 16-byte grid (the per-lane path).
    for (u32 stride = 6; stride <= 8; stride += 2) {
      u8 out[kRows * 8 + 2];
      std::memset(out, 0x5A, sizeof(out));
      quantizePositions(rows[0], kRows, box, out + 2, stride);
      for (u32 i = 0; i < kRows; ++i) {
        s16 q[3];
        std::memcpy(q, out + 2 + i * stride, sizeof(q));
        for (u32 a = 0; a < 3; ++a)
          if (q[a] != quantize(rows[i][a], box.origin[a], box.step)) ++bad;
        if (stride == 8 && (out[2 + i * 8 + 6] != 0x5A || out[2 + i * 8 + 7] != 0x5A))
          ++bad;
      }
      if (out[0] != 0x5A || out[1] != 0x5A) ++bad;
    }
    {
      alignas(16) float shifted[kRows * 4 + 1];
      std::memcpy(shifted + 1, rows, sizeof(rows));
      u8 out[kRows * 6];
      quantizePositions(shifted + 1, kRows, box, out, 6);
      for (u32 i = 0; i < kRows; ++i) {
        s16 q[3];
        std::memcpy(q, out + i * 6, sizeof(q));
        for (u32 a = 0; a < 3; ++a)
          if (q[a] != quantize(rows[i][a], box.origin[a], box.step)) ++bad;
      }
    }
    // The decoder over what the quantizer wrote, saturated ends included:
    // the grid point - step x the grid index, what the VU computes - bit for
    // bit, and dequantize's value (a host rounding downward makes its exact
    // zero -0); w = 1. Into a 16-byte aligned array (the VU0 path) and one
    // off it.
    u8 packed[kRows * 6];
    quantizePositions(rows[0], kRows, box, packed, 6);
    u32 row[4];
    rowFor(box, row);
    alignas(16) float decoded[kRows * 4 + 1];
    for (u32 offset = 0; offset < 2; ++offset) {
      std::memset(decoded, 0, sizeof(decoded));
      decodePositions(packed, 6, kRows, box, decoded + offset);
      for (u32 i = 0; i < kRows; ++i) {
        s16 q[3];
        std::memcpy(q, packed + i * 6, sizeof(q));
        const float* d = decoded + offset + i * 4;
        for (u32 a = 0; a < 3; ++a) {
          const float want =
              box.step * static_cast<float>(static_cast<s32>(row[a]) + q[a]);
          if (std::memcmp(&d[a], &want, sizeof(want)) != 0) ++bad;
          if (d[a] != dequantize(q[a], box.origin[a], box.step)) ++bad;
        }
        if (d[3] != 1.0f) ++bad;
      }
    }
  }
  return bad;
}

}  // namespace

u32 selfTest() { return knownAnswers() + batchMismatches(); }

}  // namespace Compact
}  // namespace Tyra
