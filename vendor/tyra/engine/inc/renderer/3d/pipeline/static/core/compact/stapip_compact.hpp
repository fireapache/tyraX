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

// Added by TyraX: the compact vertex format (docs/compact-vertex-format.md).
// A static-pipeline vertex is stored in 16-bit fixed point (8-bit where that
// is exact) in the layout the VIF unpacks, and widened to 32-bit lanes while it
// streams into VU1 memory. This header is the format itself - the unpack
// formats, the boxes positions are quantized in, the quantizers, and the VIF
// words a run of one attribute carries. It includes only tamtypes.h, so the
// editor and the host verifiers use exactly what the engine uses.
#pragma once

#include <tamtypes.h>

namespace Tyra {
namespace Compact {

/** The attributes a compact stream carries, one run each per package. */
enum class Attr : u8 {
  Position,   // int16 x, y, z in the bag's box; w = 1.0 from the ROW register
  St,         // int16 u, v, 4.12 fixed point; z = w = 1.0 from ROW
  Color,      // u8 r, g, b, a - what the GS takes, no precision lost
  Normal,     // int8 x, y, z (x 127), w = 0
  EnvNormal,  // an env-map normal in the ST slot: int8 x 127, like Normal
  BbCenter,   // a billboard's centre: a position
  BbParams,   // a billboard's 2x2 basis weights: int16 x 4
  BbColor     // a billboard's colour: u8 x 4
};

/** The UNPACK formats, as the `vn << 2 | vl` nibble of the UNPACK command. */
enum class Format : u8 { V2_16 = 0x5, V3_16 = 0x9, V4_16 = 0xD, V4_8 = 0xE };

/** Positions are V3-16 (6 bytes) with w from the ROW register. V4-16 (8 bytes,
 * the fourth halfword unused, w still from ROW) is the measured alternative:
 * the VU programs are the same either way, so this constant is the switch. */
constexpr Format kPositionFormat = Format::V3_16;

Format formatOf(Attr a);
/** Signed unpack (sign extension; USN clear) - everything but the colours. */
bool isSigned(Attr a);
/** True when some lanes come from the ROW register (positions: w; ST: z, w). */
bool masked(Attr a);
/** Bytes one element takes in the stream. */
u32 elementBytes(Attr a);
/** A run of `n` elements: its bytes, and the quadwords it takes padded to a
 * whole quadword. The padding is zero - VIF NOPs, the words after the unpack. */
u32 runBytes(Attr a, u32 n);
u32 runQwords(Attr a, u32 n);

/** The VIF ROW register every compact bag uses: (0, 0, 1.0f, 1.0f). A masked
 * lane takes its ROW value - w of a position, z and w of an ST (the programs
 * multiply the whole ST by Q, so z must be 1.0). */
constexpr u32 kRow[4] = {0u, 0u, 0x3F800000u, 0x3F800000u};

/** The STMASK word of an attribute: 2 bits a lane (0 = data, 1 = ROW), the same
 * pattern in all four write cycles. 0 for an unmasked attribute. */
u32 maskWord(Attr a);

/**
 * The UNPACK VIFcode for `num` elements of `a` to VU address `dest` (10 bits),
 * relative to TOPS when `useTop`. Built by hand: the SDK's unpack helpers
 * always set USN (unsigned), and every signed attribute needs it clear. Bit 14
 * is USN, bit 15 FLG (add TOPS); `num` 256 is encoded as 0.
 */
u32 unpackCode(Attr a, u32 num, u32 dest, bool useTop);

/**
 * The header quadword in front of a run: `{STMASK, mask, STCYCL 1/1, UNPACK}`
 * for a masked attribute, `{NOP, NOP, STCYCL 1/1, UNPACK}` otherwise - the
 * same quadword a baked block already spends on two NOPs, the STCYCL and the
 * UNPACK. The UNPACK targets the double buffer (TOPS).
 */
void runHeader(Attr a, u32 num, u32 dest, u32 out[4]);

/**
 * The box a bag's positions are quantized in: isotropic (one step for x, y and
 * z, so a uniform scale never bends a normal), its origin a whole number of
 * steps. A vertex is stored as round(v / step) - origin / step, its index on
 * the step's grid less the origin's - which is what makes a vertex that two
 * world boxes share decode to the same float in both (the seam rule).
 */
struct Box {
  float origin[3];
  float step;
};

/** The scene step: the smallest power of two, at least 2^-10 m, at which a
 * world bag of `largestWorldExtent` metres fits wherever it lies (65 533 steps:
 * a box holds 65 534, and a bag that starts off the grid may need one more). */
float sceneStepFor(float largestWorldExtent);

/**
 * A world bag's box (its model matrix is the identity): `sceneStep` (a power of
 * two - sceneStepFor) and an origin on its grid, so every world bag of the
 * scene quantizes a shared vertex to the same grid point. A bag wider than
 * 65 534 steps takes the next coarser power of two that holds it, for itself
 * only, and sets `*coarse` (the caller says so in its log).
 */
Box worldBox(const float mn[3], const float mx[3], float sceneStep,
             bool* coarse);

/** Any other bag's box - one per model, every part and LOD tier of it: step =
 * the largest extent / 65 534 (a 2 m body 0.03 mm), the origin a whole number
 * of steps near the box's centre. Round trip within half a step while the
 * model lies within 2^24 steps of its own origin. */
Box modelBox(const float mn[3], const float mx[3]);

/** The dequantization matrix, column-major like M4x4 (translation in
 * [12..14]): D x (q, 1) = origin + step * q. The engine uploads MVP x D. */
void dequant(const Box& box, float colMajor[16]);

/**
 * One coordinate in a box: round(v / step) - round(origin / step), rounding
 * half away from zero, saturated to +-32767 (symmetric). With a power-of-two
 * step every operation is exact, so the result is the same on every platform
 * and under every float rounding mode - the EE's FPU and VU0 round toward
 * zero, a host to nearest. NaN reads 0.
 */
s16 quantize(float v, float origin, float step);
/** origin + step * q: exact for a world box. */
float dequantize(s16 q, float origin, float step);

/** ST in 4.12 fixed point (+-8 repeats at 1/4096), saturated to +-32767. */
s16 quantizeSt(float v);
/** The whole number of repeats to subtract from one coordinate (u or v) of a
 * package's `n` ST values so they land in [-8, 8): 0 when they already do,
 * else the floor of their middle. Wrap addressing makes the shift invisible
 * (a package is drawn alone); values that still do not fit saturate. */
float stRebase(const float* uv, u32 n);
/** A colour channel, 0..255 rounded and clamped. NaN reads 0. */
u8 quantizeColor(float v);
/** A normal component x 127, rounded and clamped to +-127. */
s8 quantizeNormal(float v);

/**
 * Known answers that hold under any float rounding mode (the quantizers' edge
 * cases, the unpack words, the boxes), and every batch path against the
 * per-element routines here; the count of mismatches, 0 = pass. A game can run
 * it on the console to prove the EE agrees with the host that wrote its data.
 */
u32 selfTest();

}  // namespace Compact
}  // namespace Tyra
