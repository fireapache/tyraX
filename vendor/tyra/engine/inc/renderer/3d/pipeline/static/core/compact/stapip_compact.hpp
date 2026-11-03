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
// words a bag and a run of one attribute carry. It includes only tamtypes.h,
// so the editor and the host verifiers use exactly what the engine uses.
#pragma once

#include <tamtypes.h>

namespace Tyra {
namespace Compact {

/** The attributes a compact stream carries, one run each per package. */
enum class Attr : u8 {
  Position,   // int16 x, y, z in the bag's box (+ its origin index, ROW);
              // w = 1.0 from COL
  St,         // int16 u, v, 4.12 fixed point; z = w = 1.0 from COL
  Color,      // u8 r, g, b, a - what the GS takes, no precision lost
  Normal,     // int8 x, y, z (x 127), w = 0
  EnvNormal,  // an env-map normal in the ST slot: int8 x 127, like Normal
  BbCenter,   // a billboard's centre: a position
  BbParams,   // a billboard's 2x2 basis weights: int16 x 4
  BbColor     // a billboard's colour: u8 x 4
};

/** The UNPACK formats, as the `vn << 2 | vl` nibble of the UNPACK command. */
enum class Format : u8 { V2_16 = 0x5, V3_16 = 0x9, V4_16 = 0xD, V4_8 = 0xE };

/** Positions are V3-16 (6 bytes) with w from the COL register. V4-16 (8 bytes,
 * the fourth halfword unused, w still from COL) is the measured alternative:
 * the VU programs are the same either way, so this constant is the switch. */
constexpr Format kPositionFormat = Format::V3_16;

Format formatOf(Attr a);
/** Signed unpack (sign extension; USN clear) - everything but the colours. */
bool isSigned(Attr a);
/** True when some lanes come from the COL register (positions: w; ST: z, w). */
bool masked(Attr a);
/** Bytes one element takes in the stream. */
u32 elementBytes(Attr a);
/** A run of `n` elements: its bytes, and the quadwords it takes padded to a
 * whole quadword. The padding is zero - VIF NOPs, the words after the unpack. */
u32 runBytes(Attr a, u32 n);
u32 runQwords(Attr a, u32 n);

/** The VIF COL register every compact bag sets: 1.0f in all four. A masked
 * lane takes COL of its write cycle - cycle 0 under STCYCL 1/1 - so a
 * position's w and an ST's z and w are 1.0 (the programs multiply the whole ST
 * by Q, so z must be 1.0). */
constexpr u32 kCol[4] = {0x3F800000u, 0x3F800000u, 0x3F800000u, 0x3F800000u};

/** True for the attributes unpacked in offset MODE (STMOD 1): the VIF adds
 * the ROW register - the box's origin index - to each data lane as an integer,
 * so VU memory holds the vertex's index on the step's grid, not its offset in
 * the box. Positions and billboard centres. */
bool offset(Attr a);

/** The STMASK word of an attribute: 2 bits a lane (0 = data, 2 = COL), the same
 * pattern in all four write cycles: 0x80808080 for a position (w), 0xA0A0A0A0
 * for an ST (z, w). 0 for an unmasked attribute. */
u32 maskWord(Attr a);

/**
 * The UNPACK VIFcode for `num` elements of `a` to VU address `dest` (10 bits),
 * relative to TOPS when `useTop`. Built by hand: the SDK's unpack helpers
 * always set USN (unsigned), and every signed attribute needs it clear. Bit 14
 * is USN, bit 15 FLG (add TOPS); `num` 256 is encoded as 0.
 */
u32 unpackCode(Attr a, u32 num, u32 dest, bool useTop);

/**
 * The header quadword in front of a run - four words, the MODE in word 2:
 *   position, billboard centre: {STMASK, mask, STMOD 1, UNPACK}
 *   ST:                         {STMASK, mask, STMOD 0, UNPACK}
 *   unmasked attributes:        {NOP,    NOP,  STMOD 0, UNPACK}
 * The same quadword a baked block spends on two NOPs, the STCYCL and the
 * UNPACK: STCYCL 1/1 moves to the bag's setup (bagSetup), and its slot sets
 * the MODE, because MODE applies to every unpack, masked or not - offset on a
 * position run, normal on every other. The UNPACK targets the double buffer
 * (TOPS).
 */
void runHeader(Attr a, u32 num, u32 dest, u32 out[4]);

/**
 * The box a bag's positions are quantized in: isotropic (one step for x, y and
 * z, so a uniform scale never bends a normal), the step a power of two and the
 * origin a whole number of steps. A vertex is stored as its index on the step's
 * grid less the origin's index, round(v / step) - round(origin / step); the VIF
 * adds the origin's index back (offset MODE, the ROW register - rowFor), so VU
 * memory holds the grid index itself, and every box on one grid hands the VU
 * the same integer for one vertex.
 */
struct Box {
  float origin[3];
  float step;
};

/** The scene step: the smallest power of two, at least 2^-10 m, at which a
 * world bag of `largestWorldExtent` metres fits wherever it lies. A box spans
 * 65 534 steps; its ends' grid points can be 65 535 apart only when both are
 * exact half steps either side of 0 (rounding away from zero moves each out by
 * half a step), so the step holds 65 533 and never needs that case. */
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

/** Any other bag's box - one per model, every part and LOD tier of it: the
 * step is the smallest power of two at or above the largest extent / 65 534 (a
 * 2 m body 2^-14 m, 0.061 mm), the origin a whole number of steps near the
 * box's centre. A model further than 2^24 steps from its own origin takes
 * coarser steps until it is not, so every grid index is an exact float. */
Box modelBox(const float mn[3], const float mx[3]);

/** The ROW words of a box: its origin's grid index per axis (origin / step,
 * a whole number under 2^24 in magnitude), as 32-bit integers, and 0 in w. In
 * offset MODE the VIF adds them to a position's sign-extended lanes. */
void rowFor(const Box& box, u32 out[4]);

/** The VIF words a compact bag sends once, in front of everything else it
 * unpacks (its object data included):
 *   {STCYCL 1/1, STMOD 0, STROW, row x 4, STCOL, col x 4}
 * - twelve words, three quadwords. The STMOD 0 matters: MODE stays offset after
 * a position run, and the unpacks that follow one in another bag or package
 * (object data, a package's buffer header) must run in normal MODE. */
constexpr u32 kBagSetupWords = 12;
void bagSetup(const Box& box, u32 out[kBagSetupWords]);

/** The dequantization matrix D(step, 0), column-major like M4x4: the step on
 * the diagonal and no translation - the origin reaches VU memory through ROW,
 * as an index. D x (index, 1) = step * index. Every world bag of a scene shares
 * one, so the engine uploads one MVP x D for all of them, and a vertex two bags
 * share is the same index under the same matrix: the same clip-space position,
 * bit for bit (the seam rule). A model bag uploads its own model's MVP x D. */
void dequant(float step, float colMajor[16]);

/**
 * One coordinate in a box: round(v / step) - round(origin / step), rounding
 * half away from zero, saturated to +-32767 (symmetric). With a power-of-two
 * step every operation is exact, so the result is the same on every platform
 * and under every float rounding mode - the EE's FPU and VU0 round toward
 * zero, a host to nearest. NaN reads 0.
 */
s16 quantize(float v, float origin, float step);
/** origin + step * q - the grid point, exact while the index is under 2^24. */
float dequantize(s16 q, float origin, float step);

/** ST in 4.12 fixed point (+-8 repeats at 1/4096), saturated to +-32767. */
s16 quantizeSt(float v);
/** The whole number of repeats to subtract from one coordinate (u or v) of a
 * package's `n` ST values so they land in [-8, 8): 0 when they already do,
 * else the floor of their middle. Wrap addressing makes the shift invisible
 * (a package is drawn alone); values that still do not fit saturate. */
float stRebase(const float* uv, u32 n);
/** The same for values `stride` floats apart: a bag's ST array is Vec4s
 * (stride 4, `uv` at the u or the v of the first), a writer's (u, v) pairs
 * stride 2. */
float stRebase(const float* uv, u32 n, u32 stride);
/** A colour channel, 0..255 rounded and clamped. NaN reads 0. */
u8 quantizeColor(float v);
/** A normal component x 127, rounded and clamped to +-127. */
s8 quantizeNormal(float v);

/** A billboard's basis weight, stored x `scale` (a power of two - the
 * programs fold its inverse into the uploaded basis), rounded and saturated
 * to +-32767. */
s16 quantizeWeight(float v, float scale);
/** The largest power of two that keeps weights of magnitude up to
 * `largestWeight` inside +-32767, between 2^-16 and 2^24 (2^12 for none). */
float weightScaleFor(float largestWeight);

/**
 * The batch paths (stapip_compact_ee.cpp): VU0 macro mode and MMI on the EE,
 * the same steps in C++ elsewhere - bit-identical to the per-element
 * functions on the platform they run on (selfTest compares them).
 *
 * quantizePositions: `n` positions 4 floats apart (Vec4s; a 16-byte aligned
 * array takes the VU0 path) quantized in `box` exactly as `quantize` does each
 * coordinate, written as x, y, z int16 `outStride` bytes apart (6 for a V3-16
 * run, 8 for V4-16; the bytes after z are not written).
 *
 * decodePositions: `n` positions stored x, y, z int16 `inStride` bytes apart,
 * out as 4 floats a vertex (x, y, z, 1; a 16-byte aligned `out4` takes the VU0
 * path): step x (q + the origin's index) - the grid point the VU sees, which
 * is `dequantize` exactly for a box whose origin is on its grid.
 */
void quantizePositions(const float* in, u32 n, const Box& box, u8* out,
                       u32 outStride);
void decodePositions(const u8* in, u32 inStride, u32 n, const Box& box,
                     float* out4);

/**
 * Known answers that hold under any float rounding mode - the quantizers'
 * edge cases, the VIF words, the boxes; the count of mismatches, 0 = pass.
 * Part of selfTest; on its own it is what links with this file alone (the
 * editor).
 */
u32 knownAnswers();

/**
 * knownAnswers, plus the batch paths against the per-element functions on
 * inputs that are numbers on every platform (stapip_compact_ee.cpp): the count
 * of mismatches, 0 = pass. A game can run it on the console to show the EE -
 * its FPU, VU0 and MMI paths - agrees with the host that wrote its data.
 */
u32 selfTest();

}  // namespace Compact
}  // namespace Tyra
