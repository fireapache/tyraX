#pragma once

// Compact skinning: the skin core shared by the host verifier
// (tools/verify-skel-compact.cpp) and the EE path (docs/animated-models.md,
// "Skinning into compact streams"). It is plain scalar C++ - the reference the
// VU0 / MMI path must match bit for bit.
//
// A bind corner is stored as its grid index in the model's box (q, the
// .tskl 3 position) and a skinned corner is written as its index in the output
// box, both power-of-two steps. The output index is affine in the bind index:
//   out = Qout x (sum_k w_k M_k) x Dbind x (q, 1),
// with Dbind = D(bind step, origin index) and Qout = (1 / outStep, -origin
// index of the output box). So the output box is folded into each palette
// matrix once per pose (foldPalette), and a corner is a weighted sum of four
// folded matrices applied to its index - no per-corner float decode or encode.

// Skinning into a compact stream. A skinned part's corners are its stream's
// vertices, one for one: corner v is vertex v, so it lives in package
// v / packageSize at element v % packageSize (StaPipCompactStream::run and
// packageVertices). Its position is the int16 index in the stream's box -
// stored at run(p, 0) + k x 6. The box is per POSE: its origin is the grid point
// at the centre of this pose's own extent (skinIntoStream re-boxes the stream),
// and its step is the one skinOutputBox names (a power of two). Its normal is
// the int8 x127 triple at run(p, normal) +
// k x 4 (a lit or an env layout; the env normal is unit length already). The
// ST and colour runs are not touched by a pose: the stream is packed once
// (Compact::packArrays) from the model's own texture coordinates and colours,
// and only positions and normals are rewritten per pose. A corner that repeats
// an earlier one (the bind data's skinSource) copies that corner's written
// elements instead of skinning again - the source is always an earlier vertex,
// so its package and element are already this pose's. After the last corner
// the stream's own finish() recomputes the part boxes, the coarse groups, the
// whole-stream box and the content version: nothing else keeps a box.

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"

namespace Tyra {

/** The output box is centred on the model origin with a half-size of this
 * many times the model's R (the largest distance from the origin to a corner
 * of the pose or bind box). Raised only on evidence of out-of-box vertices. */
constexpr float kSkinBoxFactor = 1.25F;

/**
 * The box a skinned model writes its corners in: origin 0 (the model origin),
 * step the smallest power of two whose int16 range (+-32767 steps) holds
 * kSkinBoxFactor x R. A power-of-two step is the format rule - every grid
 * index is then exact and the output seams match the other boxes.
 */
Compact::Box skinOutputBox(const SkelModel& m);

namespace SkelSkin {

constexpr u32 kMaxSlots = 64;

/** A part's pose-independent extent, computed once with its bind data (bindExtent):
 * the index box [qmin, qmax] of its bind corners, and the palette slots its corners
 * weight (bit j, for j < kMaxSlots). A pose's box then costs eight corners per used
 * slot and needs no pass over the corners. */
struct SkelSkinExtent {
  s32 qmin[3];
  s32 qmax[3];
  unsigned long long used;
};

/** Computes a part's extent from its bind indices (x, y, z per corner), joints and
 * weights (four per corner). */
void bindExtent(const s16* pos, const u8* joints, const u8* weights, u32 count,
                SkelSkinExtent& out);

/** The palette folded for one pose: f = Qout x M_k x Dbind (4x4, column-major,
 * applied to the bind index (q, 1)), and r = the rotation part of M_k (3x3,
 * column-major, applied to a normal). */
struct Folded {
  float f[kMaxSlots][16];
  float r[kMaxSlots][9];
  Compact::Box out;  // the output box the folds were made for
  u32 slots = 0;     // the palette slots folded (at most kMaxSlots)
};

/**
 * Folds `slots` palette matrices (column-major 4x4, global x inverse bind, as
 * SkelJoint evaluates them) for a bind box and an output box.
 */
void foldPalette(const float* palette, u32 slots, const Compact::Box& bind,
                 const Compact::Box& out, Folded& folded);

/**
 * Skins one corner. `pos` is the bind index x, y, z (and a fourth lane the
 * caller ignores), `nrm` the bind normal x, y, z x 127, `joints` and `weights`
 * four palette slots and their u8 weights (divided by their sum). Writes the
 * output index x, y, z (fourth lane 0) and the normal x, y, z x 127 (fourth 0).
 * Positions round half away from zero and clamp to +-32767; a normal
 * renormalises before it is written. Returns 1 when any position axis was
 * clamped (a corner out of its box), else 0.
 */
void blendCorner(const Folded& folded, const s16* pos, const s8* nrm,
                 const u8* joints, const u8* weights, float px[3], float nn[3]);

u32 skinCorner(const Folded& folded, const s16* pos, const s8* nrm,
               const u8* joints, const u8* weights, const s32* originIndex,
               s16* outPos, s8* outNrm);

/** A skinned part's bind data at one LOD level (the arrays of SkelBindLod). */
namespace Fixed {
struct CornerInfl;
}

/** The scatter map of one part for one stream package size: each unique corner's
 * destinations (every corner whose blend is its own: package << 16 | element), and
 * the stream parts those destinations fall in. Built once with the bind data. */
struct SkinScatter {
  u32 size = 0;                  // the stream package size it was built for (0 = none)
  std::vector<u32> dstOff, dst;  // [count + 1], then the destinations of each corner
  std::vector<u32> rngOff, rng;  // [count + 1], then the distinct stream parts of each corner
};

/** Fills `map` for a part whose corners carry `skinSource` (first identical corner) in
 * stream packages of `packageSize` corners. */
void buildScatter(const u32* skinSource, u32 count, u32 packageSize, SkinScatter& map);

struct SkelSkinPart {
  const s16* pos;         // x, y, z per corner: the bind index in the model box (3 a corner)
  const s8* nrm;          // x, y, z, 0 per corner, x 127
  const u8* joints;       // four palette slots a corner
  const u8* weights;      // their u8 weights, four a corner
  const u32* skinSource;  // the first identical corner (<= this one)
  u32 count;              // corners = the stream's vertices
  // The part's extent (bindExtent) when the bind data carries it; null means
  // posePlace computes it.
  const SkelSkinExtent* extent = nullptr;
  // The corners' precomputed influences (Fixed::prepCorner), count entries; null
  // computes them per corner.
  const Fixed::CornerInfl* infl = nullptr;
  // The scatter map (buildScatter), rebuilt when the stream's package size differs; null
  // builds one for the call.
  SkinScatter* scatter = nullptr;
};

/** Returned by skinIntoStream when the stream cannot take the part (not
 * allocated, another vertex count or layout, or another box than the folds'). */
constexpr u32 kSkinRefused = 0xFFFFFFFFu;

/**
 * Skins a part into `out` - a stream packed from the same part (packArrays,
 * the layout with a position first and a normal, the folds' step). The output
 * box is per POSE: the stream is re-boxed (rebox) on the grid point at the
 * centre of the pose's own extent, so a body flung metres off the model origin
 * is held at its own place, and only a pose spanning more than the box's 8 m
 * clamps. Writes positions and normals per corner and finishes the stream (its
 * boxes and content version). Returns the corners with a position clamped, or
 * kSkinRefused.
 */
u32 skinIntoStream(const Folded& folded, const SkelSkinPart& part,
                   StaPipCompactStream& out);

/**
 * The stream check: a development oracle the gyms switch on (off by default -
 * it costs a decode and a float reference per corner). While enabled, every
 * stream SkelInstance::skinInto writes is compared corner by corner with a
 * float reference skin - the palette applied to the bind positions decoded in
 * the model box - and the largest difference is kept, in the stream's box
 * steps. A correct stream stays within about half a step (rounding); a stride
 * or layout error misses by metres.
 */
struct StreamCheck {
  bool enabled = false;
  u32 passes = 0;           // streams compared
  u32 corners = 0;          // corners compared
  float worstSteps = 0.0F;  // the largest difference, in steps
};
extern StreamCheck gStreamCheck;

/**
 * The vector path (EE): the VU0 blend writes model-space floats into scratch and
 * quantizeIntoStream quantizes them (Compact::quantizePositions / quantizeNormals,
 * the batch paths of stapip_compact_ee.cpp) into the stream. gUseVector selects
 * it; it is on by default and the first pass checks it (checkVectorStream), and
 * the check turns it off when the vector stream is not within one LSB of the
 * scalar core, or any quantize differs from the scalar encode of the same floats.
 */
extern bool gUseVector;
extern bool gVectorChecked;

#ifndef TYRA_SKIN_FIXED
#define TYRA_SKIN_FIXED 1
#endif

/** The fixed-point stream pass (Fixed::fold, Fixed::cornerFixed) reads the compact
 * bind data directly and writes the stream in int16; it replaces the float blend
 * for streamed parts. On by default (TYRA_SKIN_FIXED); 0 keeps the float core. */
extern bool gUseFixed;

/** Skins a part into `out` in fixed point: the box is posePlace's for this pose, the
 * folds are made for it (Fixed::fold, the part's extent for the pre-scale), each unique
 * corner is blended once and repeats copy their source's bytes. `folded` is the
 * pose's float fold (posePlace needs its box). Returns the corners with a position
 * clamped, or kSkinRefused. */
u32 skinIntoFixed(const Folded& folded, const float* palette, u32 slots,
                  const Compact::Box& bindBox, const SkelSkinPart& part,
                  StaPipCompactStream& out);

/** Places the stream's box for this pose (the extent and rebox of skinIntoStream).
 * `oi` is the box's origin in steps. False when the stream refuses the part or the rebox. */
bool posePlace(const Folded& folded, const SkelSkinPart& part, StaPipCompactStream& out,
               s32 oi[3]);

/** Quantizes a part's blended floats (`pos4` and `nrm4`, Vec4 arrays of part.count,
 * model space, 16-byte aligned) into `out`, whose box posePlace set for this pose.
 * Writes the positions and normals and finishes the stream. Returns the corners
 * with a position clamped, or kSkinRefused. */
u32 quantizeIntoStream(const float* pos4, const float* nrm4, const SkelSkinPart& part,
                       StaPipCompactStream& out);

/** The self-check's result: the corners whose vector stream differs from the scalar
 * encode of the same floats (`exact`, must be 0), and the largest difference
 * between the scalar core's stream and that encode (`worst`, in LSB). */
struct VectorCheck {
  u32 exact = 0;
  u32 worst = 0;
  u32 clamped = 0;       // the scalar core's out-of-box count (its stream is what is left)
  u32 boxMismatch = 0;   // part box values the vector pass set that the scalar finish differs from
};

/** Runs the self-check on one pose of one part. `out` holds the vector result on
 * entry and the scalar core's stream on return. `pos4` and `nrm4` are the scratch
 * the vector pass quantized; they are overwritten (the expected bytes). */
VectorCheck checkVectorStream(const Folded& folded, const SkelSkinPart& part,
                              StaPipCompactStream& out, float* pos4, float* nrm4);

/**
 * The fixed-point skin: every value a corner touches is int16 (or a 16x16 product
 * of two of them), from the compact bind indices to the stream. The palette is
 * folded once per pose into int16 matrices (fold); a corner is then a weighted
 * blend of its four joints in 16-bit fixed point (cornerFixed). Positions are
 * accurate to a step or two of the output box; normals to two x127 units after
 * renormalisation (the reciprocal square root is a table of 768 entries).
 */
namespace Fixed {

/** The folded palette for one pose. A_j = R_j x (bind step / output step) as
 * int16 in Q(15 - expA) (column-major, a[j][c * 3 + r]); B_j = the joint's
 * translation in the output index domain, Q8 steps (b[j][r]); R_j in Q(15 - expR)
 * for normals (r[j][c * 3 + r]). The exponents are pose-global: the smallest that
 * holds the largest entry. */
struct Fold {
  s16 a[kMaxSlots][9];
  s32 b[kMaxSlots][3];
  s16 r[kMaxSlots][9];
  s32 expA = 0;
  s32 expR = 0;
  s32 expT = 0;              // the blend's pre-scale: a joint term is in units of 2^-expT steps
  bool narrow = false;       // every term fits 32-bit arithmetic (the fast blend; else the 64-bit one)
  // Every joint term fits int16 for every bind index up to qMax, with the quantisation of
  // the folded entries included: the per-term clamp and its counter cannot fire.
  bool noSat = false;
  u32 slots = 0;
  s32 oiOut[3] = {0, 0, 0};  // the output box origin, in output steps
};

/** Folds `slots` palette matrices (column-major 4x4) for a bind box and an output box.
 * `qMax` bounds |q| over the bind indices the folds will see (the part's extent);
 * it sets the blend's pre-scale so a joint term fits int16. */
void fold(const float* palette, u32 slots, const Compact::Box& bind, const Compact::Box& out,
          Fold& f, u32 qMax = 32767, unsigned long long used = ~0ULL);

/** Skins one corner in fixed point. `q` is the bind index (x, y, z), `n` the bind
 * normal (x127), `joints` and `weights` four slots and their u8 weights (normalised
 * to a Q15 sum of 32768). Writes the output index x, y, z (outPos[3] = 0) and the
 * normal x127 (outNrm[3] = 0). Returns 1 when a position axis clamped to +-32767,
 * else 0. `blendSaturations` (may be null) counts the joint terms that did not fit
 * the int16 pre-scale (fold sets it so that a term fits for every bind index up to
 * qMax); a non-zero count means the corner is not within the tolerance. */
/** A corner's influences as the blend reads them: the nonzero Q15 weights in slot
 * order, with their joints (n of them, 0..4). Built once from the bind data
 * (prepCorner) - the weights depend on the bind data alone, not on the pose. */
struct CornerInfl {
  u16 w[4];
  u8 j[4];
  u8 n;
};

/** The influences of one corner (prepCorner). Integer sums are order-free, so the
 * blend over these equals the blend over the four raw slots (cornerNarrowRef). */
CornerInfl prepCorner(const u8* joints, const u8* weights);

/** The 32-bit corner blend as it was before the precomputed influences: the host
 * reference the optimised cornerFixed is checked against (bit-equal outputs and
 * saturation counts, tools/verify-skel-fixed). */
u32 cornerNarrowRef(const Fold& f, const s16* q, const s8* n, const u8* joints, const u8* weights,
                    s16* outPos, s8* outNrm, u32* blendSaturations);

/** `infl` (optional) is the corner's precomputed influences (prepCorner of the same
 * joints and weights); null computes them here. */
u32 cornerFixed(const Fold& f, const s16* q, const s8* n, const u8* joints, const u8* weights,
                s16* outPos, s8* outNrm, u32* blendSaturations, const CornerInfl* infl = nullptr);

}  // namespace Fixed

#ifndef TYRA_SKIN_PROBE
#define TYRA_SKIN_PROBE 1
#endif

#if TYRA_SKIN_PROBE
/** Attribution probe (compiled out by default): COP0 Count ticks per phase of
 * the skin pass, summed over every body and pose since the last report. The
 * phase counters are ticks; `bodies` and `folds` count the calls. */
struct Probe {
  unsigned fold = 0;     // foldPalette (once per pose)
  unsigned extent = 0;   // per-pose extent pass (blendCorner per corner)
  unsigned box = 0;      // the box from the extent, rebox
  unsigned corners = 0;  // per-corner skin + encode + repeat copies
  unsigned finish = 0;   // StaPipCompactStream::finish (part boxes, version)
  unsigned quant = 0;   // quantizeIntoStream (the vector path's quantize)
  unsigned bodies = 0;   // skinIntoStream calls that wrote
  unsigned folds = 0;
  unsigned entries = 0;   // skinIntoStream calls (wrote or refused)
  unsigned refused = 0;   // skinIntoStream calls refused
  unsigned calls = 0;     // SkelInstance::skinInto calls that reached a pass
  unsigned noTarget = 0;  // parts skinInto skipped: no spare stream or no bind
  unsigned reported = 0;  // the calls count at the last report
  unsigned early = 0;     // skinInto returns before any pass (no model, no streams, too many slots)
  unsigned paletteSize = 0;
  // The fixed-point pass (skinIntoFixed), summed over the parts since the last report.
  unsigned fixParts = 0;     // parts written
  unsigned fixUnique = 0;    // corners blended (not repeats)
  unsigned fixRepeat = 0;    // corners copied from their source
  unsigned fixPlace = 0;     // posePlace and the pose extent
  unsigned fixFold = 0;      // Fixed::fold (per part)
  unsigned fixWeights = 0;   // per unique corner: the weights' Q15 setup
  unsigned fixPos = 0;       // per unique corner: the position blend (decode, multiply, sum, round, clamp)
  unsigned fixNrm = 0;       // per unique corner: the normal blend and renormalise
  unsigned fixLoop = 0;      // the whole corner loop (unique + repeats + stores)
  unsigned fixFinish = 0;    // StaPipCompactStream::finish
};
extern Probe gProbe;
/** Logs the probe's means every 20 calls (called by SkelInstance::skinInto). */
void probeReport();
#endif

}  // namespace SkelSkin
}  // namespace Tyra
