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
// packageVertices). Its position is the int16 index in the stream's box - the
// output box, which is also the box skinOutputBox names - stored at
// run(p, 0) + k x 6; its normal is the int8 x127 triple at run(p, normal) +
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

/** The palette folded for one pose: f = Qout x M_k x Dbind (4x4, column-major,
 * applied to the bind index (q, 1)), and r = the rotation part of M_k (3x3,
 * column-major, applied to a normal). */
struct Folded {
  float f[kMaxSlots][16];
  float r[kMaxSlots][9];
  Compact::Box out;  // the output box the folds were made for
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
u32 skinCorner(const Folded& folded, const s16* pos, const s8* nrm,
               const u8* joints, const u8* weights, s16* outPos, s8* outNrm);

/** A skinned part's bind data at one LOD level (the arrays of SkelBindLod). */
struct SkelSkinPart {
  const s16* pos;         // x, y, z, 0 per corner: the bind index in the model box
  const s8* nrm;          // x, y, z, 0 per corner, x 127
  const u8* joints;       // four palette slots a corner
  const u8* weights;      // their u8 weights, four a corner
  const u32* skinSource;  // the first identical corner (<= this one)
  u32 count;              // corners = the stream's vertices
};

/** Returned by skinIntoStream when the stream cannot take the part (not
 * allocated, another vertex count or layout, or another box than the folds'). */
constexpr u32 kSkinRefused = 0xFFFFFFFFu;

/**
 * Skins a part into `out` - a stream packed from the same part (packArrays,
 * the layout with a position first and a normal, the box the folds were made
 * for). Writes positions and normals per corner and finishes the stream (its
 * boxes and content version). Returns the corners with a position clamped, or
 * kSkinRefused.
 */
u32 skinIntoStream(const Folded& folded, const SkelSkinPart& part,
                   StaPipCompactStream& out);

}  // namespace SkelSkin
}  // namespace Tyra
