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

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

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

}  // namespace SkelSkin
}  // namespace Tyra
