/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the unique-vertex skin.
*/

#pragma once

// The unique-vertex skin (SkelSkin::Unique): a part's blend, box and quantize run on its
// UNIQUE corners only - the corners whose bind data is its own (skinSource[v] == v) - and
// the scatter (SkinScatter, buildScatter) writes each unique result to every destination
// corner. A fighter's corners are only about a fifth unique (81 of 417 in a LOD2 part), so
// the float work and the quantize are paid once per unique vertex, not once per corner.
//
// The arithmetic is the VU0 macro sequence in float, the palette being the pose's float
// joint matrices (column-major 4x4, global x inverse bind, as SkelJoint evaluates them):
//   position: sum_k w_k (M_k x (p, 1)),  normal: sum_k w_k (R_k x n), renormalised.
// The bind data is decoded once (UniqueBind) into 16-byte rows: the position in model
// space (bind step x (q + bind origin index), exactly as Fixed::fold decodes it), the
// normal / 127, the joints and the normalised weights. The quantize rounds half away
// from zero to output steps and clamps to +-32767 (positions) / +-127 (normals), and
// counts a clamped unique once per destination (as the per-corner loop counted).
//
// Plain C++ only: the EE kernel (VU0 macro mode) is the same sequence on the same arrays.

#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

namespace Tyra {
namespace SkelSkin {
namespace Unique {

/** A part's unique corners, shared by every body of the model (built once per part level). */
struct UniqueBind {
  u32 count = 0;             // unique corners
  std::vector<u32> corner;   // [count] the corner index of each unique (skinSource[v] == v)
  std::vector<float> pos;    // [count][4] model-space position (w = 1)
  std::vector<float> nrm;    // [count][4] bind normal / 127 (w = 0)
  std::vector<float> wgt;    // [count][4] the four weights, normalised to sum 1 (0 when all 0)
  std::vector<u8> joints;    // [count][4] palette slots
};

/** Builds the unique arrays of a part from its bind data (the corners' arrays of a SkelMesh
 * or SkelBindLod level) and its skinSource. `bind` is the model's bind box. */
void buildUnique(const s16* q, const s8* nrm, const u8* joints, const u8* weights,
                 const u32* skinSource, u32 corners, const Compact::Box& bind, UniqueBind& out);

/** The pose's blend of every unique corner: outPos [count][4] (model space, w = 1) and
 * outNrm [count][4] (unit length, w = 0). `palette` is slots x 16 floats. */
void blend(const UniqueBind& ub, const float* palette, float* outPos, float* outNrm);

/** The pose's box origin in output steps: the grid point at the centre of the blended
 * unique positions' extent (the box posePlace would choose for every corner). An empty
 * part gives 0. */
void poseOrigin(const float* pos, u32 count, float step, s32 oi[3]);

/** The pose origin from the blended positions' extent (min and max per axis): poseOrigin's own
 * arithmetic on the extent alone, for a kernel that folds the extent while it blends and keeps no
 * position array. */
void originFromExtent(const float mn[3], const float mx[3], float step, s32 oi[3]);

/** Part bounds as packed halfword lanes (the EE's PMINH / PMAXH): `lo` and `hi` are [ranges][4]
 * s16 (x, y, z and a zero pad lane). boundsReset sets lo to 32767 and hi to -32768 on x, y, z;
 * boundsFold folds one unique's quantized position (four lanes, the fourth 0) into one range. */
void boundsReset(u32 ranges, std::vector<s16>& lo, std::vector<s16>& hi);
void boundsFold(s16* lo, s16* hi, const s16* q);

/** Quantizes one blended unique: position (model space, w = 1) and unit normal (w = 0) into
 * the output box (invStep = 1 / step, origin index oi). Returns 1 when a position axis clamped.
 * qPos and qNrm are four lanes each (the fourth 0). */
u32 quantizeOne(const float* pos, const float* nrm, float invStep, const s32 oi[3], s16* qPos,
                s8* qNrm);

/** Quantizes the blended uniques into the output box (step, origin index oi): positions
 * round to the nearest step and clamp to +-32767, normals (unit length from blend) scale to
 * +-127. qPos [count][4] (fourth lane 0), qNrm [count][4] (fourth lane 0). `clamped[u]` is
 * set to 1 for a unique with a position axis out of the box. Returns the number of uniques
 * clamped. */
u32 quantize(const float* pos, const float* nrm, u32 count, float step, const s32 oi[3],
             s16* qPos, s8* qNrm, u8* clamped);

/** The per-corner view of the uniques: scatters each unique's quantized position and normal
 * to its destinations (map from buildScatter, package size `size`) into the flat corner
 * arrays outPos [corners][4] and outNrm [corners][4]. A repeat corner copies its source's
 * bytes by construction. Returns the number of clamped destinations (clamped uniques counted
 * once per destination). `corners` bounds the flat index (package x size + element). */
u32 scatterFlat(const SkinScatter& map, u32 size, const UniqueBind& ub, const s16* qPos,
                const s8* qNrm, const u8* clamped, u32 corners, s16* outPos, s8* outNrm);

}  // namespace Unique
}  // namespace SkelSkin
}  // namespace Tyra
