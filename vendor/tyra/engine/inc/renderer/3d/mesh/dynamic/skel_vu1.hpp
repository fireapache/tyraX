#pragma once

// Skinning on VU1, EE side: the shared bind record and the per-pose palette that a
// VU1 blend pass reads (docs/animated-models.md, "Skinning on VU1"). Pure functions with
// no allocation, so the host verifier (tools/verify-skel-vu1.cpp) links them unchanged.
//
// The bind record is shared by every fighter of a model part level: it holds the bind
// index, the bind normal, and the joints and weights as the blend reads them. The
// joints are stored PRE-SCALED to the palette's qword address (kPaletteBaseQword +
// kQwordsPerJoint x slot), so the VU1 loads them straight into an index register with
// ILW. The weights stay raw u8: they sum to 255 and the palette carries the 1/255.
//
// The palette is the existing fold (SkelSkin::foldPalette: index units of the output box)
// divided by 255. Its 3x3 is also the normal's rotation: the box scale cancels in the
// renormalise, so one palette serves positions and normals.

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"

namespace Tyra {
namespace SkelVu1 {

/** The VU1 skin path (docs/animated-models.md, "Skinning on VU1"): a skinned body is
 * blended on the VU1 from the shared bind record and its palette instead of writing a
 * per-fighter stream. Default off; nothing reads it until the draw path lands. */
#ifndef TYRA_SKIN_VU1
#define TYRA_SKIN_VU1 0
#endif

/** Qwords 0..3 of VU1 data memory are the MVP (VU1_MVP_MATRIX_ADDR); the palette
 * starts after them. */
constexpr u32 kPaletteBaseQword = 4;
/** A joint's palette is four float columns: c0, c1, c2 and the translation. */
constexpr u32 kQwordsPerJoint = 4;
/** The pre-scaled joint offset is one byte (ILW reads a byte lane): 4 x slot + base
 * <= 255 bounds the palette at this many slots. */
constexpr u32 kMaxSlots = (255 - kPaletteBaseQword) / kQwordsPerJoint + 1;
/** A palette slot is 64 bytes. */
constexpr u32 kSlotBytes = kQwordsPerJoint * 16;

/**
 * One corner of the shared bind record. 20 bytes: the bind index (int16, the model's
 * grid), the bind normal (int8 x127, w = 0), the pre-scaled joint offsets and the raw
 * weights. The texture coordinates and colour the drawn stream also carries are static
 * per part level and stay in the existing packed run (10 bytes), so a corner of the
 * whole bind data is 30 bytes.
 */
struct BindCorner {
  s16 pos[3];
  s16 pad0;
  s8 nrm[4];
  u8 joint[4];   // kPaletteBaseQword + kQwordsPerJoint x slot, per influence
  u8 weight[4];  // raw u8, the four sum to 255
};
static_assert(sizeof(BindCorner) == 20, "the shared bind record is 20 bytes");

/**
 * Builds a part level's bind records from its compact mesh. Returns false (and writes
 * nothing usable) when a joint slot cannot be pre-scaled into a byte, or a corner's weights
 * do not sum to 255 (the palette's 1/255 is then wrong for that corner).
 */
bool buildBind(const SkelMesh& mesh, u32 slots, BindCorner* out);

/**
 * One pose's palette for a model: `slots` folded matrices (SkelSkin::foldPalette against
 * the bind box and the output box), each divided by 255. `palette` receives 16 floats a
 * slot, column-major, at palette[16 x slot]. The caller owns the storage (two sides, see
 * the docs); `slots` must not exceed kMaxSlots.
 */
void buildPalette(const float* pose, u32 slots, const Compact::Box& bind,
                  const Compact::Box& out, float* palette);

}  // namespace SkelVu1
}  // namespace Tyra
