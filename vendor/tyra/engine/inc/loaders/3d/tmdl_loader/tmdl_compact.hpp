/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the compact model rules shared by the .tmdl writer (the
# editor's src/tmdl.cpp), the .tmdl parse (tmdl_parse.cpp) and the ASCII path.
# Host-compilable: it needs no EE header, so the editor and the host verifier
# link it as it is.
*/

#pragma once

#include <vector>

#include "loaders/3d/obj_loader/lean_obj_loader.hpp"

namespace Tyra {
namespace TmdlCompact {

/**
 * The extent of a set of points (xyz, `stride` floats apart), accumulated over
 * every mesh of one model. The model's box is taken from its bounds.
 */
struct Bounds {
  float mn[3] = {0.0f, 0.0f, 0.0f};
  float mx[3] = {0.0f, 0.0f, 0.0f};
  bool any = false;

  void add(const float* xyz, u32 count, u32 stride);
  /** Compact::modelBox over the bounds; an empty set boxes at the origin. */
  Compact::Box box() const;
};

/**
 * One mesh quantized in `box`: `count` vertices of 8 floats (pos, normal, uv).
 * Its UV grid is chosen here (uvGrid), so the result is a function of the
 * vertices and the box alone.
 */
CompactMesh fromFloats(const float* verts, u32 count, const Compact::Box& box);

/**
 * The UV grid of `count` vertices: the origin is the floor of the least u and
 * the least v; the fraction is the largest from 12 down to 4 that keeps the
 * largest (uv - origin) x 2^fraction inside int16. A cube's 0..1 UVs get 12, a
 * plane tiled 40 x 40 gets 9.
 */
void uvGrid(const float* verts, u32 count, s8* fracBits, s32 origin[2]);

/** A compact mesh back to 8 floats a vertex (pos, normal, uv), into `out`. */
void toFloats(const CompactMesh& m, const Compact::Box& box,
              std::vector<float>& out);

/**
 * A whole model that carries only float data (an ASCII .obj, a .tmdl 1-4):
 * its box from its bounds, every compact copy and the compact shadow proxy.
 */
void compactModel(LeanObjMesh& m);

}  // namespace TmdlCompact
}  // namespace Tyra
