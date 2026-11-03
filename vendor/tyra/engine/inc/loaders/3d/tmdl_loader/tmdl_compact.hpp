/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the compact model rules shared by the .tmdl writer (the
# editor's src/tmdl.cpp), the .tmdl parse (tmdl_parse.cpp) and the ASCII path.
# Host-compilable and self-contained: the editor includes this header by its
# path, with the engine's include tree private to its compact library, so the
# includes below are relative (vugen.cpp does the same for stapip_compact.hpp).
*/

#pragma once

#include <vector>

#include "../../../renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

namespace Tyra {

struct LeanObjMesh;

/**
 * One mesh in the compact form (.tmdl version 5, plan 52): the same surface as
 * the float vertices beside it, 8 floats a vertex, quantized in the owning
 * LeanObjMesh::box. Positions are the box's grid index less the origin's
 * (Compact::quantize), normals x 127, UVs (uv - uvOrigin) x 2^uvFracBits. The
 * UV grid is per mesh: the largest fraction that keeps its range in int16.
 */
struct CompactMesh {
  u32 vertexCount = 0;
  std::vector<s16> pos;  // x, y, z per vertex
  std::vector<s8> nrm;   // x, y, z, 0 per vertex
  std::vector<s16> uv;   // u, v per vertex
  s8 uvFracBits = 12;    // 4..12
  s32 uvOrigin[2] = {0, 0};
};

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
