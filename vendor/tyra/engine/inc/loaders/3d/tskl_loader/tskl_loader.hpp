/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: loader for .tskl skeletal-animation models.
*/

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <tamtypes.h>

#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

// Matrices are plain 64-byte-aligned float arrays, not M4x4: the parse is
// host-compilable (tools/verify-tskl-compact.cpp), and the VU0 path reads
// them as 16-byte quadwords, which the alignment keeps true.
namespace Tyra {

/** One node of the model's hierarchy with its bind-pose local transform. */
struct SkelNode {
  s32 parent = -1;  // -1 = root
  u8 hasMatrix = 0; // 1 = use `matrix` (such nodes are never animated)
  float t[3] = {0.0F, 0.0F, 0.0F};
  float r[4] = {0.0F, 0.0F, 0.0F, 1.0F};  // x, y, z, w quaternion
  float s[3] = {1.0F, 1.0F, 1.0F};
  alignas(64) float matrix[16] = {};  // column-major, valid when hasMatrix
};

/** One matrix-palette slot: global(node) * ibm skins the verts bound to it. */
struct SkelJoint {
  u32 node = 0;
  alignas(64) float ibm[16] = {};  // inverse bind matrix, column-major
};

/** One keyframe track: a node's translation / rotation / scale over time. */
struct SkelChannel {
  u32 node = 0;
  u8 path = 0;  // 0 translation, 1 rotation, 2 scale
  u8 step = 0;  // 1 = STEP interpolation (hold left key), 0 = linear
  std::vector<float> times;  // seconds from clip start, ascending
  std::vector<float> vec;    // path 0/2: keyCount * 3
  std::vector<s16> quat;     // path 1: keyCount * 4, component * 32767
};

struct SkelClip {
  std::string name;
  float duration = 0.0F;  // seconds; 0 = static pose
  std::vector<SkelChannel> channels;
};

/**
 * One mesh in the compact form (.tskl 3): positions are the model
 * box's grid index less the origin's, normals x 127 (x, y, z, 0), UVs
 * (uv - uvOrigin) x 2^uvFracBits on the mesh's own grid. Joints and weights
 * are as they always were. A decimated variant (LOD) of a part is one of these.
 */
struct SkelLod {
  u32 vertexCount = 0;
  std::vector<s16> pos;    // x, y, z per vertex
  std::vector<s8> nrm;     // x, y, z, 0 per vertex
  std::vector<s16> uv;     // u, v per vertex; empty when untextured
  s8 uvFracBits = 12;      // 4..12
  s32 uvOrigin[2] = {0, 0};
  std::vector<u8> joints;  // vertexCount * 4 palette slots
  std::vector<u8> weights; // vertexCount * 4; skinning divides by sum
};

/**
 * One draw batch in bind pose (flat triangle list) with palette bindings. The
 * base mesh is the SkelLod it inherits; `lods` are its distance variants.
 */
struct SkelPart : SkelLod {
  std::string name;
  std::string texturePath;  // relative to the ELF cwd, "" = untextured
  float color[4] = {1.0F, 1.0F, 1.0F, 1.0F};
  std::vector<SkelLod> lods;  // [0] ~50% verts, [1] ~25%; may be empty
};

/**
 * A skeletal model: shared, immutable data every SkelInstance samples from.
 * Written by the TyraX build (src/glbparser.cpp writeTskl) - keep the
 * binary layout in sync with it.
 */
struct SkelModel {
  std::vector<SkelNode> nodes;
  std::vector<u32> order;  // parents-first traversal (computed at load)
  std::vector<SkelJoint> palette;
  std::vector<SkelClip> clips;  // >= 1
  std::vector<SkelPart> parts;
  // The one isotropic box every part and LOD tier is quantized in (.tskl 3;
  // a v1-2 file is boxed at load).
  Compact::Box box{};
  // Pose AABB, union over every clip (sampled by the baker; files written
  // before 2026-07 carry the clip-0 t=0 box). Used for box collision and,
  // padded, for whole-instance frustum culling.
  float min[3] = {0.0F, 0.0F, 0.0F};
  float max[3] = {0.0F, 0.0F, 0.0F};
};

/**
 * Decodes one compact mesh to floats in the model's box: positions (3 a
 * vertex), normals (3, renormalised after the int8 grid) and, when `uv` is
 * non-null and the mesh is textured, UVs (2). Any output may be null.
 */
void decodeMesh(const Compact::Box& box, const SkelLod& mesh, float* pos,
                float* nrm, float* uv);

/**
 * Loader for the TyraX .tskl format: node hierarchy, skin palette,
 * bind-pose mesh and keyframe tracks of a .glb model. Poses are evaluated
 * and skinned on the EE at runtime by SkelInstance (renderer/3d/mesh).
 *
 * Reads the whole file into memory first (fseek is unreliable over the PS2
 * host filesystem) and works on both host: and cdrom0: boot paths.
 */
class TsklLoader {
 public:
  /**
   * @param relativePath path relative to the ELF cwd, e.g. "models/hero.tskl"
   * @return parsed model, or nullptr when the file is missing/malformed
   */
  static std::unique_ptr<SkelModel> load(const std::string& relativePath);

  /**
   * Parses a whole .tskl image (versions 1-3) in memory. A version 1-2 image
   * is converted to the compact form here, boxed by the editor's rule. Same
   * merging of equal materials as load. @return false on a bad, unknown or
   * truncated image, with `out` left empty.
   */
  static bool parse(const u8* data, size_t size, SkelModel& out);
};

}  // namespace Tyra
