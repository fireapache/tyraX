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

#include "../../../renderer/3d/mesh/dynamic/blend_space_weights.hpp"
#include "../../../renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

// Matrices are plain 64-byte-aligned float arrays, not M4x4: the parse is
// host-compilable (tools/verify-tskl-compact.cpp), and the VU0 path reads them
// as quadwords, which the alignment keeps true.
namespace Tyra {

/** One node of the model's hierarchy with its bind-pose local transform. */
struct SkelNode {
  s32 parent = -1;  // -1 = root
  u8 hasMatrix = 0; // 1 = use `matrix` (such nodes are never animated)
  float t[3] = {0.0F, 0.0F, 0.0F};
  float r[4] = {0.0F, 0.0F, 0.0F, 1.0F};  // x, y, z, w quaternion
  float s[3] = {1.0F, 1.0F, 1.0F};
  alignas(64) float matrix[16] = {};  // column-major, valid when hasMatrix
  std::string name;  // glTF node name ("" in files baked before the NAME block)
};

/** One matrix-palette slot: global(node) * ibm skins the verts bound to it. */
struct SkelJoint {
  u32 node = 0;
  alignas(64) float ibm[16] = {};  // inverse bind matrix, column-major
};

/** The `timeArray` of a channel that has one key: it holds that value at every time. */
constexpr u16 kSkelConstantChannel = 0xFFFF;
/** `SkelChannel::step` bits: 1 = STEP interpolation; 2 = the values are floats (6 int16 words a
 * key holding the bits of x, y, z): a translation or scale kept as authored. */
constexpr u8 kSkelStepBit = 1;
constexpr u8 kSkelFloatBit = 2;

/**
 * One keyframe track of a clip (.tskl 4): a node's translation / rotation / scale. Its keys' times
 * are a shared time array of the clip (`ticks`, picked by `timeArray`); its values are int16 in
 * `SkelClip::values` from `valueAt` - 3 a key for a translation (in the clip's tBox) or a scale
 * (in its sBox), 4 for a rotation (the quaternion x 32767). A translation or scale can keep its
 * authored floats instead (kSkelFloatBit), and does by default: then the tBox / sBox are not
 * used. A one that never changes is one key. The sampler is SkelClipCodec
 * (skel_clip_codec.hpp).
 */
struct SkelChannel {
  u32 node = 0;
  u8 path = 0;   // 0 translation, 1 rotation, 2 scale
  u8 step = 0;   // kSkelStepBit: hold the left key, else linear; kSkelFloatBit: exact floats
  u16 timeArray = kSkelConstantChannel;  // into SkelClip::timeArrays; 0xFFFF = one key
  u32 valueAt = 0;  // the first key's first value, in SkelClip::values (a float key: 6 s16
                    // holding the bits of its 3 floats)
};

/**
 * One animation clip in the compact form (.tskl 4). Key times are whole ticks of `tickRate`
 * (key seconds = tick / tickRate); channels whose keys fall on the same ticks share one time
 * array; a channel with one key stores its value once and no times.
 */
struct SkelClip {
  std::string name;
  float duration = 0.0F;  // seconds; 0 = static pose
  float tickRate = 24.0F;  // ticks a second: 24, 25, 30, 48, 50, 60, 120 or 1000
  Compact::Box tBox{};  // the translation values' box (isotropic: one step, an origin an axis)
  Compact::Box sBox{};  // the scale values' box
  std::vector<u16> ticks;       // every time array, one after another
  std::vector<u32> timeArrays;  // (first << 16) | count, into `ticks`
  std::vector<s16> values;      // every channel's keys
  std::vector<SkelChannel> channels;
};

/**
 * One mesh in the compact form (.tskl 3): positions are the model box's grid
 * index less the origin's, normals x 127 (x, y, z, 0), UVs (uv - uvOrigin) x
 * 2^uvFracBits on the mesh's own grid (empty when untextured). Joints and
 * weights are as they always were. Decode with decodeMesh.
 */
struct SkelMesh {
  u32 vertexCount = 0;
  std::vector<s16> pos;    // x, y, z per vertex
  std::vector<s8> nrm;     // x, y, z, 0 per vertex
  std::vector<s16> uv;     // u, v per vertex; empty when untextured
  s8 uvFracBits = 12;      // 4..12
  s32 uvOrigin[2] = {0, 0};
  std::vector<u8> joints;  // vertexCount * 4 palette slots
  std::vector<u8> weights; // vertexCount * 4; skinning divides by sum
};

/** A decimated variant of a part's mesh (same layout, fewer triangles). */
struct SkelLod : SkelMesh {
  // An AUTHORED LOD's own texture (the .tskl "LTEX" trailer), relative to the
  // ELF cwd; "" = draw with the part's texture.
  std::string texturePath;
};

struct SkelBindCache;  // skel_instance.hpp: bind data shared by all instances

/** One draw batch in bind pose (flat triangle list) with palette bindings. */
struct SkelPart : SkelMesh {
  std::string name;
  std::string texturePath;  // relative to the ELF cwd, "" = untextured
  float color[4] = {1.0F, 1.0F, 1.0F, 1.0F};
  std::vector<SkelLod> lods;  // [0] ~50% verts, [1] ~25%; may be empty
};

/** A 2D blend space (docs/blend-spaces.md): clips of this model placed at
 * (direction, speed) points, blended by SkelInstance's blend-space mode. Baked
 * from a .blendspace asset into the .tskl's "BLSP" trailer; each sample's
 * `clip` is an index into SkelModel::clips. */
struct SkelBlendSpace {
  std::string name;        // the name a game asks for (setAnimationMode)
  float stride = 0.0F;     // metres per phase cycle: the engine-played phase
                           // advances by distance (0 = by the clips' time)
  float smoothing = 0.0F;  // s: the input point's time constant (0 = none)
  float lodFade = 0.15F;   // s: the crossfade of a blend-LOD switch
  bool grid = false;       // laid out as a 2D grid ("BLGR"), else the compass
  blendspace::Grid lines;  // the grid's lines, when grid
  std::vector<blendspace::Sample> samples;
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
  std::vector<SkelBlendSpace> blendSpaces;  // optional "BLSP" trailer
  // The one isotropic box every mesh of the model is quantized in (.tskl 3; a
  // v1-2 file is boxed at load).
  Compact::Box box{};
  // Pose AABB, union over every clip (sampled by the baker; files written
  // before 2026-07 carry the clip-0 t=0 box). Used for box collision and,
  // padded, for whole-instance frustum culling.
  float min[3] = {0.0F, 0.0F, 0.0F};
  float max[3] = {0.0F, 0.0F, 0.0F};
  // The skinning loop's repacked bind data (every part, every LOD level),
  // built by the first SkelInstance and shared by the rest - it is identical
  // for all of them, and per instance it cost ~57 bytes per vertex per level.
  mutable std::shared_ptr<const SkelBindCache> bindCache;
  // A fingerprint of everything in the model but its look (the parts' texture paths and
  // colours): skeleton, palette, clips, blend spaces and every mesh. Set by the parse. Two
  // files with the same key differ only in how they are painted (a material override bakes its
  // own .tskl), so one SkelModel serves both - see sharesGeometryWith.
  u64 geometryKey = 0;

  /** Whether `other` is the same model painted differently: the same geometryKey and the same
   * shape (parts, levels, palette, clips). One SkelModel and its bind data then serve both, each
   * keeping its own SkelLook. */
  bool sharesGeometryWith(const SkelModel& other) const {
    return geometryKey != 0 && geometryKey == other.geometryKey && parts.size() == other.parts.size() &&
           palette.size() == other.palette.size() && clips.size() == other.clips.size() &&
           nodes.size() == other.nodes.size();
  }

  /** The most levels of detail any part has: 1 + the longest chain of decimated or authored
   * levels (4 when a part carries a level 3, the far tier). */
  u8 levelCount() const {
    size_t most = 1;
    for (const SkelPart& part : parts)
      if (part.lods.size() + 1 > most) most = part.lods.size() + 1;
    return static_cast<u8>(most);
  }
};

/**
 * What painting a model differently changes (a material override bakes its own .tskl): each
 * part's texture and colour, and its authored LODs' textures. A game that runs two looks of one
 * model keeps one SkelModel for both and a SkelLook for each.
 */
struct SkelLook {
  struct Part {
    std::string texturePath;
    float color[4] = {1.0F, 1.0F, 1.0F, 1.0F};
    std::vector<std::string> lodTexturePaths;  // per decimated or authored level (level - 1)
  };
  std::vector<Part> parts;

  /** The look a parsed model has. */
  static SkelLook of(const SkelModel& model);
};

/**
 * Decodes one compact mesh to floats in the model's box: positions (3 a
 * vertex), normals (3, renormalised after the int8 grid) and, when `uv` is
 * non-null and the mesh is textured, UVs (2). Any output may be null.
 */
void decodeMesh(const Compact::Box& box, const SkelMesh& mesh, float* pos,
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
   * Parses a whole .tskl image (versions 1-4) in memory, trailers included. A
   * version 1-2 image is converted to the compact form here, boxed by the
   * editor's rule, and the clips of a version 1-3 image are converted to the
   * compact clips of version 4 by the editor's own rule (SkelClipCodec::pack).
   * Texture names are as the file has them (load adds the directory).
   * @return false on a bad, unknown or truncated image.
   */
  static bool parse(const u8* data, size_t size, SkelModel& out);
};

}  // namespace Tyra
