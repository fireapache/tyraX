/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: per-object skeletal playback; pose evaluation on
# the EE, vertex skinning on VU0 in macro mode.
*/

#pragma once

// Optional per-instance pose/skin timings; zero instructions in normal builds.
#ifndef TYRA_SKEL_PROFILE
#define TYRA_SKEL_PROFILE 0
#endif

#include <memory>
#include <new>
#include <vector>

#include "loaders/3d/tskl_loader/tskl_loader.hpp"
#include "renderer/3d/mesh/dynamic/skel_batch.hpp"
#include "renderer/3d/mesh/dynamic/skel_indexed_config.hpp"
#include "renderer/3d/mesh/dynamic/skel_indexed_shared.hpp"
#include "renderer/3d/mesh/dynamic/skel_render_slots.hpp"
#include "renderer/3d/mesh/dynamic/skel_skin.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"
#include "./dynamic_mesh.hpp"

namespace Tyra {

/** One part at one LOD level, bind-pose data repacked into 16-byte-aligned
 * qwords for the VU0 skinning loop (positions w = 1, normals w = 0 so the
 * translation column drops out; weights pre-normalized to sum 1; joints
 * re-sorted per vertex by descending weight with the nonzero count in
 * `influences`, so the loop dispatches to a blend of exactly 0, 1, 2 or 4
 * matrices). Immutable once built, so every instance of the model reads the
 * same copy (SkelModel::bindCache).
 *
 * The model's own levels carry the skin source and the unique level (`uniq`) only: the compact
 * mesh (SkelMesh) is the bind pose. The float rows below are filled for the unique level and
 * for the full level a body that skins float arrays asks for (SkelInstance::ensureFloatLevel,
 * `floatLevel`), not for every corner of every level at load. */
struct SkelBindLod {
  std::vector<Vec4> bindPositions, bindNormals, skinWeights;
  std::vector<u8> sortedJoints, influences;
  std::vector<u32> skinSource;  // first identical corner, always <= this corner
  std::vector<Vec4> uvs;        // packed texture coords, levels > 0 (a float level only)
  SkelSkin::SkelSkinExtent extent;  // the index box and used slots (bindExtent)
  // The full level's float rows (every corner), made when a body first skins float arrays and
  // alive while one holds it. Shared by the bodies of the model; empty otherwise.
  mutable std::weak_ptr<const SkelBindLod> floatLevel;
  // The stream scatter map (built on first use for a stream package size; shared by the
  // instances of the model).
  mutable SkelSkin::SkinScatter scatter;
  // The batch kernel's influence entries and CSR (built on first use; gUseBatch only).
  mutable SkelSkin::Batch::Part batch;
  // The unique corners only (the corners whose skinSource is their own), as a level of their
  // own: positions, normals, weights and joints in the same layout. Its skinSource is empty (the
  // identity). skinIntoUniqueVu0 blends it with the VU0 corner loop. Built with the cache.
  std::shared_ptr<SkelBindLod> uniq;
  // The corner index of each unique (uniq's level only): made on first use by the plain-stream
  // unique pass, which scatters the uniques to their corners; empty for indexed streams.
  std::vector<u32> uniqCorner;
  // The indexed stream's per-level half (TYRA_SKIN_INDEXED): the packaging, the static blocks and
  // the pose-buffer pool, built the first time a body of the model draws this level. Shared by
  // every instance; `idxFailed` says the build was tried and refused (no retry, the level
  // keeps its plain streams).
  mutable std::shared_ptr<SkelSkin::Indexed::Shared> idx;
  mutable bool idxFailed = false;
  u32 count = 0;
};

struct SkelBindCache {
  std::vector<std::vector<SkelBindLod>> parts;  // [part][lod]
};

/**
 * One scene object's view of a SkelModel: playback state (with crossfade),
 * per-frame pose evaluation (EE) and matrix-palette skinning (VU0 macro
 * mode - runs on COP2 next to the EE, no microprogram involved).
 *
 * The skinned result lives in an owned single-frame DynamicMesh whose
 * vertex/normal arrays are overwritten in place every update. Render the
 * arrays (mesh->materials[i]->frames[0]) through StaPip like any static
 * geometry: one vertex upload, no VU1 program swap, EE clipping - and use
 * advance()/ensurePose() to skip the pose+skin work for culled instances
 * (~0.9 ms per 1092-vert instance on real hardware; PCSX2's fast EE hides
 * it). DynamicPipeline rendering still works (one frame = verticesFrom ==
 * verticesTo, interpolation 0) but submits every vertex twice.
 */
class SkelInstance {
 public:
  /** `floatMesh` false: a body that only ever draws render slots (docs/render-slots.md). It
   * makes none of the per-body float mesh arrays (48 bytes a corner) - the mesh keeps its
   * bounding boxes and two-vertex stand-ins for the arrays - and ensurePose / lodArrays have
   * nothing to skin or return. */
  explicit SkelInstance(const SkelModel* t_model, bool floatMesh = true);
  ~SkelInstance();

  /** Added by TyraX: a new instance of `model`, or null - with nothing made -
   * when the heap has no room for one. It first asks, in one block from the
   * nothrow operator new[], for what the constructor will take (bytesFor, plus
   * a margin) and gives it straight back, so the constructor's own allocations
   * fit where that block did. A caller that gets null has no instance yet:
   * it plays on without one and asks again later. */
  static std::unique_ptr<SkelInstance> create(const SkelModel* model, bool floatMesh = true);

  /** Added by TyraX: what the constructor allocates for `model`, in bytes -
   * an estimate that errs high: every part's full-mesh arrays (positions,
   * normals, UVs), the per-node pose scratch, the palette, the clip cursors,
   * the mesh's own objects and, when the model has none yet, the bind data
   * its first instance builds with the temporary table that builds it (the
   * largest level's vertices x ~132 bytes on the EE). LOD levels' skin
   * outputs are made later, on first use, and are not in it. */
  static size_t bytesFor(const SkelModel& model, bool floatMesh = true);

  const SkelModel* model;

  /** Single-frame output mesh (owned). Materials carry the part colors in
   * `ambient` and fresh ids - (re)link textures against these ids. */
  std::unique_ptr<DynamicMesh> mesh;

  /**
   * (Re)starts a clip from its first frame.
   * @param fadeSeconds > 0 crossfades from whatever pose is currently
   * showing (linear blend of local transforms, nlerp on rotations).
   */
  void play(u32 clip, bool loop, float fadeSeconds = 0.0F);

  /** Live loop-flag change without restarting (a finished one-shot resumes
   * wrapping on the next update, like DynamicMeshAnimation did). */
  void setLoop(bool loop) { cur.loop = loop; }

  /** Jumps the playing clip to `seconds` (clamped to the clip) without
   * restarting it; the pose is re-evaluated on the next ensurePose(). This
   * is how a game DRIVES a clip by one of its own values instead of playing
   * it: a two-key clip (frame 0 = standing, last frame = crouched) sought to
   * the crouch depth every frame is a linear interpolation between the two
   * poses, with the game holding the parameter. */
  void seek(float seconds);

  /** A HELD blend layer: from now on the pose is
   * lerp(playing pose, `clip` at `seconds`, weight) - rotations nlerp like
   * the crossfade - until it is cleared (weight <= 0 or a clip out of range).
   * Unlike play()'s crossfade it never advances on its own: the caller owns
   * both the time and the weight, so an offset (a lean, an aim) rides on top
   * of whatever is playing and survives a clip switch underneath it. */
  void setBlend(s32 clip, float seconds, float weight);

  /** 2D blend spaces (docs/blend-spaces.md): the model's blend space by
   * name (SkelModel::blendSpaces, the .tskl "BLSP" trailer), or -1. */
  s32 findBlendSpace(const char* name) const;

  /** Puts the instance in BLEND-SPACE mode on `space` (or back to its clip
   * with -1): instead of one playing clip it shows up to 3 of the space's
   * samples, weighted by the input below. A switch either way crossfades
   * from the pose showing over `fadeSeconds`. play() leaves the mode too
   * (with its own fade). The held layer (setBlend) rides on top in both. */
  void setBlendSpace(s32 space, float fadeSeconds = 0.0F);
  s32 blendSpace() const { return bsSpace; }

  /** The blend space's input: degrees from the facing (+ = right), m/s,
   * and the sync samples' phase 0..1 - or < 0 to let advanceBlend() play
   * it. Smoothed by the space's `smoothing`; read by advanceBlend(). */
  void setBlendInput(float dirDeg, float speed, float phase);

  /** The blend LOD: true shows only the best-fitting sample (the heaviest,
   * with hysteresis), crossfading over the space's `lodFade` on a change. */
  void setBlendLod(bool singleSample) { bsSingle = singleSample; }
  /** What the last setBlendLod asked - the state blendspace::lodSingle's
   * distance band needs. */
  bool blendLodSingle() const { return bsSingle; }

  /** Blend-space bookkeeping for a frame of `dt` seconds (0 = paused: the
   * smoothing, the clocks and the phase hold): smooths the input, picks the
   * samples and their weights, advances the phase and the own-time clocks
   * and any mode/LOD crossfade. The pose itself waits for ensurePose(). */
  void advanceBlend(float dt);

  /** The samples the pose is built from (blend LOD included): up to 3 clip
   * indices and weights, heaviest first. 0 outside blend-space mode. While a
   * switch crossfades (into the mode, a blend-LOD change) the weights sum to
   * less than 1: the rest of the pose is the one it fades from. */
  int shownSamples(s32 clips[3], float weights[3]) const;

  /**
   * Advances playback by dt seconds (scale dt for playback speed), then
   * evaluates the pose and skins into the mesh when anything changed.
   * dt = 0 holds the pose (still skins once after construction/play()).
   *
   * @returns true the frame the clip reaches its last frame: once for
   * one-shot clips, on every wrap for looping ones (the animFinished
   * contract of TyraX games).
   */
  bool update(float dt);

  /** Playback bookkeeping only - the expensive pose evaluation and skinning
   * are deferred until ensurePose(). For instances culled away from the
   * camera: time (and the animFinished contract) keeps running, the mesh
   * keeps its last skinned pose. Same return contract as update(). */
  bool advance(float dt);

  /** Runs only the crossfade by dt seconds. A clip that is HELD (seek(),
   * advanced by 0) still has to fade IN on wall-clock time, or the pose it
   * was switched to never shows - advance(0) leaves the fade where it is. */
  void advanceFade(float dt);

  /** Evaluates the pose and skins into the mesh if playback moved since the
   * last skin (or the requested LOD differs from the last one skinned).
   * Call before reading the arrays of a visible instance.
   * @param lod 0 = full mesh; higher levels use the decimated variants
   * baked into the .tskl, clamped per part to what the file carries.
   * @returns true when the arrays were rewritten (bbox caches keyed on
   * the buffers should be invalidated), false when everything was current. */
  bool ensurePose(u8 lod = 0);

  /** LOD levels available (including the full mesh: always >= 1). Parts can
   * carry different chain lengths; this is the longest one - lodArrays()
   * clamps per part. */
  u8 lodCount() const { return maxLodLevels; }

  /** Playback state for telemetry: the playing clip, its time, and the
   * crossfade's progress (1 = none in flight). */
  s32 currentClip() const { return cur.clip; }
  float currentTime() const { return cur.time; }
  float fadeProgress() const { return fadeT; }

  /** The level the out arrays currently hold (last ensurePose target). A
   * renderer throttling pose refreshes must still call ensurePose when the
   * wanted level differs - other levels' buffers hold older skins (or the
   * bind pose before their first ever skin). */
  u8 currentLod() const { return lastSkinnedLod; }

  /** A node (bone) by its glTF name, or -1: unknown, or a .tskl baked before
   * node names were stored. A linear search - resolve once, keep the index. */
  s32 findNode(const char* name) const;

  /** The node's MODEL-space matrix (column-major, the instance's local
   * frame) as of the last pose evaluated by ensurePose() - so it matches the
   * mesh the instance last drew, animation LOD included. Multiply by the
   * object's own transform for world space. False before the first pose or
   * for an index out of range. */
  bool nodeGlobal(s32 node, M4x4* out) const;

  /** nodeGlobal for a caller that needs THIS instance's matrices even when
   * the renderer did not evaluate them: a body that FOLLOWED another
   * instance's skin this frame (pose sharing - two bodies on the same clip
   * at the same time skin once), or one never drawn at all. Evaluates the
   * current pose first in those two cases, once per pose change; an instance
   * that skinned itself answers exactly as nodeGlobal (animation LOD
   * included). The skin is untouched - ensurePose keeps its own schedule. */
  bool currentNodeGlobal(s32 node, M4x4* out);

  /** The CLIPS' pose of a node (MODEL space), whatever is drawn: with no
   * override it is currentNodeGlobal; under a pose override (a ragdoll) it
   * is what the playing clip - crossfade and held layer included - would
   * show now, evaluated on demand once per clip change into its own buffer
   * (the drawn globals and the skin are untouched). How a game drives a
   * ragdoll TOWARD an animation (a motorized ragdoll). */
  bool clipNodeGlobal(s32 node, M4x4* out);

  /** Bring the node matrices up to the instance's CURRENT clip state, for a
   * simulation that must not depend on how often the renderer poses this body:
   * the far tier poses a body every 8th frame and the animation LOD every 2nd
   * or 4th, so the drawn matrices (nodeGlobal, currentNodeGlobal) can be up to
   * 7 frames behind the clips. Evaluates the skeleton only (no palette, no
   * skin) and only when the matrices are behind - once per clip change - and
   * returns whether it did. The drawn skin and its schedule are untouched:
   * ensurePose / skinInto still re-evaluate and skin on their own turn. */
  bool refreshBones();

  /** The renderer says, per drawn frame, whether this instance followed
   * another's skin (pose sharing) instead of skinning itself. */
  void setFollowing(bool on) { following = on; }

  /** Modified by TyraX: a FOLLOWER takes the node matrices of the instance
   * whose skin it is drawn with (pose sharing: the same model, the same pose)
   * - so a bone a game reads (currentNodeGlobal: a gun on a hand) sits on the
   * body as DRAWN, animation LOD included, and costs a copy instead of the
   * whole pose evaluation a stale follower used to run on every read (~0.13
   * ms a follower a frame for Gunner's crowd - docs/animated-models.md,
   * "What a crowd costs"). A no-op unless `owner` has a posed skeleton of
   * the same model. */
  void adoptPose(const SkelInstance& owner);

  /** How many nodes the model has - the length of a pose override. */
  size_t nodeCount() const;

  /** Drive the pose from the game (a ragdoll): `nodeGlobals` are MODEL-space
   * matrices (the frame nodeGlobal() reports), one per node in the model's
   * node order - `count` must be nodeCount(). While set, the clips are
   * ignored: play/seek/advance keep their bookkeeping, but evalPose builds
   * the palette from these. The instance shares its pose with nobody
   * (poseEquals is false), and it is re-skinned only when the matrices
   * CHANGE - a body at rest costs no skin. False (nothing changed) for a
   * count mismatch. */
  bool setPoseOverride(const M4x4* nodeGlobals, size_t count);
  /** Back to the clips: the next ensurePose() re-evaluates them. */
  void clearPoseOverride();
  bool poseOverridden() const { return overridden; }

  /** A mesh vertex where the instance's CURRENT pose puts it, in model
   * space: the bind-pose vertex nearest `bindPos` (model space, the .glb's
   * own coordinates; within `maxDist`, else false), skinned on the EE with
   * the current palette. Evaluates the pose if playback moved since the last
   * evaluation - independent of whether, or at which LOD, the renderer last
   * skinned this instance (pose sharing, animation LOD) - so it costs a pose
   * plus a scan of the bind vertices: a test/tool call, not per-frame game
   * logic. */
  bool vertexModel(const float bindPos[3], float out[3], float maxDist = 0.05F);

  /** The renderable arrays of one part at one LOD level (clamped to the
   * part's chain). Valid after the matching ensurePose(lod). Never allocates:
   * a level whose skin output the heap could not supply when ensurePose
   * skinned it answers the cheapest level the part holds - the one that was
   * skinned in its place (`level` says which). */
  struct LodArrays {
    Vec4* vertices;
    Vec4* normals;
    Vec4* textureCoords;  // nullptr on untextured parts
    u32 count;
    u8 level;  // the level actually returned (the request clamped to the
               // part's chain, or the cheapest held one when the heap had no
               // room for it) - which SkelLod::texturePath applies
  };
  LodArrays lodArrays(size_t part, u8 lod);

  /** Added by TyraX: the stream a part at a LOD level is drawn from - the drawn
   * side of its pair, else the cheapest level that has one (`level` says which).
   * A null stream: nothing written yet. */
  struct LodStream {
    const StaPipCompactStream* stream;
    u8 level;
  };
  LodStream lodStream(size_t part, u8 lod);
  /** The side of a part's pair a pass may write now: the one no frame reads (its
   * span retired and collected), made on first use under the Skinning budget;
   * null when refused or still read. */
  StaPipCompactStream* spareStream(size_t part, u8 lod);
  /** Skins the instance's pose into the spare sides `partStreams` (one per part,
   * null = skip; from spareStream) with the scalar core, then makes each written
   * side the drawn one and retires the side it replaces. Returns the corners out
   * of their box. Evaluates the pose itself (no float skin). */
  u32 skinInto(u8 lod, StaPipCompactStream* const* partStreams);
  /** Frees the stream pairs of the levels a body has not used for a few passes
   * (the levels other than `lod`'s); a level in use again is re-made. */
  void releaseIdleStreams(u8 lod);
  /** Frees the float bind-pose arrays of the mesh (level 0 vertices, normals and
   * texture coordinates) once every part draws a stream: a streamed body never
   * reads them (its bags point at the stream), so they are dead weight. Does
   * nothing until each part has a drawn stream; the bounding boxes stay. */
  void releaseFloatMesh();
  /** Whether skinInto has a pass to make for `lod`: the pose moved, or the level
   * changed, since the last pass. */
  bool streamPassDue(u8 lod) const {
    return !compactValid || lod != compactLod ||
           (overridden ? overrideDirty : poseDirty);
  }
  /** The level of the last pass, and how many passes this instance has made (a
   * caller compares the count to learn whether a pass wrote). */
  u8 streamLevel() const { return compactLod; }
  u32 streamPasses() const { return compactPasses; }
  /** The parts (one per model part, every level chained under it). */
  size_t partCount() const { return partLods.size(); }

  // --- render slots (docs/render-slots.md) ---------------------------------------------------
  /** Builds the model's bind data when no body has yet (the sizes below read it). */
  static void prepareModel(const SkelModel& model);
  /** The bytes of ONE side of a slot of `tier` for `model`: every part at the level the tier
   * names, as the format the parts will be placed in (the indexed pose buffer when the part
   * is the indexed program's, else the plain stream), boxes included. 0 when the model cannot
   * use slots (more parts than kMaxSlotParts, a part that cannot be placed). `packageSize` is
   * the stream package of the bags (the template's kSkinStreamPackage). This is the
   * SkelRenderSlots::SizeFn of the engine. */
  static u32 slotSideBytes(const SkelModel& model, u8 tier, u32 packageSize);
  /** Places the parts' payload in the slot's two sides (once per model: a slot that is already
   * placed for this model is untouched). False, with the slot unplaced, when a part does not fit
   * or memory for the model's shared data is refused. */
  bool bindSlot(SkelSlot& slot, u32 packageSize);
  /** Whether the slot needs a pass: nothing written for this body yet, or the pose moved. */
  bool slotPassDue(const SkelSlot& slot) const {
    return !slot.written() || (overridden ? overrideDirty : poseDirty);
  }
  /** Skins the pose into the slot's spare side (every part at the level its tier names), then
   * flips it to be the drawn one and stamps `frame`. False - nothing changed, the slot keeps
   * what it drew - when the placement failed, a part was refused, or a frame still reads the
   * spare side. `outside` gets the corners out of their box. */
  bool skinIntoSlot(SkelSlot& slot, u32 frame, u32 packageSize, u32* outside = nullptr);
  /** The level a part draws at a slot tier (the tier clamped to the part's chain). */
  u8 slotLevel(size_t part, u8 tier) const;
  /** Skinned memory this body owns, in bytes: its streams' blocks and boxes, its pose buffers
   * (indexed streams hold theirs in a pool, counted in the model's bytes, not here), the
   * float mesh arrays and the lower-level skin outputs. A body that only draws render slots
   * owns 0. */
  u32 skinnedBytes() const;
  bool streamOnlyBody() const { return streamOnly; }
  /** The layout a part's streams are packed in - the layout of the bag that draws
   * them (StaPipCompactLayout::forBag), so the texture wrap and the lighting flags
   * match. Set before the part's streams are made; ignored once they are. */
  void setStreamLayout(size_t part, const StaPipCompactLayout& layout);

  // Implementation detail: one part at one LOD level of THIS instance - the
  // shared bind data (SkelBindLod) plus the skin output. Level 0 outputs
  // alias the mesh's frame arrays (ownVertices stays empty); deeper levels
  // own theirs, made the first time the level is skinned - when the heap has
  // room for them: until then the part skins and draws the cheapest level it
  // holds (lodArrays answers that level).
  struct PartLod {
    const SkelBindLod* bind = nullptr;
    std::unique_ptr<Vec4[]> ownVertices, ownNormals;  // levels > 0
    Vec4* outV = nullptr;   // skin destination (own or mesh frame)
    Vec4* outN = nullptr;
    Vec4* uvPtr = nullptr;  // nullptr on untextured parts
    u32 count = 0;
    // Added by TyraX: this level as a compact stream (skinInto). Two sides: the
    // one drawn (side) and the one written by the next pass - a side is
    // rewritten only once no frame reads it (spareStream, the lease rule).
    // Made on first need (SkelInstance::ensureStreams / ensureIndexed) and null on a body that
    // draws render slots only, whose streams are the slots': two streams are about 300 bytes a
    // level of every part of every body.
    std::unique_ptr<StaPipCompactStream[]> streams;
    bool anyStream() const { return streams && (streams[0].allocated() || streams[1].allocated()); }
    bool bothStreams() const { return streams && streams[0].allocated() && streams[1].allocated(); }
    /** The pair, made when missing (nothrow, under the caller's budget); false when refused. */
    bool makeStreams() {
      if (!streams) streams.reset(new (std::nothrow) StaPipCompactStream[2]);
      return streams != nullptr;
    }
    // Indexed (TYRA_SKIN_INDEXED): streams[0] is the level's one stream - boxes only, its block
    // the pose buffer `pose` - and streams[1] is unused. The pose buffer comes from the level's
    // pool (SkelBindLod::idx) and is swapped on every pass.
    SkelSkin::Indexed::PoseBuf* pose = nullptr;
    bool indexed = false;
    u8 side = 0;
    bool streamed = false;  // a pass has written the drawn side
    u8 idlePasses = 0;      // passes since a pass last used this level
  };

  /** True when both instances strike the identical pose this frame (same
   * model, same clip at the same time, no crossfade in flight) - their
   * skinned meshes are interchangeable, so a renderer can skin one and draw
   * it for every instance in the group (with per-instance matrix/color).
   * Instances autoplaying the same clip from scene load stay equal forever;
   * a scripted play()/speed change simply splits the group. */
  bool poseEquals(const SkelInstance& other) const {
    if (overridden || other.overridden || model != other.model) return false;
    if (auxWeight != other.auxWeight ||
        (auxWeight > 0.0F && (aux.clip != other.aux.clip || aux.time != other.aux.time)))
      return false;
    // The base pose as (clip, time, weight) triples: a clip-mode pose is one
    // triple, a blend-space pose its shown samples - so a body showing one
    // sample (blend LOD) shares with a clip-mode body on that clip and time.
    s32 ca[3], cb[3];
    float ta[3], tb[3], wa[3], wb[3];
    const int na = baseKey(ca, ta, wa), nb = other.baseKey(cb, tb, wb);
    if (na < 0 || na != nb) return false;  // a crossfade in flight never shares
    for (int k = 0; k < na; k++)
      if (ca[k] != cb[k] || ta[k] != tb[k] || wa[k] != wb[k]) return false;
    return true;
  }

 private:
  struct Layer {
    s32 clip = -1;
    float time = 0.0F;
    bool loop = false;
    bool held = false;  // placed by seek(): never advances, fading out either
    std::vector<u32> cursors;  // per-channel last key index (times ascend)
  };

#if TYRA_SKEL_PROFILE
  u32 profileCount = 0, profilePose = 0, profileSkin = 0;
#endif
  Layer cur, prev;                // prev is only alive during a crossfade
  float fadeDuration = 0.0F;
  float fadeT = 1.0F;             // 0 = all prev, 1 = all cur (fade done)
  Layer aux;                      // the held blend layer (setBlend), -1 = off
  float auxWeight = 0.0F;         // 0 = all playing pose, 1 = all aux
  // Blend-space mode (setBlendSpace): the shown samples as layers, their
  // weights, the smoothed input point, the sync phase, the own-time clocks
  // (per sample of the space) and the blend LOD's pick.
  s32 bsSpace = -1;
  Layer bs[3];
  float bsWeight[3] = {0.0F, 0.0F, 0.0F};
  int bsCount = 0;
  blendspace::Point bsPoint{0.0F, 0.0F};
  float bsInDir = 0.0F, bsInSpeed = 0.0F;  // the raw input (setBlendInput)
  bool bsPointSet = false;        // false: the next advanceBlend starts AT the input
  float bsPhase = 0.0F;           // 0..1, shared by the sync samples
  float bsHeldPhase = -1.0F;      // >= 0: the game holds the phase
  std::vector<float> bsOwnClock;  // seconds, per sample (own-time ones)
  bool bsSingle = false, bsWasSingle = false;
  int bsLodSample = -1;           // the sample shown alone (blend LOD)
  // A FROZEN pose a switch fades from (blend-space mode in or out, a blend
  // LOD change): sampled once at the switch, sized on first use.
  std::vector<float> localsFrom;
  std::vector<u8> animatedFrom;
  float fromT = 1.0F, fromDuration = 0.0F;
  bool poseDirty = true;          // initial pose not yet skinned
  bool overridden = false;        // setPoseOverride: the game drives the pose
  bool overrideDirty = false;     // the override changed since the last skin
  std::vector<M4x4> overrideGlobals;  // per node, model space (setPoseOverride)
  bool posedOnce = false;         // globals hold a real pose (nodeGlobal)
  bool globalsStale = true;       // the pose changed since evalPose last ran
  bool adoptedBehind = false;     // the globals are an owner's, itself behind its clips
  bool clipStale = true;          // the clips moved since evalClipGlobals last ran
  std::vector<M4x4> clipGlobals;  // the clips' pose under an override (clipNodeGlobal)
  bool following = false;         // the renderer drew another's skin for it
  bool oneShotDone = false;
  u8 lastSkinnedLod = 0;          // which level the out arrays hold
  u8 maxLodLevels = 1;            // longest per-part chain incl. the base

  // scratch buffers, sized once in the constructor
  std::vector<float> localsCur, localsPrev;  // nodes * 10: t[3] r[4] s[3]
  std::vector<u8> animatedCur, animatedPrev; // node had a channel this pose
  std::vector<M4x4> globals;                 // per node
  std::vector<M4x4> palette;                 // per palette slot

  std::vector<std::vector<PartLod>> partLods;  // [part][lod]
  // [part][lod]: the float rows of the levels this body skins as float arrays (skinParts, the
  // vector stream path); empty for a body that streams only.
  std::vector<std::vector<std::shared_ptr<const SkelBindLod>>> floatBinds;
  std::vector<u8> partLevel;  // [part]: the level skinParts last skinned
  Compact::Box skinBox{};     // the output box of compact skinning (skinOutputBox)
  bool skinBoxReady = false;
  bool floatMeshReleased = false;  // releaseFloatMesh has run
  bool streamOnly = false;         // made without the float mesh arrays (render slots)
  std::vector<StaPipCompactLayout> streamLayouts;  // [part]; empty = the default
  bool compactValid = false;  // a skinInto pass has run
  u8 compactLod = 0;          // its level
  u32 compactPasses = 0;

  void advanceLayer(Layer& layer, float dt);
  /** The pose before the held layer: the clip (crossfade included) or the
   * blend space's samples, then the frozen pose by fromT, into localsCur. */
  void evalBaseLocals();
  /** Samples the pose showing now and fades from it over `seconds`. */
  void freezeFrom(float seconds);
  void advanceFrom(float dt);
  /** The base pose's identity for pose sharing: -1 while any crossfade is
   * in flight, else the count of (clip, time, weight) triples written. */
  int baseKey(s32 clips[3], float times[3], float weights[3]) const;
  void evalLocals(Layer& layer, std::vector<float>& locals,
                  std::vector<u8>& animated);
  /** locals = lerp(from, locals, w) per node (nlerp on the rotation), and
   * the animated flags merge - the one blend both the crossfade and the
   * held layer use. */
  void blendLocals(const std::vector<float>& from,
                   const std::vector<u8>& fromAnimated, float w);
  void evalGlobals();  // the node matrices only: evalPose less the palette
  void evalPose();
  /** The clips' pose (crossfade and held layer included) into `out`. */
  void evalClipGlobals(std::vector<M4x4>& out);
  void skinParts(u8 lod);
  /** Holds the float rows of a part level (SkelBindLod::floatLevel), made when no body has them;
   * false, with nothing made, while the Skinning budget has no room for them. */
  bool ensureFloatLevel(size_t part, u8 level);
  /** Makes both sides of a part's level (nothrow, under the Skinning budget) when
   * missing; false while refused. */
  bool ensureStreams(size_t part, u8 level);
  /** The indexed stream of a part level (TYRA_SKIN_INDEXED): made on first need - the level's
   * shared packaging when no body has built it, the body's stream and first pose buffer.
   * False when the flag is off, the part's layout is not the indexed program's (textured, lit,
   * the package size the blocks were built for) or memory is refused; the part then keeps its
   * plain streams. */
  bool ensureIndexed(size_t part, u8 level);
  /** Gives the body's pose buffer of a level back to the pool and frees its stream. */
  void dropIndexed(PartLod& pl);
  /** One plain part level's pass into `target` (the fixed / unique / vector cores). */
  u32 skinPlainPart(size_t part, u8 level, StaPipCompactStream& target, bool& foldedReady);
};

}  // namespace Tyra
