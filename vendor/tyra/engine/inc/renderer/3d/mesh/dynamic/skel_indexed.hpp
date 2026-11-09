/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the skinned compact stream with indexed corners.
*/

#pragma once

// The skinned compact stream with INDEXED corners (docs/animated-models.md, "Skinned streams
// with indexed corners").
//
// A skinned part's compact stream stores TRIANGLE CORNERS: a corner is a vertex of one
// triangle, and a vertex is shared by about five of them, so every pose the skin stores each
// unique vertex five times. This layout stores each unique vertex ONCE per package and lets
// VU1 assemble the triangles by index:
//
//   static, per model part level (shared by every body, never rewritten):
//     per package: the program's entry, the CORNER TABLE (S, T, Q and the unique's address,
//     one V4-16 element a corner) and the kick - the "static block";
//   per pose, per body:
//     per package: the package's UNIQUE vertices, positions (V4-16, 8 bytes) and normals
//     (V4-8, 4 bytes), each a compact run - the "pose block".
//
// The package boundaries are the plain stream's (the same packageSize corners a package), so a
// package's corners, its parts (a third of a package, the frustum classifier's boxes) and its
// ST run are the plain stream's, byte for byte (the host verifier proves it). A unique vertex
// two packages use is in both of them (`duplication()`).
//
// VU1 buffer, TOPS-relative (a half of the double buffer: kHalfQw quadwords), n corners and
// u <= n uniques in the package:
//
//   qw 0                      scale xyz, count (the program's header, as now)
//   qw 1                      the GIF prim tag
//   qw 2 .. 2 + n             the corner table: lanes S, T, Q (int32, 4.12 x scale) and the
//                             address of the corner's unique, a TOPS-relative quadword index
//                             (uniqueAddr(n) + local index): `ilw.w` reads it as it is
//   qw 2 + n ..               the program's output: the GIF tag block (up to 9 qw) and three
//                             qw a corner - written by VU1, never by the VIF
//   qw uniqueAddr(n) = 11 + 4n .. + u               the unique positions (grid index in w = 1)
//   qw uniqueAddr(n) + kUniqueSplit .. + u          the unique normals (int8 x 127)
//
// so a corner's position is at its address and its normal kUniqueSplit quadwords on: one
// address register and two `lq`, the second with a constant offset. The split is a constant
// of the program (not of the package): a package fits when 11 + 4n + kUniqueSplit + u <=
// kHalfQw. A package of kMaxPackage corners therefore holds at most 74 uniques; one whose
// corners are all different (a flat-shaded model) is refused and the part takes smaller packages.
//
// Plain C++ (tamtypes.h and the compact format only): the host verifier
// (tools/verify-skel-indexed.cpp) runs exactly this code.

#include <tamtypes.h>

#include <vector>

#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"

namespace Tyra {
namespace SkelSkin {
namespace Indexed {

/** Quadwords of one half of the VU1 double buffer the renderer fills: (VU1_STAPIP_DBUFFER_END
 * 944 - the program constants' 22) / 2 = 461, less the last quadword, which belongs to the other
 * half's header (StaPipQBufferRenderer::setDoubleBuffer). A package must leave the output
 * (3 qw a corner + the tag block) and the uniques inside it. */
constexpr u32 kHalfQw = 460;
constexpr u32 kHeaderQw = 2;     // scale / count, prim tag
constexpr u32 kTagBlockQw = 9;   // the GIF tag block the program stores in front of its vertices
constexpr u32 kMaxPackage = 75;  // the plain cull_td package, and the most the split holds
/** Normals sit this many quadwords after the positions. Both unique arrays are at most
 * kMaxPackage long, so the two never overlap. */
constexpr u32 kUniqueSplit = kMaxPackage;

/** The first corner-table quadword. */
constexpr u32 kCornerAddr = kHeaderQw;
/** The TOPS-relative quadword of unique 0 of an n-corner package: past the output. */
constexpr u32 uniqueAddr(u32 n) { return kHeaderQw + n + kTagBlockQw + 3 * n; }
/** True when an n-corner package with u uniques fits one half of the double buffer. */
constexpr bool fits(u32 n, u32 u) {
  return n >= 3 && n <= kMaxPackage && u <= n && u <= kUniqueSplit &&
         uniqueAddr(n) + kUniqueSplit + u <= kHalfQw;
}

constexpr u32 kPosBytes = 8;  // a unique's position: int16 x, y, z and a pad (V4-16)
constexpr u32 kNrmBytes = 4;  // a unique's normal: int8 x, y, z and a pad (V4-8)
constexpr u32 kCornerBytes = 8;  // a corner table element: int16 S, T, Q and the address

/** The bind-time packaging of one part level: which uniques each package holds and where every
 * corner finds its own. Shared by every body of the model. */
struct Level {
  u32 corners = 0;
  u32 packageSize = 0;  // corners a package; the last one may hold fewer
  u32 packages = 0;
  u32 uniques = 0;      // the level's unique corners (skinSource[v] == v), in corner order
  /** [corners] the level-unique index of each corner (the rank of its skinSource). */
  std::vector<u32> uid;
  /** [packages + 1] offsets into `src`. */
  std::vector<u32> first;
  /** [first.back()] the level-unique index of each package-unique, package by package in the
   * order its corners first use them. The pose gather reads this and nothing else. */
  std::vector<u32> src;
  /** [corners] each corner's local index in its package's unique list. */
  std::vector<u16> local;

  u32 packageCorners(u32 p) const {
    const u32 lo = p * packageSize;
    return corners - lo < packageSize ? corners - lo : packageSize;
  }
  u32 packageUniques(u32 p) const { return first[p + 1] - first[p]; }
  u32 totalUniques() const { return first.empty() ? 0 : first.back(); }
  /** Package-uniques over level uniques: how many times the pose stores a unique on average. */
  double duplication() const {
    return uniques == 0 ? 0.0 : (double)totalUniques() / (double)uniques;
  }
};

/** Builds `out` from a part level's skinSource (the first identical corner of each corner;
 * itself when unique). `packageSize` is the plain stream's: a multiple of 3, 3..kMaxPackage.
 * False when it is not, or a package would not fit the buffer, or - `*refused` set, when given -
 * the budget has no room for the tables (a later call may succeed). The tables are std::vectors,
 * so each stage first asks for its exact size from the nothrow operator new (skel_room.hpp): a
 * full budget refuses, it never halts on a throwing request. */
bool build(const u32* skinSource, u32 corners, u32 packageSize, Level& out,
           bool* refused = nullptr);

/** Bind-time triangle order for locality (optional; the plain order is the model's). Packs
 * triangles into packages of packageSize / 3 greedily: each next triangle is the unused one
 * touching the open package that adds the fewest new uniques, so a vertex's triangles land in
 * the same package and `duplication()` falls. `order[i]` is the original triangle that goes
 * i-th. A part drawn in this order is the same picture (draw order inside an opaque part does
 * not show) and is a different corner stream: the plain stream and the indexed one are both
 * built from the reordered part. */
void orderTriangles(const u32* skinSource, u32 corners, u32 packageSize,
                    std::vector<u32>& order);

/** Where each package's pose block starts in a body's pose buffer, in bytes (16-aligned). A
 * pose block is {pos header qw, pos run, nrm header qw, nrm run}, each run padded to whole
 * quadwords (`Compact::runQwords`-style, zero). */
struct PoseLayout {
  std::vector<u32> block;  // [packages + 1]; back() is the buffer's size
  static u32 posQw(u32 u) { return (u * kPosBytes + 15) / 16; }
  static u32 nrmQw(u32 u) { return (u * kNrmBytes + 15) / 16; }
  /** Offset of the normal run's header in a block. */
  static u32 nrmHeader(u32 u) { return 16 + posQw(u) * 16; }
  static u32 blockBytes(u32 u) { return 16 + posQw(u) * 16 + 16 + nrmQw(u) * 16; }
  u32 bytes() const { return block.empty() ? 0 : block.back(); }
};
void layoutPose(const Level& level, PoseLayout& out);

/** Writes the run headers (VIFcodes) of every pose block: constant for a level, written once
 * when the body's pose buffer is allocated. `buf` is layout.bytes() long. */
void initPose(const Level& level, const PoseLayout& layout, u8* buf);

/** The pose write: package by package, copies each package-unique's quantized position (8
 * bytes) and normal (4 bytes) from the level-unique arrays (`qPos` [uniques][4] s16, `qNrm`
 * [uniques][4] s8 - what both skin kernels produce: Unique::quantize and Batch::finishPose)
 * into the pose buffer. One store of each a package-unique, about `duplication()` stores a
 * level unique, in place of the plain stream's one a corner. */
void writePose(const Level& level, const PoseLayout& layout, const s16* qPos, const s8* qNrm,
               u8* buf);

/** Where each package's static block starts, in bytes (16-aligned). A static block is
 * {entry (3 qw), corner header qw, corner table, kick qw}. */
struct StaticLayout {
  std::vector<u32> block;  // [packages + 1]
  static u32 cornerQw(u32 n) { return (n * kCornerBytes + 15) / 16; }
  static u32 blockBytes(u32 n) { return 48 + 16 + cornerQw(n) * 16 + 16; }
  static constexpr u32 kEntryBytes = 48;
  u32 bytes() const { return block.empty() ? 0 : block.back(); }
};
void layoutStatic(const Level& level, StaticLayout& out);

/** Builds every static block: the entry quadwords from `entry` (3 qw a package, the program's
 * - null leaves NOPs), the corner table from `stq` ([corners][3] int16: the plain stream's ST
 * run elements, rebased and scaled per package) and each corner's address, the kick from
 * `kickCodes` (the two VIFcodes after STMOD 0: FLUSH, MSCAL; null leaves NOPs). */
void buildStatic(const Level& level, const StaticLayout& layout, const s16* stq,
                 const u32* entry, const u32* kickCodes, u8* out);

/** Statistics of a level, for the numbers table. */
struct Stats {
  u32 corners = 0, uniques = 0, packageUniques = 0, packages = 0;
  u32 maxPackageUniques = 0, minPackageUniques = 0;
  u32 poseBytes = 0;    // the pose buffer of one body, one side
  u32 staticBytes = 0;  // the shared static blocks
  u32 gatherBytes = 0;  // the shared gather list (u16 a package-unique)
  u32 plainBytes = 0;   // the plain stream of one side (the shipped layout)
};
Stats statsOf(const Level& level, const StaPipCompactLayout* plainLayout);

}  // namespace Indexed
}  // namespace SkelSkin
}  // namespace Tyra
