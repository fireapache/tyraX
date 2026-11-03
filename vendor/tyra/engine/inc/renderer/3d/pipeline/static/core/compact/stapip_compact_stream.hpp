/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# TyraX
#-----------------------------------------------------------------------
*/

// Added by TyraX: the compact stream (docs/compact-vertex-format.md, "The
// stream"). A bag's vertices stored once, in compact form, laid out exactly
// as a baked VIF block: per package three entry quadwords, a run header and
// its payload per attribute, a kick quadword - so a wholly visible bag replays
// it with one REF. The core takes float arrays and includes only tamtypes.h,
// so the host verifier runs it; StaPipBag and the classifier's boxes are EE
// types, and their half (forBag, packBag, boxForBag, bboxes) is built for the
// EE only. stapip_compact_tyra.hpp adds Vec4 / Color overloads.
#pragma once

#include <tamtypes.h>
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

namespace Tyra {

class StaPipBag;
class StaPipBagPackagesBBox;

/**
 * What a stream holds, per package: the attributes in the order the program
 * unpacks them (vugen's attrBlocks: position, then ST - or the env normal -
 * when textured, then normals when lit, else colours unless one colour serves
 * the whole bag; a billboard's centres, weights, colours), each one run.
 */
struct StaPipCompactLayout {
  /** Vertices a package (the last may hold fewer): the bag's package size,
   * 3..256 (an UNPACK moves at most 256). */
  u16 packageSize;
  u8 attrCount;
  Compact::Attr attrs[3];
  /** Each package is one triangle strip (StaPipBag::stripped). */
  bool stripped;
  bool billboard;

  /** The layout of a program class (vugen's attrBlocks): `texture` adds ST -
   * the env normal with `env`, which wins over `lit`; `lit` puts normals where
   * the colours go; `singleColor` drops the colours. */
  static StaPipCompactLayout forClass(bool texture, bool env, bool lit,
                                      bool singleColor, u32 packageSize,
                                      bool stripped);
  /** Billboards: centres, 2x2 basis weights, colours. */
  static StaPipCompactLayout forBillboards(u32 packageSize);
  /** The layout the program StaPipCore picks for `bag` reads (EE builds). */
  static StaPipCompactLayout forBag(const StaPipBag& bag, u32 packageSize);

  /** The VU1 address a package of `n` vertices unpacks attribute `attrIndex`
   * to, relative to TOPS: VERT_DATA + attrIndex x n. */
  u32 dest(u8 attrIndex, u32 n) const;
  /** The index of `a` in attrs, -1 when the layout has none. */
  s32 indexOf(Compact::Attr a) const;

  bool operator==(const StaPipCompactLayout& o) const;
  bool operator!=(const StaPipCompactLayout& o) const { return !(*this == o); }
};

/**
 * A bag's vertices: one 16-byte aligned block in the baked block's layout,
 *   per package p:  entry (3 qw) | per attribute a: header (1 qw) + run | kick
 * where a run holds the package's elements of one attribute in compact form,
 * zero-padded to whole quadwords (the padding reads as VIF NOPs), and its
 * header is Compact::runHeader. The entry and kick quadwords are the program's
 * (count, GIF tag, MSCAL); the stream leaves them NOPs for the renderer to
 * capture. Beside the block: one box per third of a package (a "part",
 * packageSize / 3 vertices), made when the stream is written.
 *
 * Not in the block: Compact::bagSetup(box) (STCYCL, STMOD 0, the box's origin
 * index in ROW, COL), which the renderer sends once per bag before anything
 * the bag unpacks, its object data included.
 */
class StaPipCompactStream {
 public:
  StaPipCompactStream();
  ~StaPipCompactStream();
  StaPipCompactStream(const StaPipCompactStream&) = delete;
  StaPipCompactStream& operator=(const StaPipCompactStream&) = delete;
  StaPipCompactStream(StaPipCompactStream&& o) noexcept;
  StaPipCompactStream& operator=(StaPipCompactStream&& o) noexcept;

  /**
   * Makes room for `vertexCount` vertices of `layout` in `box` - one nothrow
   * allocation in whatever memory scope is open (and, on the EE, the
   * classifier's boxes) - zeroed, the run headers written. False, with
   * nothing kept, when memory is refused or the arguments cannot make a
   * stream: no vertices, a package size outside 3..256, or a box that is not
   * a Compact::worldBox / modelBox one (a power-of-two step, the origin a
   * whole number of steps under 2^24). Releases what the stream held first.
   */
  bool allocate(const StaPipCompactLayout& layout, u32 vertexCount,
                const Compact::Box& box);
  void release();
  bool allocated() const { return block != nullptr; }
  /** Allocated with this layout, vertex count and box, bit for bit. */
  bool matches(const StaPipCompactLayout& layout, u32 vertexCount,
               const Compact::Box& box) const;

  u32 vertexCount() const { return count; }
  u32 packageCount() const { return packages; }
  u32 packageVertices(u32 p) const;

  /** Attribute `a`'s payload in package `p` (elements in vertex order). */
  u8* run(u32 p, u8 a);
  const u8* run(u32 p, u8 a) const;
  /** Its header quadword - the payload follows it. */
  const u8* runHeader(u32 p, u8 a) const;
  /** Its payload's quadwords, padding included (Compact::runQwords). */
  u32 runQwords(u32 p, u8 a) const;
  /** The three quadwords in front of package `p`'s runs, and the one after. */
  u8* entry(u32 p);
  u8* kick(u32 p);

  const void* data() const { return block; }
  u32 qwords() const { return total; }

  const Compact::Box& box() const { return bx; }
  const StaPipCompactLayout& layout() const { return lay; }

  /** A billboard stream's weight scale (Compact::weightScaleFor): the weights
   * are stored x it, the renderer divides the uploaded basis by it. */
  float weightScale() const { return wscale; }
  void setWeightScale(float scale) { wscale = scale; }

  /**
   * Done writing: the part boxes are made from the stored positions, the
   * classifier's boxes (EE) from them, and contentVersion moves. A writer's
   * end() calls it; so does Compact::packBag.
   */
  void finish();

  /** The parts: packageSize / 3 vertices each, the last one shorter. */
  u32 partCount() const { return parts; }
  /** A part's box - the smallest that holds its decoded positions. */
  void partBox(u32 part, float mn[3], float mx[3]) const;
  /** The same boxes as StaPipCore classifies with (EE builds only). */
  const StaPipBagPackagesBBox& bboxes() const;

  /** Moves whenever the bytes may have changed: allocate and finish. */
  u32 contentVersion() const { return version; }

 private:
  u32 packageQwords(u32 p) const;
  u32 runOffset(u32 p, u8 a) const;  // the header's quadword in the block
  void moveFrom(StaPipCompactStream& o);
  // The classifier's boxes: EE builds keep a StaPipBagPackagesBBox.
  bool makeBBoxes();
  void refreshBBoxes();
  void freeBBoxes();

  u8* raw;    // the allocation
  u8* block;  // `raw` aligned to 16
  float* partBoxes;  // six floats a part: min x, y, z, max x, y, z
  StaPipBagPackagesBBox* packagesBBox;
  StaPipCompactLayout lay;
  Compact::Box bx;
  float wscale;
  u32 count, packages, parts, total;
  u32 fullQw, lastQw;          // a full package's quadwords, the last one's
  u16 fullRun[3], lastRun[3];  // a run header's quadword in its package
  u32 version;
};

/**
 * Writes a stream in vertex order: per vertex the attributes its layout has
 * (any order; one it does not have is ignored), then next(); end() when done.
 * Positions are quantized in the stream's box as they come, colours, normals
 * and weights likewise; ST is held until its package is complete and then
 * rebased by whole repeats, each coordinate on its own (Compact::stRebase).
 */
class StaPipCompactWriter {
 public:
  explicit StaPipCompactWriter(StaPipCompactStream& stream);

  /** Position, or a billboard's centre. */
  void position(const float p[3]);
  void st(float u, float v);
  /** Colour, or a billboard's colour: 0..255 a channel. */
  void color(const float rgba[4]);
  /** Normal - or the env normal, normalised as it is written. */
  void normal(const float n[3]);
  /** A billboard's 2x2 basis weights, stored x the stream's weightScale. */
  void bbParams(const float m[4]);
  void next();
  /** Completes the last package and finishes the stream. */
  void end();

  /** Packages whose ST still left [-8, 8) after the rebase and saturated -
   * a triangle spanning more than 16 repeats. */
  u32 stClamped() const { return clamped; }

  static constexpr u32 kMaxPackage = 256;

 private:
  void openPackage(u32 p);
  void closePackage();

  StaPipCompactStream* s;
  u32 package;    // the package being written
  u32 k;          // the vertex's index there
  u32 n;          // the package's vertices
  u8* runs[3];    // the package's payloads
  s32 posSlot, stSlot, colorSlot, normalSlot, paramsSlot;
  bool envNormals;
  u32 posBytes;
  u32 clamped;
  float stBuf[2 * kMaxPackage];  // the open package's (u, v)
};

/**
 * Reads a stream: one vertex at a time, decoded to floats, or positions in a
 * batch (VU0 and MMI on the EE). Positions decode to the grid point the VU
 * sees, step x (q + the origin's index); ST to what the stream holds - the
 * rebased value; colours to 0..255; normals to q / 127. An attribute the
 * layout lacks reads 0. A view reads the stream's layout and box when it is
 * made: make a new one after the stream is allocated again.
 */
class StaPipCompactView {
 public:
  explicit StaPipCompactView(const StaPipCompactStream& stream);

  u32 count() const;
  void position(u32 i, float out[4]) const;  // x, y, z, 1
  void st(u32 i, float out[2]) const;
  void color(u32 i, float out[4]) const;
  void normal(u32 i, float out[3]) const;    // the normal or the env normal
  void bbParams(u32 i, float out[4]) const;  // the weights, / weightScale
  /** Positions first .. first + n - 1, 4 floats each (x, y, z, 1); a 16-byte
   * aligned `out4` takes the VU0 path. */
  void positions(u32 first, u32 n, float* out4) const;

 private:
  const u8* element(u32 i, s32 slot) const;

  const StaPipCompactStream* s;
  s32 posSlot, stSlot, colorSlot, normalSlot, paramsSlot;
  u32 posBytes;
  s32 row[3];  // the box's origin index (Compact::rowFor)
  float step;
};

/** A float bag's arrays as the packer reads them: 4 floats an element (Vec4,
 * Color), null when absent. */
struct StaPipCompactSource {
  const float* positions;  // x, y, z (a billboard's centres)
  const float* sts;        // u, v; the env normal; a billboard's weights
  const float* colors;     // r, g, b, a, 0..255; null with one colour
  const float* normals;    // x, y, z
  u32 count;
};

namespace Compact {

/**
 * Packs float arrays into `out` - what a writer given the same values writes,
 * byte for byte (positions through the batch quantizer) - and finishes it.
 * `out` keeps its block when it already matches (layout, count, box), else is
 * allocated; false when that is refused. A billboard stream's weight scale is
 * made from its largest weight. `stClamped` (optional) gets the writer's
 * count.
 */
bool packArrays(const StaPipCompactLayout& layout,
                const StaPipCompactSource& source, const Box& box,
                StaPipCompactStream& out, u32* stClamped = nullptr);

/** A float bag packed into `out` (EE builds): its layout, its arrays. */
bool packBag(const StaPipBag& bag, u32 packageSize, const Box& box,
             StaPipCompactStream& out);

/**
 * The box for `count` positions 4 floats apart: world space - a bag whose
 * model matrix is exactly the identity - takes worldBox at the scene step
 * (`coarse` set when it did not fit), anything else modelBox.
 */
Box boxFor(const float* positions, u32 count, bool worldSpace, bool* coarse);

/** The box for a bag (EE builds): boxFor its positions, world space when its
 * model matrix is the identity; a coarse world box is logged once a bag:
 * `[COMPACT] coarse box bag=<address> extent=<m>`. */
Box boxForBag(const StaPipBag& bag);

/** The scene step every world bag uses (sceneStepFor the scene's largest
 * world bag): a power of two, at least 2^-10 m; the generated game sets it at
 * scene load. Until then 2^-9 m (a 128 m bag). */
void setSceneStep(float step);
float sceneStep();

}  // namespace Compact
}  // namespace Tyra
