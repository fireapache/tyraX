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
// so the host verifier runs it; StaPipBag is an EE type, and its half
// (forBag, packBag, boxForBag) is built for the EE only.
// stapip_compact_tyra.hpp adds Vec4 / Color overloads.
#pragma once

#include <tamtypes.h>
#include "renderer/3d/pipeline/static/core/compact/stapip_compact.hpp"

namespace Tyra {

class StaPipBag;

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
  /** ST coordinates a package does NOT rebase by whole repeats: bit 0 u,
   * bit 1 v - a texture whose wrap is not REPEAT on that axis (CLAMP, a
   * region mode), where a whole-repeat shift would move the picture. Their
   * values beyond [-8, 8) saturate instead, which under CLAMP samples the
   * same edge texel (docs/compact-vertex-format.md, "ST"). */
  u8 stFixed;

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
 * capture. Beside the block, in the same allocation: one box per third of a
 * package (a "part", packageSize / 3 vertices), one per group of
 * kCoarseParts parts and one for the whole stream - the boxes a frustum
 * classifier reads, made when the stream is written.
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
   * allocation in whatever memory scope is open, the boxes included - zeroed,
   * the run headers written, the weight scale back at its default. False, with
   * nothing kept, when memory is refused or the arguments cannot make a
   * stream: no vertices, a package size outside 3..256, or a box that is not
   * a Compact::worldBox / modelBox one (a power-of-two step, the origin a
   * whole number of steps under 2^24). Releases what the stream held first -
   * so never call it on a stream a DMA chain may still read: allocate a
   * second stream instead.
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
   * are stored x it, the renderer divides the uploaded basis by it. Set it
   * after allocate() and before any weight is written; allocate and release
   * put it back to 4096 (Compact::packArrays sets it itself). */
  float weightScale() const { return wscale; }
  void setWeightScale(float scale) { wscale = scale; }

  /**
   * Done writing: the boxes are made from the stored positions (the part
   * boxes, then the coarse groups and the whole stream's box from them) and
   * contentVersion moves. A writer's end() calls it; so does
   * Compact::packArrays.
   */
  void finish();

  /** The parts: packageSize / 3 vertices each, the last one shorter. */
  u32 partCount() const { return parts; }
  /** A part's box - the smallest that holds its decoded positions. */
  void partBox(u32 part, float mn[3], float mx[3]) const;
  /** Every part's box, six floats a part: min x, y, z, then max x, y, z -
   * contiguous, for a classifier to read in place. Null when not allocated. */
  const float* partBoxes() const { return partBox_; }
  /** Parts a coarse group holds: 24, eight full packages. */
  static constexpr u32 kCoarseParts = 24;
  /** The coarse groups - parts [g x kCoarseParts, (g + 1) x kCoarseParts),
   * the last one shorter - each the union of its parts' boxes, six floats a
   * group like partBoxes(). */
  u32 coarseCount() const { return coarse; }
  const float* coarseBoxes() const { return coarseBox_; }
  /** The whole stream's box (the union of every part's), six floats. */
  const float* bounds() const { return wholeBox_; }

  /** Moves whenever the bytes may have changed: allocate and finish. */
  u32 contentVersion() const { return version; }

  /**
   * Renderer-owned (docs/compact-vertex-format.md, "Routes"): the key of the
   * program and state the entry and kick quadwords were captured for - all
   * zero while they are NOPs. allocate and release clear it.
   */
  static constexpr u32 kEntryKeyWords = 6;
  const u32* entryKey() const { return ekey; }
  bool entriesCaptured() const { return ekey[0] != 0; }
  void setEntryKey(const u32 key[kEntryKeyWords]);
  /**
   * Renderer-owned: package `p`'s entry and kick from a program's own words -
   * the two VIFcodes of its count / GIF-tag CNT (STMOD 0 and the UNPACK) and
   * that CNT's two quadwords, then the two VIFcodes of its kick (FLUSH, MSCAL
   * or MSCNT). Both quadwords' first slot is STMOD 0 whatever the program
   * wrote, the second a NOP: no package leaves VIF1 in offset MODE (a
   * position run's STMOD 1 persists - docs/compact-vertex-format.md). Written
   * only into a stream no chain reads yet (the stream's contract).
   */
  void setPackageWords(u32 p, const u32 entryCodes[2], const void* entryData,
                       const u32 kickCodes[2]);
  /** Every entry and kick back to NOPs and the key cleared - never half
   * captured. */
  void clearPackageWords();
  /** Renderer-owned: the renderer registered the block as an immutable span
   * (a stream the game owns, drawn from in place); on the EE release() - and
   * so allocate and the destructor - unregisters it first, which waits for
   * every DMA reader. */
  void setSpanRegistered(bool on) { spanRegistered = on; }
  bool isSpanRegistered() const { return spanRegistered; }

 private:
  u32 packageQwords(u32 p) const;
  u32 runOffset(u32 p, u8 a) const;  // the header's quadword in the block
  void moveFrom(StaPipCompactStream& o);

  u8* raw;    // the allocation
  u8* block;  // `raw` aligned to 16
  // Behind the block, six floats a box: min x, y, z, max x, y, z.
  float* partBox_;    // one a part
  float* coarseBox_;  // one a coarse group
  float* wholeBox_;   // the whole stream
  StaPipCompactLayout lay;
  Compact::Box bx;
  float wscale;
  u32 count, packages, parts, coarse, total;
  u32 fullQw, lastQw;          // a full package's quadwords, the last one's
  u16 fullRun[3], lastRun[3];  // a run header's quadword in its package
  u32 version;
  u32 ekey[kEntryKeyWords];  // entryKey()
  bool spanRegistered;
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

  /** Packages whose ST no whole-repeat shift brought inside the 4.12 range,
   * so values saturated (Compact::stRebase: wider than 15 repeats, at most
   * 16 - a triangle spanning about that many). */
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

/**
 * One run of attribute `a` written exactly as packArrays writes it, for `n`
 * vertices of `source` - vertex index[i], or first + i when `index` is null -
 * into `run` (the payload, its zero padding up to Compact::runQwords
 * included). ST is rebased over these `n` vertices as one package; billboard
 * weights are stored x `weightScale`. True when an ST value still saturated.
 * What a renderer packs a package, a slice or an expanded strip with.
 */
bool packRun(Attr a, const StaPipCompactSource& source, const u32* index,
             u32 first, u32 n, const Box& box, float weightScale, u8* run,
             u8 stFixed = 0);

/** The same from a stream: the elements of its attribute slot `a` for its
 * vertices index[i] (or first + i), copied as they are - the integers are
 * the stream's, an ST keeps its own package's rebase. */
void copyRun(const StaPipCompactStream& stream, u8 a, const u32* index,
             u32 first, u32 n, u8* run);

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
