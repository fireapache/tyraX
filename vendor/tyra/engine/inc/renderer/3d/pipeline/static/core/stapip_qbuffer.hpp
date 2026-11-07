/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Sandro Sobczyński <sandro.sobczynski@gmail.com>
*/

#pragma once

#include <math3d.h>
#include <string>
#include "debug/debug.hpp"
#include "./bag/packaging/stapip_bag_package.hpp"
#include "./bag/stapip_bag.hpp"
#include "./compact/stapip_compact_stream.hpp"
#include "./stapip_probes.hpp"

namespace Tyra {

/**
 * Modified by TyraX: where the bag being drawn keeps its vertices in the
 * compact vertex format (docs/compact-vertex-format.md). The renderer sets it
 * once per bag (StaPipQBufferRenderer::setCompactSource), before any fill: a
 * fill reads a package straight out of `stream`, or packs it - from the
 * stream's elements, or from the bag's float arrays when it has no stream -
 * into the copy pool, in `box`, as `layout`'s runs.
 */
struct StaPipQBufferSource {
  const StaPipBag* bag = nullptr;
  /** The bag's stream (its own, or the one the renderer packed for it), or
   * null: every fill packs the bag's float arrays. */
  const StaPipCompactStream* stream = nullptr;
  /** The box the bag's positions are quantized in - the stream's, or the one
   * a fill packs into (the bag's bagSetup and MVP x D carry it). */
  Compact::Box box = {{0.0F, 0.0F, 0.0F}, 1.0F};
  /** The layout the bag's program reads; packageSize is this draw's. */
  StaPipCompactLayout layout = {};
  /** A billboard bag's weight scale (Compact::weightScaleFor). */
  float weightScale = 4096.0F;
  /** Added by TyraX: the stream is INDEXED (StaPipCompactStream::isIndexed) and this draw may
   * send its packages as they are - the indexed cull program is resident and the bag's class is
   * the one it replaces. False sends every package expanded to plain corners. */
  bool indexedOk = false;
};

class StaPipQBuffer {
 public:
  StaPipQBuffer();
  ~StaPipQBuffer();

  void setMaxVertCount(const u32& count);
  // Modified by TyraX: switch the copy pools to their other side - call once
  // per packet SEND, right where the packet double buffer flips (see the pool
  // comment in stapip_qbuffer.cpp).
  static void flipPoolSide();
  // Modified by TyraX: has any slot copied into the CURRENT pool side since
  // the last flip? A packet that references such a copy must be sent before
  // the next bag reuses the slot - see StaPipQBufferRenderer::flushBuffers.
  static bool poolTouched();
  /** Diagnostic: is this address inside one of the copy pools? See the
   * definition - it exists to settle where the acceptance gate's geometry
   * hash gets its irreproducibility from. */
  static bool isPoolAddress(const void* addr);

  /** Modified by TyraX: the bag every fill reads, until the next call. */
  static void setSource(const StaPipQBufferSource& source);
  static const StaPipQBufferSource& source();
  /** Packages packed into the pool since the last take (StaPipTelemetry). */
  static u32 takePackedPackages();

  /**
   * Modified by TyraX: package `p` of `stream`, in place - each run is the
   * stream's own header and payload, nothing is copied.
   */
  void fillByStream(const StaPipCompactStream& stream, u32 p);

  /**
   * Added by TyraX: package `p` of an INDEXED stream, in place - its pose block and its static
   * block (the corner table), two REFs; nothing is copied. `run[0]` / `runQwords[0]` name the
   * pose block, `run[1]` / `runQwords[1]` the static block's corner header and table, and
   * `indexedStream` marks the buffer for the indexed program. Only the cull route sends such a
   * buffer as it is; expandIndexed turns it into plain corners for the clip routes.
   */
  void fillByIndexed(const StaPipCompactStream& stream, u32 p);
  /** The indexed stream this buffer is a package of, or null (a plain buffer). */
  const StaPipCompactStream* indexedStream;
  /** Added by TyraX: makes an indexed buffer a plain one - the package's corners decoded into
   * the copy pool as the source layout's runs, ready for the clip programs. Keeps the buffer's
   * bag, clip planes and package. A buffer the pool refuses is left empty (not drawn). */
  void expandIndexed();
  /** Indexed packages sent as they are / expanded to plain corners since the last take. */
  static void takeIndexedCounts(u32* sent, u32* expanded);
  /** The renderer sent `n` indexed packages as they are. */
  static void noteIndexedSent(u32 n = 1);

  /**
   * Modified by TyraX: `pkg`'s vertices packed into the copy pool as the
   * source layout's runs - read from the source stream's elements (copied,
   * so the integers are the stream's) or quantized from the bag's float
   * arrays. False when the pool refused the memory: the buffer is empty
   * (size 0) and the package is not drawn this frame.
   */
  bool fillByPack(const StaPipBagPackage& pkg);

  /**
   * The package as it is: in place when it is one of the source stream's
   * packages (fillByStream), else packed into the pool (fillByPack).
   * Modified by TyraX: the name predates the compact format - it used to
   * point the buffer at the bag's float arrays.
   */
  void fillByPointer(const StaPipBagPackage& pkg);

  /**
   * One contiguous range of the source bag: in place when it is one of its
   * stream's packages, else packed into the pool.
   */
  void fillByPointer(StaPipBag* bag, u32 offset, u32 count);

  /** Three packages together, packed into the pool. */
  void fillByCopyMax(const StaPipBagPackage& pkg1, const StaPipBagPackage& pkg2,
                     const StaPipBagPackage& pkg3);

  /** Two packages together, packed into the pool. */
  void fillByCopy1By2(const StaPipBagPackage& pkg1,
                      const StaPipBagPackage& pkg2);

  /** One package, packed into the pool (the EE clipper's input). */
  void fillByCopy1By3(const StaPipBagPackage& pkg);

  /**
   * Modified by TyraX: expand `triCount` triangles of a STRIPPED package back
   * into an ordinary triangle list, packed into the copy pool.
   *
   * The clip programs and the EE clipper both loop by whole triangles over a
   * triangle list, so a stripped package that genuinely crosses a clip plane
   * cannot be handed to either as it stands. It is expanded here instead -
   * even triangle i is (v[i], v[i+1], v[i+2]) and an odd one swaps its first
   * two vertices to preserve the strip's alternating winding. The result is an
   * ordinary list qbuffer that every existing route already handles. The
   * buffer's own `stripped` flag is cleared, which is what makes the GIF tag
   * this buffer produces say PRIM_TRIANGLE, so one bag may mix the two.
   *
   * `firstTri` is an index into the package's triangles, not its vertices.
   * The expansion is 3x, so the caller must chunk it: at most maxVertCount / 3
   * triangles at a time, which is the same triangle budget the list path's
   * subpackages carry.
   */
  void fillByStripExpand(const StaPipBagPackage& pkg, u32 firstTri,
                         u32 triCount);

  /**
   * Modified by TyraX: the EE clipper's float input - the buffer's compact
   * runs decoded into the pool's float arrays (`vertices`, `sts`, `colors`,
   * `normals`): positions the grid point the VU sees, ST and colours as
   * stored, normals and env normals as their int8 values x 127 (the rows
   * that multiply a normal carry the 1/127, docs/compact-vertex-format.md).
   * False when the pool refused the memory.
   */
  bool decodeToFloats();

  /**
   * @brief Deallocate dynamic data if it was allocated and allocate new data
   * specified by size. The EE clipper's float output for the as_is programs.
   * @param size 48 is max
   */
  void reallocateManually(const u16& size);

  bool any() const;

  StaPipBag* bag;

  /** Float streams: the as_is programs' input (the EE clipper's output) and
   * the EE clipper's input (decodeToFloats). Null otherwise. */
  Vec4* vertices;
  Vec4* sts;
  Vec4* colors;
  Vec4* normals;
  u32 size;

  /**
   * Modified by TyraX: the buffer's per-vertex blocks in the compact vertex
   * format (docs/compact-vertex-format.md), one run a block in the program's
   * block order (vertices, then ST, then normals or colours). `run[k]` points
   * at the block's header quadword - STMASK, the mask, STMOD and the UNPACK,
   * which names the block's own VU1 address - with its payload right behind
   * it, `runQwords[k]` is that payload in quadwords (Compact::runQwords). A
   * program's addProgramQBufferDataToPacket sends each with one REF
   * (Packet2TyraUtils::addCompactRun); the fills that make a buffer set them.
   */
  const void* run[3];
  u32 runQwords[3];
  /** Modified by TyraX: the runs lie back to back from run[0] (every fill
   * lays them out so): `bodyQwords` headers and payloads in all, which the
   * renderer sends with ONE REF. 0 for a float buffer. */
  u32 bodyQwords;
  /** Conservative OR of the source packages' exact VU clip-plane masks. */
  u8 clipPlaneMask;

  /**
   * Modified by TyraX: this buffer's vertices are a triangle STRIP, so the
   * GIF tag the microprogram writes must carry PRIM_TRIANGLE_STRIP. It is per
   * BUFFER and not per bag on purpose - a stripped bag's clip-routed packages
   * are expanded back to lists (fillByStripExpand) and travel in the same
   * flush as its stripped ones.
   */
  bool stripped;

  /**
   * Modified by TyraX: which package of the current bag's RETAINED command
   * block this buffer is, or -1 when it has none (a copied, clipped or
   * strip-expanded buffer, a bag the cache refused, or the feature compiled
   * out). See StaPipRetainedCommands in stapip_qbuffer_renderer.hpp.
   *
   * StaPipQBufferRenderer::getBuffer() resets it - the one gate every buffer
   * passes through before any fill - so a slot recycled out of a retained bag
   * cannot carry a stale index into a copied or clipped one. StaPipCore sets
   * it back after the fill, for the routes that may be retained.
   */
  int retainIndex;

  /**
   * Modified by TyraX: this buffer's package index inside its bag's compact
   * STREAM when that stream's entry and kick quadwords may be replayed for it
   * (docs/baked-vif-stream.md), or -1. Set only by the DIRECT, wholly-visible
   * cull route - the one route whose packages are a fixed slice of the bag -
   * and reset by getBuffer() like retainIndex above.
   */
  int bakeIndex;
  /** Modified by TyraX: the buffer is package `streamPackage` of the source
   * stream, filled in place (fillByStream), or -1 (packed into the pool). */
  int streamPackage;
#if TYRA_STAPIP_PROBE_UNCACHED_CHAIN
  /**
   * Probe B only (stapip_probes.hpp): this buffer's streams live in the
   * qbuffer COPY POOL, written by the EE between sends, rather than in the
   * bag's own long-lived arrays. The DMA REF tags then point at memory whose
   * dirty cache lines only `FlushCache` writes back, so a send carrying one of
   * these must keep the flush. Cleared by getBuffer(), set by the copy fills
   * and by StaPipClipper::writeChunk.
   */
  bool probeCopyFilled;
#endif

  void print() const;
  void print(const char* name) const;
  void print(const std::string& name) const { print(name.c_str()); }
  std::string getPrint(const char* name = nullptr) const;

 private:
  u32 maxVertCount;
  /** Pack `n` vertices of the source bag - `index[i]`, or `first + i` when
   * `index` is null - into this slot's compact pool block. */
  bool pack(const u32* index, u32 first, u32 n);
  void clearRuns();
  void deallocateDynamicData();
  void allocateDynamicData(u16 size, StaPipBag* bag);
  u8 _isDynamicallyAllocated, _stAllocated, _colorAllocated, _normalAllocated;
};

}  // namespace Tyra
