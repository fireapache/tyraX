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

// Added by TyraX: a non-moving allocator over one byte range - a two-level
// segregated-fit heap (TLSF). The free blocks sit in lists by size class, two
// bitmaps say which lists are not empty, and a released block is merged with its
// free neighbours at once through the sizes every block header keeps of itself
// and of the block before it. release() takes constant time. So does allocate(),
// except for one walk of a single size-class list when only that class can hold
// the request (a size that is not at a class boundary and has no bigger block to
// take it from).
//
// It never grows and never moves a block by itself; slideDown() is the one
// operation that does, and only when asked. It keeps no state outside the object
// and the range it was given, so one RegionHeap per region is the whole idea.
//
// Every block starts with a 16-byte header (flags, size, the previous block's
// size, a budget id and an owner id), so the payload is 16-aligned and the
// budget and owner of any pointer are one read away. Sizes below count whole
// blocks, headers included, unless a function says "payload".
//
// For a tool that shows how the region is doing, the heap keeps two answers as
// it goes, so nobody ever walks it to ask: the number of free blocks, and - when
// it is given storage for one (attachMap) - an occupancy map: kMapCells cells
// over the range, each the bytes of used blocks in it. allocate, release and
// slideDown add or take a block's bytes in the cells it spans: two cells and a
// mark where it starts and stops covering whole cells, however big it is.
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#pragma once

#include <cstddef>
#include <cstdint>

namespace Tyra {
namespace Memory {

class RegionHeap {
 public:
  static constexpr size_t kHeader = 16;    // bytes in front of every payload
  static constexpr size_t kMinAlign = 16;  // the alignment every payload has
  static constexpr unsigned kMapCells = 64;  // cells of the occupancy map

  // Storage for an occupancy map (attachMap). Kept outside the heap: a manager
  // holds a RegionHeap for every budget id, and only the regions it shows need
  // one. `part[c]` is the used bytes of the blocks that cover part of cell c;
  // `full` marks, as differences, where a block starts (+1) and stops (-1)
  // covering whole cells, so a block of any size touches four entries.
  struct Map {
    uint32_t part[kMapCells];
    int8_t full[kMapCells + 1];
  };

  struct BlockInfo {
    void* payload;    // the address allocate() returned (or would return)
    size_t bytes;     // payload bytes: the block's size minus kHeader
    bool free;        // a free block has no budget or owner worth reading
    uint16_t budget;  // as passed to allocate()
    uint32_t owner;   // as passed to allocate(), or set by setOwner()
  };

  RegionHeap() = default;
  RegionHeap(const RegionHeap&) = delete;
  RegionHeap& operator=(const RegionHeap&) = delete;

  // Takes over [base, base + bytes) as one free block and forgets anything the
  // heap held before. base is 16-aligned, bytes a multiple of 16 between 32 and
  // 4 GB. The range stays the caller's; nothing is written outside it.
  void init(void* base, size_t bytes);

  // A payload of at least `size` bytes aligned to `align` (a power of two; less
  // than 16 means 16), charged to `budget` and `owner`. nullptr when nothing
  // fits - and then nothing has changed. Alignments above 16 over-allocate by up
  // to the alignment; such a block is never moved by slideDown().
  void* allocate(size_t size, size_t align, uint16_t budget, uint32_t owner);

  // Gives a block back and merges it with its free neighbours. nullptr is
  // ignored. With TYRA_MEMORY_CHECKS a pointer that is not a live allocation of
  // this heap (outside it, misplaced, released twice) goes to Memory::fail.
  void release(void* p);

  // Payload bytes the block really has (at least what was asked for).
  size_t usable(const void* p) const;
  uint16_t budgetOf(const void* p) const;
  uint32_t ownerOf(const void* p) const;
  void setOwner(void* p, uint32_t owner);

  // Whether p lies inside the range given to init() - a range test only.
  bool contains(const void* p) const;

  // Whether p looks like the payload of a live block of this heap: inside the
  // range, 16-aligned, with a header that is not free, has a sane size and agrees
  // with its neighbours' sizes. A best-effort sanity check of the header and its
  // neighbours, not a proof: it cannot see a stale pointer into a block that has
  // since been released and reused, and arbitrary payload bytes can pass for a
  // header.
  bool owns(const void* p) const;

  // Whether p was allocated with an alignment of 16 or less, so slideDown() may
  // move it.
  bool slidable(const void* p) const;

  // If the block in front of p's is free, moves p's block (header and payload)
  // to the start of that free block, leaves the room it vacated as one free
  // block (merged with the free block behind it, if any) and returns the new
  // payload address. Otherwise - or when p is not slidable - returns nullptr and
  // changes nothing. The caller owns the pointer fix-up.
  void* slideDown(void* p);

  // used + freeBytes == capacity, always. Both count whole blocks, headers and
  // the slack of over-aligned blocks included.
  size_t used() const { return used_; }
  size_t peak() const { return peak_; }   // the most used() has been since init
  size_t freeBytes() const { return capacity_ - used_; }
  size_t capacity() const { return capacity_; }

  // The largest allocation of align <= 16 that would succeed now: the payload of
  // the largest free block, 0 when there is none.
  size_t largestFree() const;

  // How many free blocks the range is in, kept as the heap goes (never a walk):
  // 1 for an empty heap, 0 for a full one. With freeBytes() and largestFree()
  // it says how fragmented the free space is.
  uint32_t freeBlocks() const { return freeBlocks_; }

  // Gives the heap an occupancy map to keep in `map` (filled from the blocks
  // it holds now - one walk, here and never again), or takes it away (null).
  // The storage stays the caller's and must outlive its use; init() keeps the
  // map and empties it.
  void attachMap(Map* map);
  bool hasMap() const { return map_ != nullptr; }
  // The cells that cover the range - kMapCells, fewer only for a range under
  // kMapCells * 16 bytes - and the bytes of each but the last, which has what
  // is left: a multiple of 16, the range / kMapCells rounded up. 0 with no
  // range.
  unsigned mapCells() const { return cells_; }
  size_t cellBytes() const { return cellBytes_; }
  // The bytes of used blocks (headers included) in cell `c`: exact. 0 with no
  // map or past mapCells().
  size_t cellUsed(unsigned c) const;
  // The used fraction of every cell into `out`: 0 is empty, 255 full, a cell
  // with any used byte in it reads at least 1 and only a full one 255. Cells
  // past mapCells() - every cell, with no map - read 0. Linear in kMapCells.
  void occupancy(uint8_t out[kMapCells]) const;
  // The same in two halves, for a caller that holds a lock only for the copy:
  // copyMap takes the map as it is (zeros with none), occupancy reads a copy.
  void copyMap(Map& out) const;
  void occupancy(const Map& m, uint8_t out[kMapCells]) const;

  // Visits every block in address order, free ones included. The heap must not
  // be changed from inside f.
  template <class F>
  void forEachBlock(F f) const {
    BlockInfo info;
    size_t offset = 0;
    while (blockAt(offset, info)) f(info);
  }

  // Walks every block and every free list and compares them with the totals,
  // the bitmaps, the free-block count and the occupancy map. True when the heap
  // is consistent. For verifiers and for a game that wants to check itself;
  // linear in the number of blocks.
  bool validate() const;

 private:
  struct Block;
  struct FreeLinks;

  static constexpr unsigned kRows = 27;    // first level: size classes by power of two
  static constexpr unsigned kColumns = 8;  // second level: lists per power of two

  struct Index {
    unsigned row;
    unsigned column;
  };

  // Reads the block that starts `offset` bytes into the range into `out` and
  // moves `offset` to the next one. False at the end of the range.
  bool blockAt(size_t& offset, BlockInfo& out) const;

  Block* blockOf(const void* payload) const;
  Block* nextOf(const Block* b) const;
  Block* prevOf(const Block* b) const;

  static Index mapInsert(size_t blockBytes);
  static Index mapSearch(size_t blockBytes);
  void insertFree(Block* b);
  void removeFree(Block* b);
  Block* findFree(size_t blockBytes) const;

  // The occupancy map: the cell an offset into the range falls in, a cell's
  // bytes, and a block's bytes added to (or taken from) the cells it spans.
  unsigned cellOf(size_t offset) const;
  size_t cellSize(unsigned c) const;
  void mapSpan(const void* blockStart, size_t bytes, bool add);
  void clearMap();

  unsigned char* base_ = nullptr;
  size_t capacity_ = 0;
  size_t used_ = 0;
  size_t peak_ = 0;
  uint32_t freeBlocks_ = 0;
  uint32_t rowMap_ = 0;                 // bit r: row r has a non-empty list
  uint8_t columnMap_[kRows] = {};       // bit c of [r]: list (r, c) is not empty
  Block* lists_[kRows][kColumns] = {};  // heads of the free lists
  Map* map_ = nullptr;
  unsigned cells_ = 0;      // cells that cover the range
  size_t cellBytes_ = 0;    // bytes of a cell (the last may have fewer)
  uint32_t cellRecip_ = 0;  // 2^32 / cellBytes_, rounded down: cellOf's divide
};

}  // namespace Memory
}  // namespace Tyra
