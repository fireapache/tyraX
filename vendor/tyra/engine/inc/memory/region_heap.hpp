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
// segregated-fit heap (TLSF). Allocation and release take constant time: the
// free blocks sit in lists by size class, two bitmaps say which lists are not
// empty, and a released block is merged with its free neighbours at once through
// the sizes every block header keeps of itself and of the block before it.
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

  // Visits every block in address order, free ones included. The heap must not
  // be changed from inside f.
  template <class F>
  void forEachBlock(F f) const {
    BlockInfo info;
    size_t offset = 0;
    while (blockAt(offset, info)) f(info);
  }

  // Walks every block and every free list and compares them with the totals and
  // the bitmaps. True when the heap is consistent. For verifiers and for a game
  // that wants to check itself; linear in the number of blocks.
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
  bool owns(const void* payload) const;

  static Index mapInsert(size_t blockBytes);
  static Index mapSearch(size_t blockBytes);
  void insertFree(Block* b);
  void removeFree(Block* b);
  Block* findFree(size_t blockBytes) const;

  unsigned char* base_ = nullptr;
  size_t capacity_ = 0;
  size_t used_ = 0;
  size_t peak_ = 0;
  uint32_t rowMap_ = 0;                 // bit r: row r has a non-empty list
  uint8_t columnMap_[kRows] = {};       // bit c of [r]: list (r, c) is not empty
  Block* lists_[kRows][kColumns] = {};  // heads of the free lists
};

}  // namespace Memory
}  // namespace Tyra
