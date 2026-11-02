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

// Added by TyraX: the region heap (inc/memory/region_heap.hpp). Portable C++ -
// no PS2 header, no assumption about the size of a pointer.
//
// Layout. The range is a chain of blocks, each starting with a 16-byte header:
//
//   u32 sizeAndFlags   whole block size (a multiple of 16) | bit 0 free | bit 1 over-aligned
//   u32 prevSize       size of the block before (0 for the first)
//   u16 budget         who is charged
//   u16 spare
//   u32 owner          who asked
//
// The payload follows the header, so it is 16-aligned. A free block keeps its
// free-list links in its payload. The previous block's size is always valid, so
// both neighbours of any block are one subtraction or addition away.
//
// Size classes. A block of u = size / 16 units falls in row r (u below 8: row 0,
// column u; otherwise r = floor(log2 u) - 2 and the column is the next three
// bits of u under the leading one). A search for n bytes starts at the first
// class whose smallest block is at least n, so the head of any non-empty list
// from there on fits - two bit-scans find it. If that finds nothing, the one
// class n itself falls in is walked: it may hold a block just big enough.
//
// The free-block count follows insertFree / removeFree: every free block is in
// exactly one list. The occupancy map follows every change of used_: a block's
// bytes go into the cell it starts in and the cell it ends in, and the cells in
// between are marked whole in `full` - a difference array, summed when the map
// is read - so a block costs the same whatever it spans.

#include "memory/region_heap.hpp"

#include <cstring>

#include "memory/failure.hpp"

namespace Tyra {
namespace Memory {
namespace {

constexpr size_t kMinBlock = 2 * RegionHeap::kHeader;  // header + room for the links
constexpr size_t kMaxBlock = 0xFFFFFFF0u;              // sizes are 32 bits wide
constexpr uint32_t kFreeFlag = 1;
constexpr uint32_t kOverAlignedFlag = 2;
constexpr uint32_t kFlagMask = 0xF;

inline uintptr_t addressOf(const void* p) { return reinterpret_cast<uintptr_t>(p); }

// The address the public function returns to - the code that called the heap.
// A macro, so the builtin runs in that function and not in a helper.
#define TYRA_HEAP_CALLER() reinterpret_cast<uintptr_t>(__builtin_return_address(0))

}  // namespace

struct RegionHeap::Block {
  uint32_t sizeAndFlags;
  uint32_t prevSize;
  uint16_t budget;
  uint16_t spare;
  uint32_t owner;

  size_t size() const { return sizeAndFlags & ~kFlagMask; }
  bool isFree() const { return (sizeAndFlags & kFreeFlag) != 0; }
  bool isOverAligned() const { return (sizeAndFlags & kOverAlignedFlag) != 0; }
  void set(size_t size, uint32_t flags) {
    sizeAndFlags = static_cast<uint32_t>(size) | flags;
  }
  unsigned char* bytes() { return reinterpret_cast<unsigned char*>(this); }
  const unsigned char* bytes() const {
    return reinterpret_cast<const unsigned char*>(this);
  }
  void* payload() { return bytes() + RegionHeap::kHeader; }
  FreeLinks* links() const;
};

struct RegionHeap::FreeLinks {
  Block* next;
  Block* prev;
};

RegionHeap::FreeLinks* RegionHeap::Block::links() const {
  return reinterpret_cast<FreeLinks*>(const_cast<unsigned char*>(bytes()) +
                                      RegionHeap::kHeader);
}

// ---- size classes ---------------------------------------------------------

RegionHeap::Index RegionHeap::mapInsert(size_t blockBytes) {
  const uint64_t units = static_cast<uint64_t>(blockBytes) >> 4;
  if (units < 8) return Index{0, static_cast<unsigned>(units)};
  const unsigned top = 63 - static_cast<unsigned>(__builtin_clzll(units));
  return Index{top - 2, static_cast<unsigned>((units >> (top - 3)) & 7)};
}

RegionHeap::Index RegionHeap::mapSearch(size_t blockBytes) {
  uint64_t units = static_cast<uint64_t>(blockBytes) >> 4;
  if (units >= 8) {
    // Round up to the next class boundary: every block in that class, and in
    // every class above it, is at least blockBytes.
    const unsigned top = 63 - static_cast<unsigned>(__builtin_clzll(units));
    units += (uint64_t(1) << (top - 3)) - 1;
  }
  return mapInsert(static_cast<size_t>(units << 4));
}

void RegionHeap::insertFree(Block* b) {
  const Index at = mapInsert(b->size());
  FreeLinks* links = b->links();
  links->prev = nullptr;
  links->next = lists_[at.row][at.column];
  if (links->next != nullptr) links->next->links()->prev = b;
  lists_[at.row][at.column] = b;
  columnMap_[at.row] = static_cast<uint8_t>(columnMap_[at.row] | (1u << at.column));
  rowMap_ |= 1u << at.row;
  ++freeBlocks_;
}

void RegionHeap::removeFree(Block* b) {
  const Index at = mapInsert(b->size());
  FreeLinks* links = b->links();
  if (links->prev != nullptr) {
    links->prev->links()->next = links->next;
  } else {
    lists_[at.row][at.column] = links->next;
  }
  if (links->next != nullptr) links->next->links()->prev = links->prev;
  if (lists_[at.row][at.column] == nullptr) {
    columnMap_[at.row] =
        static_cast<uint8_t>(columnMap_[at.row] & ~(1u << at.column));
    if (columnMap_[at.row] == 0) rowMap_ &= ~(1u << at.row);
  }
  --freeBlocks_;
}

RegionHeap::Block* RegionHeap::findFree(size_t blockBytes) const {
  if (blockBytes > capacity_) return nullptr;

  const Index from = mapSearch(blockBytes);
  if (from.row < kRows) {
    const uint32_t columns = columnMap_[from.row] & (0xFFu << from.column);
    if (columns != 0) {
      return lists_[from.row][__builtin_ctz(columns)];
    }
    const uint32_t rows = rowMap_ & (~0u << (from.row + 1));
    if (rows != 0) {
      const unsigned row = static_cast<unsigned>(__builtin_ctz(rows));
      return lists_[row][__builtin_ctz(columnMap_[row])];
    }
  }

  // Nothing in a class that is certain to fit: the class blockBytes itself falls
  // in may still hold a block that is big enough.
  const Index own = mapInsert(blockBytes);
  for (Block* b = lists_[own.row][own.column]; b != nullptr; b = b->links()->next) {
    if (b->size() >= blockBytes) return b;
  }
  return nullptr;
}

// ---- neighbours -----------------------------------------------------------

RegionHeap::Block* RegionHeap::blockOf(const void* payload) const {
  return reinterpret_cast<Block*>(const_cast<unsigned char*>(
      static_cast<const unsigned char*>(payload)) - kHeader);
}

RegionHeap::Block* RegionHeap::nextOf(const Block* b) const {
  const unsigned char* next = b->bytes() + b->size();
  if (next >= base_ + capacity_) return nullptr;
  return reinterpret_cast<Block*>(const_cast<unsigned char*>(next));
}

RegionHeap::Block* RegionHeap::prevOf(const Block* b) const {
  if (b->prevSize == 0) return nullptr;
  return reinterpret_cast<Block*>(const_cast<unsigned char*>(b->bytes()) - b->prevSize);
}

// ---- the occupancy map ----------------------------------------------------

// offset / cellBytes_ without a divide: the reciprocal's estimate is the
// quotient or one short of it (offsets are below 2^32), and one test settles it.
unsigned RegionHeap::cellOf(size_t offset) const {
  const uint32_t o = static_cast<uint32_t>(offset);
  const uint32_t bytes = static_cast<uint32_t>(cellBytes_);
  uint32_t q = static_cast<uint32_t>((static_cast<uint64_t>(o) * cellRecip_) >> 32);
  if (o - q * bytes >= bytes) ++q;
  return q;
}

size_t RegionHeap::cellSize(unsigned c) const {
  return c + 1 < cells_ ? cellBytes_ : capacity_ - size_t(cells_ - 1) * cellBytes_;
}

void RegionHeap::mapSpan(const void* blockStart, size_t bytes, bool add) {
  Map* m = map_;
  if (m == nullptr || bytes == 0) return;
  const size_t offset = static_cast<size_t>(addressOf(blockStart) - addressOf(base_));
  const unsigned first = cellOf(offset);
  const unsigned last = cellOf(offset + bytes - 1);
  // Unsigned: a take that runs ahead of its add (slideDown) wraps and comes back.
  const auto put = [m, add](unsigned c, size_t n) {
    const uint32_t v = static_cast<uint32_t>(n);
    m->part[c] = add ? m->part[c] + v : m->part[c] - v;
  };
  if (first == last) {
    put(first, bytes);
    return;
  }
  put(first, size_t(first + 1) * cellBytes_ - offset);
  put(last, offset + bytes - size_t(last) * cellBytes_);
  if (last > first + 1) {
    const int8_t d = add ? 1 : -1;
    m->full[first + 1] = static_cast<int8_t>(m->full[first + 1] + d);
    m->full[last] = static_cast<int8_t>(m->full[last] - d);
  }
}

void RegionHeap::clearMap() {
  if (map_ != nullptr) std::memset(map_, 0, sizeof *map_);
}

void RegionHeap::attachMap(Map* map) {
  map_ = map;
  clearMap();
  if (map_ == nullptr || capacity_ == 0) return;
  for (size_t offset = 0; offset < capacity_;) {
    const Block* b = reinterpret_cast<const Block*>(base_ + offset);
    if (b->size() < kMinBlock) break;  // a corrupt chain must not loop forever
    if (!b->isFree()) mapSpan(b, b->size(), true);
    offset += b->size();
  }
}

size_t RegionHeap::cellUsed(unsigned c) const {
  if (map_ == nullptr || c >= cells_) return 0;
  int run = 0;
  for (unsigned i = 0; i <= c; ++i) run += map_->full[i];
  return map_->part[c] + (run > 0 ? cellSize(c) : 0);
}

void RegionHeap::copyMap(Map& out) const {
  if (map_ != nullptr) {
    out = *map_;
  } else {
    std::memset(&out, 0, sizeof out);
  }
}

void RegionHeap::occupancy(uint8_t out[kMapCells]) const {
  if (map_ == nullptr) {
    std::memset(out, 0, kMapCells);
    return;
  }
  occupancy(*map_, out);
}

void RegionHeap::occupancy(const Map& m, uint8_t out[kMapCells]) const {
  int run = 0;
  for (unsigned c = 0; c < kMapCells; ++c) {
    if (c >= cells_) {
      out[c] = 0;
      continue;
    }
    run += m.full[c];
    size_t size = cellSize(c);
    size_t used = run > 0 ? size : m.part[c];
    if (used == 0) {
      out[c] = 0;
    } else if (used >= size) {
      out[c] = 255;
    } else {
      // A 32-bit divide: shift both until size * 255 fits.
      while (size >= (size_t(1) << 24)) {
        size >>= 1;
        used >>= 1;
      }
      const uint32_t f = static_cast<uint32_t>(used) * 255u / static_cast<uint32_t>(size);
      out[c] = static_cast<uint8_t>(f < 1 ? 1 : f > 254 ? 254 : f);
    }
  }
}

// ---- the heap -------------------------------------------------------------

void RegionHeap::init(void* base, size_t bytes) {
  static_assert(sizeof(Block) == kHeader, "the block header is 16 bytes");
  static_assert(sizeof(FreeLinks) <= kMinBlock - kHeader,
                "the smallest block has room for its free-list links");

  unsigned char* start = static_cast<unsigned char*>(base);
  size_t usableBytes = bytes;

  // Take what the contract promises; an unchecked build quietly settles for the
  // 16-aligned part of the range that fits in the 32-bit sizes.
  const size_t skew = (kMinAlign - (addressOf(start) & (kMinAlign - 1))) & (kMinAlign - 1);
  if (skew != 0) {
    start += skew;
    usableBytes = usableBytes > skew ? usableBytes - skew : 0;
  }
  if (start == nullptr) usableBytes = 0;
  usableBytes &= ~(kMinAlign - 1);
  if (usableBytes > kMaxBlock) usableBytes = kMaxBlock;
  if (usableBytes < kMinBlock) usableBytes = 0;
#if TYRA_MEMORY_CHECKS
  if (start == nullptr || usableBytes != bytes) {
    fail(Failure{"region heap bad range", nullptr, bytes, TYRA_HEAP_CALLER()});
  }
#endif

  base_ = start;
  capacity_ = usableBytes;
  used_ = 0;
  peak_ = 0;
  freeBlocks_ = 0;
  rowMap_ = 0;
  std::memset(columnMap_, 0, sizeof columnMap_);
  std::memset(lists_, 0, sizeof lists_);
  clearMap();
  cells_ = 0;
  cellBytes_ = 0;
  cellRecip_ = 0;
  if (capacity_ == 0) return;

  // The map's cells: the range / kMapCells rounded up to 16, so every block
  // edge falls on a cell's 16-byte grid; a small range has fewer cells.
  cellBytes_ = ((capacity_ + kMapCells - 1) / kMapCells + kMinAlign - 1) & ~(kMinAlign - 1);
  cells_ = static_cast<unsigned>((capacity_ + cellBytes_ - 1) / cellBytes_);
  cellRecip_ = static_cast<uint32_t>((uint64_t(1) << 32) / cellBytes_);

  Block* all = reinterpret_cast<Block*>(base_);
  all->set(capacity_, kFreeFlag);
  all->prevSize = 0;
  all->budget = 0;
  all->spare = 0;
  all->owner = 0;
  insertFree(all);
}

void* RegionHeap::allocate(size_t size, size_t align, uint16_t budget, uint32_t owner) {
  if (align < kMinAlign) align = kMinAlign;
  if ((align & (align - 1)) != 0) {
#if TYRA_MEMORY_CHECKS
    fail(Failure{"region heap alignment is not a power of two", nullptr, size,
                 TYRA_HEAP_CALLER()});
#endif
    return nullptr;
  }
  if (capacity_ < kMinBlock || size > capacity_ - kHeader) return nullptr;

  // Whole block: header, payload rounded to 16, at least the smallest block.
  size_t need = kHeader + ((size + kMinAlign - 1) & ~(kMinAlign - 1));
  if (need < kMinBlock) need = kMinBlock;

  // An over-aligned payload needs room to move up to its alignment, and when the
  // heap's first block cannot absorb a gap of one header, one alignment more.
  size_t search = need;
  if (align > kMinAlign) {
    if (align >= capacity_ || need > capacity_ - align - kHeader) return nullptr;
    search = need + align + kHeader;
  }

  Block* b = findFree(search);
  if (b == nullptr) return nullptr;
  removeFree(b);

  const size_t total = b->size();
  size_t gap = 0;
  if (align > kMinAlign) {
    const uintptr_t first = addressOf(b->bytes()) + kHeader;
    const uintptr_t aligned = (first + align - 1) & ~(uintptr_t(align) - 1);
    gap = static_cast<size_t>(aligned - first);
    // One header is too little for a block of its own: it joins the block in
    // front - and the first block has none, so it moves on to the next boundary.
    if (gap == kHeader && b->prevSize == 0) gap += align;
  }

  Block* a = b;
  if (gap != 0) {
    a = reinterpret_cast<Block*>(b->bytes() + gap);
    if (gap == kHeader) {
      Block* before = prevOf(b);
      before->set(before->size() + gap, before->sizeAndFlags & kFlagMask);
      used_ += gap;
      mapSpan(b, gap, true);
      a->prevSize = static_cast<uint32_t>(before->size());
    } else {
      b->set(gap, kFreeFlag);
      insertFree(b);
      a->prevSize = static_cast<uint32_t>(gap);
    }
  }

  // Keep what is asked for; a remainder big enough to be a block goes back.
  const size_t room = total - gap;
  const size_t take = (room - need >= kMinBlock) ? need : room;
  a->set(take, align > kMinAlign ? kOverAlignedFlag : 0);
  a->budget = budget;
  a->spare = 0;
  a->owner = owner;
  if (take < room) {
    Block* rest = reinterpret_cast<Block*>(a->bytes() + take);
    rest->set(room - take, kFreeFlag);
    rest->prevSize = static_cast<uint32_t>(take);
    rest->budget = 0;
    rest->spare = 0;
    rest->owner = 0;
    Block* after = nextOf(rest);
    if (after != nullptr) after->prevSize = static_cast<uint32_t>(rest->size());
    insertFree(rest);
  } else {
    Block* after = nextOf(a);
    if (after != nullptr) after->prevSize = static_cast<uint32_t>(take);
  }

  used_ += take;
  if (used_ > peak_) peak_ = used_;
  mapSpan(a, take, true);
  return a->payload();
}

bool RegionHeap::owns(const void* p) const {
  const uintptr_t at = addressOf(p), lo = addressOf(base_), hi = lo + capacity_;
  if (at < lo + kHeader || at >= hi || ((at - lo) & (kMinAlign - 1)) != 0) return false;

  const Block* b = blockOf(p);
  const size_t size = b->size();
  if (b->isFree() || size < kMinBlock || (size & (kMinAlign - 1)) != 0) return false;
  if (size > hi - addressOf(b)) return false;
  const Block* after = nextOf(b);
  if (after != nullptr && after->prevSize != size) return false;
  if (b->prevSize != 0) {
    if (b->prevSize > addressOf(b) - lo) return false;
    const Block* before = prevOf(b);
    if (before->size() != b->prevSize) return false;
  }
  return true;
}

void RegionHeap::release(void* p) {
  if (p == nullptr) return;
#if TYRA_MEMORY_CHECKS
  if (!owns(p)) {
    fail(Failure{"region heap invalid release", nullptr, 0, TYRA_HEAP_CALLER()});
    return;
  }
#endif

  Block* b = blockOf(p);
  size_t size = b->size();
  used_ -= size;
  mapSpan(b, size, false);

  Block* next = nextOf(b);
  if (next != nullptr && next->isFree()) {
    removeFree(next);
    size += next->size();
#if TYRA_MEMORY_CHECKS
    next->sizeAndFlags = kFreeFlag;  // a merged-away header is not a block any more
#endif
  }
  Block* before = prevOf(b);
  if (before != nullptr && before->isFree()) {
    removeFree(before);
    size += before->size();
#if TYRA_MEMORY_CHECKS
    b->sizeAndFlags = kFreeFlag;
#endif
    b = before;
  }

  b->set(size, kFreeFlag);
  b->budget = 0;
  b->spare = 0;
  b->owner = 0;
  Block* after = nextOf(b);
  if (after != nullptr) after->prevSize = static_cast<uint32_t>(size);
  insertFree(b);
}

void* RegionHeap::slideDown(void* p) {
  if (p == nullptr) return nullptr;
  Block* moving = blockOf(p);
  if (moving->isFree() || moving->isOverAligned()) return nullptr;
  Block* hole = prevOf(moving);
  if (hole == nullptr || !hole->isFree()) return nullptr;

  const size_t movingSize = moving->size();
  const size_t holeSize = hole->size();
  const uint32_t holePrev = hole->prevSize;
  Block* next = nextOf(moving);
  removeFree(hole);
  mapSpan(moving, movingSize, false);
  mapSpan(hole, movingSize, true);

  // The block goes to where the hole began; the hole is what is left behind it.
  std::memmove(hole->bytes(), moving->bytes(), movingSize);
  hole->prevSize = holePrev;

  Block* left = reinterpret_cast<Block*>(hole->bytes() + movingSize);
  size_t leftSize = holeSize;
  left->set(leftSize, kFreeFlag);
  left->prevSize = static_cast<uint32_t>(movingSize);
  left->budget = 0;
  left->spare = 0;
  left->owner = 0;
  if (next != nullptr && next->isFree()) {
    removeFree(next);
    leftSize += next->size();
    left->set(leftSize, kFreeFlag);
    next = nextOf(left);
  }
  if (next != nullptr) next->prevSize = static_cast<uint32_t>(leftSize);
  insertFree(left);
  return hole->payload();
}

size_t RegionHeap::usable(const void* p) const { return blockOf(p)->size() - kHeader; }

uint16_t RegionHeap::budgetOf(const void* p) const { return blockOf(p)->budget; }

uint32_t RegionHeap::ownerOf(const void* p) const { return blockOf(p)->owner; }

void RegionHeap::setOwner(void* p, uint32_t owner) { blockOf(p)->owner = owner; }

bool RegionHeap::contains(const void* p) const {
  const uintptr_t at = addressOf(p), lo = addressOf(base_);
  return at >= lo && at - lo < capacity_;
}

bool RegionHeap::slidable(const void* p) const { return !blockOf(p)->isOverAligned(); }

size_t RegionHeap::largestFree() const {
  for (unsigned row = kRows; row-- > 0;) {
    if (columnMap_[row] == 0) continue;
    const unsigned column = 31 - static_cast<unsigned>(__builtin_clz(columnMap_[row]));
    size_t largest = 0;
    for (const Block* b = lists_[row][column]; b != nullptr; b = b->links()->next) {
      if (b->size() > largest) largest = b->size();
    }
    return largest - kHeader;
  }
  return 0;
}

bool RegionHeap::blockAt(size_t& offset, BlockInfo& out) const {
  if (offset >= capacity_) return false;
  Block* b = reinterpret_cast<Block*>(base_ + offset);
  if (b->size() < kMinBlock) return false;  // a corrupt chain must not loop forever
  out.payload = b->payload();
  out.bytes = b->size() - kHeader;
  out.free = b->isFree();
  out.budget = b->budget;
  out.owner = b->owner;
  offset += b->size();
  return true;
}

bool RegionHeap::validate() const {
  // The chain of blocks.
  size_t offset = 0, before = 0, usedSum = 0, freeBlocks = 0;
  bool beforeFree = false;
  while (offset < capacity_) {
    const Block* b = reinterpret_cast<const Block*>(base_ + offset);
    const size_t size = b->size();
    if (size < kMinBlock || (size & (kMinAlign - 1)) != 0 || size > capacity_ - offset) {
      return false;
    }
    if (b->prevSize != before) return false;
    if (b->isFree()) {
      if (beforeFree) return false;  // two free neighbours would have been merged
      ++freeBlocks;
    } else {
      usedSum += size;
    }
    beforeFree = b->isFree();
    before = size;
    offset += size;
  }
  if (offset != capacity_ || usedSum != used_ || used_ > peak_) return false;
  if (freeBlocks != freeBlocks_) return false;

  // The occupancy map: every cell's used bytes, from the chain.
  if (map_ != nullptr) {
    size_t cellSums[kMapCells] = {};
    for (size_t at = 0; at < capacity_;) {
      const Block* b = reinterpret_cast<const Block*>(base_ + at);
      if (!b->isFree()) {
        for (size_t from = at, end = at + b->size(); from < end;) {
          const unsigned c = cellOf(from);
          const size_t cellEnd = size_t(c + 1) * cellBytes_;
          const size_t upTo = end < cellEnd ? end : cellEnd;
          cellSums[c] += upTo - from;
          from = upTo;
        }
      }
      at += b->size();
    }
    for (unsigned c = 0; c < cells_; ++c) {
      if (cellSums[c] != cellUsed(c)) return false;
    }
  }

  // The free lists: every free block once, in the class its size says.
  size_t listed = 0;
  for (unsigned row = 0; row < kRows; ++row) {
    if (((rowMap_ >> row) & 1u) != (columnMap_[row] != 0 ? 1u : 0u)) return false;
    for (unsigned column = 0; column < kColumns; ++column) {
      const Block* head = lists_[row][column];
      if (((columnMap_[row] >> column) & 1u) != (head != nullptr ? 1u : 0u)) return false;
      const Block* last = nullptr;
      for (const Block* b = head; b != nullptr; b = b->links()->next) {
        if (!contains(b) || !b->isFree()) return false;
        const Index at = mapInsert(b->size());
        if (at.row != row || at.column != column) return false;
        if (b->links()->prev != last) return false;
        last = b;
        if (++listed > freeBlocks) return false;
      }
    }
  }
  return listed == freeBlocks;
}

}  // namespace Memory
}  // namespace Tyra
