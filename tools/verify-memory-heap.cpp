// Added by TyraX: host verifier for the region heap (inc/memory/region_heap.hpp,
// src/memory/region_heap.cpp). Built and run by tools/verify-memory.py, once with
// TYRA_MEMORY_CHECKS=0 and once with =1.

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <map>
#include <random>
#include <vector>

#include "memory/region_heap.hpp"
#include "verify-memory-check.hpp"

namespace {

alignas(16) unsigned char arena[1 << 20];
alignas(4096) unsigned char tiny[8192];

// ---- helpers over the heap's address-order walk -------------------------

// The payload bytes of the free block that starts at this payload address.
size_t freeBlockBytesAt(const RegionHeap& h, const void* payload) {
  size_t bytes = 0;
  h.forEachBlock([&](const RegionHeap::BlockInfo& b) {
    if (b.free && b.payload == payload) bytes = b.bytes;
  });
  return bytes;
}

size_t blockCount(const RegionHeap& h) {
  size_t n = 0;
  h.forEachBlock([&](const RegionHeap::BlockInfo&) { ++n; });
  return n;
}

// Sum of the sizes (header included) of the allocated or of the free blocks.
size_t walkedBytes(const RegionHeap& h, bool free) {
  size_t sum = 0;
  h.forEachBlock([&](const RegionHeap::BlockInfo& b) {
    if (b.free == free) sum += b.bytes + RegionHeap::kHeader;
  });
  return sum;
}

// Whether the block before the one at this payload is free.
bool prevIsFree(const RegionHeap& h, const void* payload) {
  bool result = false, last = false;
  h.forEachBlock([&](const RegionHeap::BlockInfo& b) {
    if (b.payload == payload) result = last;
    last = b.free;
  });
  return result;
}

// ---- the free-block count and the occupancy map, against the walk ---------

uint32_t walkedFreeBlocks(const RegionHeap& h) {
  uint32_t n = 0;
  h.forEachBlock([&](const RegionHeap::BlockInfo& b) {
    if (b.free) ++n;
  });
  return n;
}

// Each cell's used bytes, from the walk: a used block covers
// [payload - header, payload + bytes) of the range that starts at `base`.
std::vector<size_t> walkedCells(const RegionHeap& h, const void* base) {
  std::vector<size_t> cells(h.mapCells(), 0);
  const size_t cell = h.cellBytes();
  h.forEachBlock([&](const RegionHeap::BlockInfo& b) {
    if (b.free) return;
    size_t from = uintptr_t(b.payload) - RegionHeap::kHeader - uintptr_t(base);
    const size_t end = from + b.bytes + RegionHeap::kHeader;
    while (from < end) {
      const size_t c = from / cell;
      const size_t upTo = std::min(end, (c + 1) * cell);
      cells[c] += upTo - from;
      from = upTo;
    }
  });
  return cells;
}

// The fraction a cell of `size` bytes with `used` of them used must read.
uint8_t expectedFraction(size_t used, size_t size) {
  if (used == 0) return 0;
  if (used >= size) return 255;
  const uint64_t f = uint64_t(used) * 255u / size;
  return uint8_t(f < 1 ? 1 : f > 254 ? 254 : f);
}

// Whether the heap's free-block count and map say what the walk says: every
// cell's bytes exactly, and the fractions occupancy() reads from them (also
// through copyMap).
bool mapAgrees(const RegionHeap& h, const void* base) {
  if (walkedFreeBlocks(h) != h.freeBlocks()) return false;
  uint8_t occ[RegionHeap::kMapCells], fromCopy[RegionHeap::kMapCells];
  h.occupancy(occ);
  RegionHeap::Map copy;
  h.copyMap(copy);
  h.occupancy(copy, fromCopy);
  if (!h.hasMap()) {
    return allBytes(occ, sizeof occ, 0) && allBytes(fromCopy, sizeof fromCopy, 0);
  }
  const std::vector<size_t> cells = walkedCells(h, base);
  for (unsigned c = 0; c < RegionHeap::kMapCells; ++c) {
    if (c >= cells.size()) {
      if (occ[c] != 0 || fromCopy[c] != 0 || h.cellUsed(c) != 0) return false;
      continue;
    }
    const size_t size =
        c + 1 < cells.size() ? h.cellBytes() : h.capacity() - c * h.cellBytes();
    if (h.cellUsed(c) != cells[c]) return false;
    const uint8_t want = expectedFraction(cells[c], size);
    if (occ[c] != want || fromCopy[c] != want) return false;
  }
  return true;
}

// ---- the brief's checks ---------------------------------------------------

void testRoundTripAndExhaustion() {
  RegionHeap h;
  h.init(arena, sizeof arena);
  const size_t free0 = h.freeBytes(), largest0 = h.largestFree();
  CHECK(free0 == sizeof arena);
  CHECK(largest0 == sizeof arena - RegionHeap::kHeader);
  CHECK(h.used() == 0 && h.peak() == 0 && h.capacity() == sizeof arena);
  CHECK(h.validate());

  // alignment and round trip
  void* a = h.allocate(100, 64, 7, 0xA);
  CHECK(a != nullptr && uintptr_t(a) % 64 == 0);
  CHECK(h.usable(a) >= 100);
  void* b = h.allocate(5000, 128, 7, 0xB);
  CHECK(b != nullptr && uintptr_t(b) % 128 == 0);
  CHECK(h.budgetOf(a) == 7 && h.ownerOf(b) == 0xB);
  CHECK(h.ownerOf(a) == 0xA && h.budgetOf(b) == 7);
  CHECK(!h.slidable(a) && !h.slidable(b));
  CHECK(h.contains(a) && h.contains(b));
  CHECK(h.validate());
  h.release(a);
  h.release(b);
  CHECK(h.freeBytes() == free0 && h.largestFree() == largest0);
  CHECK(h.used() == 0);
  CHECK(h.validate());

  // exhaustion changes nothing
  CHECK(h.allocate(sizeof arena, 16, 0, 0) == nullptr);
  CHECK(h.freeBytes() == free0);
  CHECK(h.allocate(sizeof arena - RegionHeap::kHeader + 1, 16, 0, 0) == nullptr);
  CHECK(h.allocate(SIZE_MAX, 16, 0, 0) == nullptr);
  CHECK(h.allocate(SIZE_MAX - 8, 128, 0, 0) == nullptr);
  CHECK(h.allocate(size_t(1) << 33, 16, 0, 0) == nullptr);
  CHECK(h.freeBytes() == free0 && h.largestFree() == largest0 && h.validate());

  // the whole region in one block, and nothing left
  void* all = h.allocate(sizeof arena - RegionHeap::kHeader, 16, 0, 0);
  CHECK(all != nullptr);
  CHECK(h.freeBytes() == 0 && h.largestFree() == 0 && h.used() == sizeof arena);
  CHECK(h.allocate(1, 16, 0, 0) == nullptr);
  h.release(all);
  CHECK(h.freeBytes() == free0 && h.largestFree() == largest0 && h.validate());

  // releasing nullptr is a no-op
  h.release(nullptr);
  CHECK(h.freeBytes() == free0 && h.validate());
}

void testCoalescing() {
  RegionHeap h;
  h.init(arena, sizeof arena);
  const size_t free0 = h.freeBytes(), largest0 = h.largestFree();
  void* x = h.allocate(1000, 16, 0, 0);
  void* y = h.allocate(1000, 16, 0, 0);
  void* z = h.allocate(1000, 16, 0, 0);
  CHECK(x && y && z);
  h.release(y);
  h.release(x);
  CHECK(freeBlockBytesAt(h, x) >= 2000);  // x and y are one free block now
  CHECK(h.validate());
  h.release(z);
  CHECK(blockCount(h) == 1 && h.freeBytes() == free0 && h.largestFree() == largest0);

  // every release order ends as one block
  void* p[6];
  for (int i = 0; i < 6; ++i) p[i] = h.allocate(200 + 50 * i, 16, 0, 0);
  const int order[6] = {3, 0, 5, 1, 4, 2};
  for (int i = 0; i < 6; ++i) {
    h.release(p[order[i]]);
    CHECK(h.validate());
  }
  CHECK(blockCount(h) == 1 && h.freeBytes() == free0 && h.largestFree() == largest0);
}

void testSlide() {
  RegionHeap h;
  h.init(arena, sizeof arena);
  const size_t free0 = h.freeBytes();

  // B moves into A's hole, its bytes intact
  void* p = h.allocate(256, 16, 0, 0);
  void* q = h.allocate(256, 16, 3, 1);
  void* r = h.allocate(256, 16, 0, 2);
  std::memset(q, 0x5A, 256);
  CHECK(h.slidable(q) && h.slidable(p));
  CHECK(h.slideDown(q) == nullptr);  // the block before q is allocated
  h.release(p);
  const size_t usedBefore = h.used(), freeBefore = h.freeBytes();
  void* q2 = h.slideDown(q);
  CHECK(q2 == p);
  CHECK(allBytes(q2, 256, 0x5A));
  CHECK(h.budgetOf(q2) == 3 && h.ownerOf(q2) == 1);
  CHECK(h.used() == usedBefore && h.freeBytes() == freeBefore);  // moved, not changed
  CHECK(h.validate());
  // the hole is now behind it, merged with the free block that followed
  CHECK(freeBlockBytesAt(h, static_cast<unsigned char*>(q2) + h.usable(q2) +
                                RegionHeap::kHeader) > 0);
  CHECK(h.slideDown(q2) == nullptr);  // at the start: nothing before it
  // r can slide too: the hole moved to its front
  void* r2 = h.slideDown(r);
  CHECK(r2 != nullptr && r2 < r);
  CHECK(h.slideDown(r2) == nullptr);
  h.release(q2);
  h.release(r2);
  CHECK(h.freeBytes() == free0 && blockCount(h) == 1 && h.validate());

  // an over-aligned block never slides
  void* a = h.allocate(64, 16, 0, 0);
  void* o = h.allocate(64, 128, 0, 0);
  h.release(a);
  CHECK(!h.slidable(o) && h.slideDown(o) == nullptr);
  h.release(o);
  CHECK(h.freeBytes() == free0 && h.validate());
}

void testAccounting() {
  RegionHeap h;
  h.init(arena, sizeof arena);
  void* a = h.allocate(100, 16, 0, 0);
  const size_t usedA = h.used();
  CHECK(usedA >= 100 + RegionHeap::kHeader && usedA == h.usable(a) + RegionHeap::kHeader);
  CHECK(h.peak() == usedA);
  void* b = h.allocate(3000, 16, 0, 0);
  const size_t usedAB = h.used();
  CHECK(usedAB > usedA && h.peak() == usedAB);
  CHECK(h.used() + h.freeBytes() == sizeof arena);
  h.release(b);
  CHECK(h.used() == usedA && h.peak() == usedAB);  // the peak stays
  h.release(a);
  CHECK(h.used() == 0 && h.peak() == usedAB);
  CHECK(walkedBytes(h, false) == 0 && walkedBytes(h, true) == sizeof arena);

  // owner and budget live in the block header
  void* c = h.allocate(48, 16, 9, 77);
  CHECK(h.budgetOf(c) == 9 && h.ownerOf(c) == 77);
  h.setOwner(c, 123456);
  CHECK(h.ownerOf(c) == 123456 && h.budgetOf(c) == 9);
  h.forEachBlock([&](const RegionHeap::BlockInfo& i) {
    if (i.payload == c) CHECK(!i.free && i.owner == 123456 && i.budget == 9);
  });
  h.release(c);

  // contains is a range test
  CHECK(h.contains(arena) && h.contains(arena + sizeof arena - 1));
  CHECK(!h.contains(arena + sizeof arena) && !h.contains(nullptr));
  int outside = 0;
  CHECK(!h.contains(&outside));

  // zero-size and tiny sizes give distinct, usable blocks
  void* z0 = h.allocate(0, 16, 0, 0);
  void* z1 = h.allocate(1, 16, 0, 0);
  CHECK(z0 && z1 && z0 != z1 && uintptr_t(z0) % 16 == 0 && uintptr_t(z1) % 16 == 0);
  CHECK(h.usable(z1) >= 1);
  h.release(z0);
  h.release(z1);
  CHECK(h.used() == 0 && h.validate());
}

void testOwns() {
  RegionHeap h;
  h.init(arena, sizeof arena);
  unsigned char* a = static_cast<unsigned char*>(h.allocate(256, 16, 1, 2));
  unsigned char* b = static_cast<unsigned char*>(h.allocate(256, 64, 1, 3));
  CHECK(a != nullptr && b != nullptr);

  // true for a live payload, over-aligned or not
  CHECK(h.owns(a) && h.owns(b));

  // false outside the region
  int outside = 0;
  CHECK(!h.owns(nullptr) && !h.owns(&outside));
  CHECK(!h.owns(reinterpret_cast<void*>(uintptr_t(arena) - RegionHeap::kHeader)));
  CHECK(!h.owns(arena + sizeof arena));

  // false for the start of the region (the first header, not a payload)
  CHECK(!h.owns(arena));

  // false for a pointer into the middle of a payload, whatever it holds
  const unsigned char fills[] = {0x00, 0x5A, 0xFF};
  for (unsigned char fill : fills) {
    std::memset(a, fill, 256);
    CHECK(!h.owns(a + 8));    // not 16-aligned
    CHECK(!h.owns(a + 16));   // aligned, but its "header" is payload bytes
    CHECK(!h.owns(a + 64));
    CHECK(!h.owns(a + 240));
  }
  CHECK(h.owns(a) && h.owns(b));  // the live ones did not notice

  // false once released (a is the first block and has no block in front to merge
  // into, so its header stays and says free)
  h.release(a);
  CHECK(!h.owns(a) && h.owns(b));
  h.release(b);
  CHECK(h.validate());
}

void testOverAligned() {
  RegionHeap h;

  // At the very start of a region there is no block before the gap to absorb it:
  // a payload that is 16 bytes short of its alignment is moved on instead.
  h.init(tiny, sizeof tiny);
  const size_t free0 = h.freeBytes();
  void* a = h.allocate(10, 32, 0, 0);
  CHECK(a != nullptr && uintptr_t(a) % 32 == 0 && h.usable(a) >= 10);
  CHECK(h.validate());
  h.release(a);
  CHECK(h.freeBytes() == free0 && blockCount(h) == 1 && h.validate());

  // A gap of at least two headers becomes a free block in front.
  void* b = h.allocate(10, 256, 0, 0);
  CHECK(b != nullptr && uintptr_t(b) % 256 == 0 && h.validate());
  CHECK(blockCount(h) >= 3);  // the gap, the block, the rest
  h.release(b);
  CHECK(h.freeBytes() == free0 && blockCount(h) == 1 && h.validate());

  // A gap of one header joins the allocated block before.
  void* first = h.allocate(1, 16, 0, 0);  // the smallest block: 32 bytes
  CHECK(first == tiny + RegionHeap::kHeader);
  const size_t firstUsable = h.usable(first);
  void* c = h.allocate(10, 32, 0, 0);
  CHECK(c != nullptr && uintptr_t(c) % 32 == 0 && h.validate());
  CHECK(h.usable(first) == firstUsable + RegionHeap::kHeader);
  CHECK(h.used() + h.freeBytes() == sizeof tiny);
  h.release(c);
  h.release(first);
  CHECK(h.freeBytes() == free0 && blockCount(h) == 1 && h.validate());

  // The large and awkward ones, with the contents checked.
  const size_t aligns[] = {32, 64, 128, 256, 512, 1024, 2048};
  void* held[7];
  for (int i = 0; i < 7; ++i) {
    held[i] = h.allocate(40 + i, aligns[i], 1, 100 + i);
    CHECK(held[i] != nullptr && uintptr_t(held[i]) % aligns[i] == 0);
    if (held[i]) {
      std::memset(held[i], 0x30 + i, 40 + i);
      CHECK(h.ownerOf(held[i]) == uint32_t(100 + i) && !h.slidable(held[i]));
    }
    CHECK(h.validate());
  }
  for (int i = 0; i < 7; ++i) {
    if (held[i]) {
      CHECK(allBytes(held[i], 40 + i, uint8_t(0x30 + i)));
      h.release(held[i]);
    }
  }
  CHECK(h.freeBytes() == free0 && blockCount(h) == 1 && h.validate());

  // The tightest regions: 64 bytes cannot hold a 32-byte block on a 32-byte
  // boundary behind its header, 80 can (a 48-byte gap, then the block).
  h.init(tiny, 64);
  CHECK(h.allocate(10, 32, 0, 0) == nullptr && h.validate() && h.freeBytes() == 64);
  h.init(tiny, 80);
  void* t = h.allocate(10, 32, 0, 0);
  CHECK(t != nullptr && uintptr_t(t) % 32 == 0 && h.validate());
  CHECK(h.used() == 32 && h.freeBytes() == 48);
  h.release(t);
  CHECK(h.freeBytes() == 80 && blockCount(h) == 1 && h.validate());

  // A free block at the very start that is one header short of fitting the
  // moved-on block is passed over, not used.
  h.init(tiny, sizeof tiny);
  void* hole = h.allocate(48, 16, 0, 0);  // a 64-byte block at the start
  void* keep = h.allocate(1, 16, 0, 0);
  h.release(hole);
  void* moved = h.allocate(10, 32, 0, 0);
  CHECK(moved != nullptr && uintptr_t(moved) % 32 == 0 && h.validate());
  CHECK(uintptr_t(moved) > uintptr_t(keep));
  h.release(moved);
  h.release(keep);
  CHECK(h.freeBytes() == sizeof tiny && blockCount(h) == 1 && h.validate());
}

// The segregated lists must not hide a block that is big enough: a request whose
// size is not at a size-class boundary still finds the one free block of its
// own class.
void testExactReuse() {
  RegionHeap h;
  h.init(arena, sizeof arena);
  const size_t free0 = h.freeBytes();
  std::vector<void*> blocks;
  while (void* p = h.allocate(1100, 16, 0, 0)) blocks.push_back(p);
  CHECK(blocks.size() > 800);
  CHECK(h.largestFree() < 1100);  // nothing is left that would take another
  CHECK(h.allocate(1100, 16, 0, 0) == nullptr);
  const size_t freeFull = h.freeBytes();

  for (size_t i : {size_t(0), blocks.size() / 2, blocks.size() - 1}) {
    void* hole = blocks[i];
    h.release(hole);
    CHECK(h.validate());
    void* again = h.allocate(1100, 16, 0, 0);
    CHECK(again == hole);
    CHECK(h.freeBytes() == freeFull);
    CHECK(h.allocate(1100, 16, 0, 0) == nullptr);
  }

  // alternate blocks released: nothing merges, every hole is reusable
  for (size_t i = 0; i < blocks.size(); i += 2) h.release(blocks[i]);
  CHECK(h.validate());
  size_t refilled = 0;
  while (h.allocate(1100, 16, 0, 0)) ++refilled;
  CHECK(refilled == (blocks.size() + 1) / 2);
  CHECK(h.validate());

  // everything back (the refilled holes returned the same addresses)
  std::vector<void*> live;
  h.forEachBlock([&](const RegionHeap::BlockInfo& b) {
    if (!b.free) live.push_back(b.payload);
  });
  CHECK(live.size() == blocks.size());
  for (void* p : live) h.release(p);
  CHECK(h.freeBytes() == free0 && blockCount(h) == 1 && h.validate());
}

void testChecks() {
  RegionHeap h;
  h.init(arena, sizeof arena);
  const size_t free0 = h.freeBytes();
#if TYRA_MEMORY_CHECKS
  installThrowingFailure();
  void* a = h.allocate(64, 16, 1, 2);
  CHECK(a != nullptr);
  h.release(a);
  CHECK(throwsFailure([&] { h.release(a); }, "invalid release"));  // twice
  int outside = 0;
  CHECK(throwsFailure([&] { h.release(&outside); }, "invalid release"));
  void* b = h.allocate(64, 16, 1, 2);
  CHECK(throwsFailure([&] { h.release(static_cast<unsigned char*>(b) + 8); },
                      "invalid release"));
  CHECK(throwsFailure([&] { h.allocate(64, 48, 0, 0); }, "alignment"));
  CHECK(!capturedFailureLines().empty());
  CHECK(capturedFailureLines()[0].compare(0, 22, "[MEM] FAIL region heap") == 0);
  CHECK(h.validate());
  h.release(b);
#else
  CHECK(h.allocate(64, 48, 0, 0) == nullptr);  // not a power of two: nothing fits
  CHECK(h.allocate(64, 0, 0, 0) != nullptr);   // 0 means the minimum alignment
#endif
  CHECK(h.validate());
  (void)free0;
}

// ---- the free-block count and the occupancy map ---------------------------

void testFreeBlocksAndMap() {
  RegionHeap::Map map;
  RegionHeap h;
  h.init(arena, sizeof arena);
  uint8_t occ[RegionHeap::kMapCells];
  CHECK(h.freeBlocks() == 1 && !h.hasMap());
  CHECK(h.mapCells() == 64 && h.cellBytes() == sizeof arena / 64);
  std::memset(occ, 7, sizeof occ);
  h.occupancy(occ);
  CHECK(allBytes(occ, sizeof occ, 0) && h.cellUsed(0) == 0);  // no map, no cells
  CHECK(mapAgrees(h, arena));

  // A map attached late is filled from the blocks already there.
  void* a = h.allocate(100, 16, 0, 0);  // a 128-byte block at the start
  CHECK(h.freeBlocks() == 1);
  h.attachMap(&map);
  CHECK(h.hasMap() && h.cellUsed(0) == 128 && mapAgrees(h, arena));

  // A block over several cells: its ends in the two cells it touches, the
  // cells between full.
  const size_t cell = h.cellBytes();
  void* b = h.allocate(3 * cell, 16, 0, 0);  // 3 cells + 16 bytes, from 128
  CHECK(b != nullptr);
  CHECK(h.cellUsed(0) == cell && h.cellUsed(1) == cell && h.cellUsed(2) == cell);
  CHECK(h.cellUsed(3) == 128 + 16 && h.cellUsed(4) == 0);
  h.occupancy(occ);
  CHECK(occ[0] == 255 && occ[1] == 255 && occ[2] == 255);
  CHECK(occ[3] == expectedFraction(144, cell) && occ[3] == 2 && occ[4] == 0);
  CHECK(mapAgrees(h, arena) && h.validate());

  // A hole in front: two free blocks, the first cell no longer full.
  h.release(a);
  CHECK(h.freeBlocks() == 2 && h.cellUsed(0) == cell - 128);
  h.occupancy(occ);
  CHECK(occ[0] == expectedFraction(cell - 128, cell) && occ[0] < 255);
  CHECK(mapAgrees(h, arena));

  // slideDown takes its bytes along.
  void* b2 = h.slideDown(b);
  CHECK(b2 == a && h.freeBlocks() == 1);
  CHECK(h.cellUsed(0) == cell && h.cellUsed(3) == 16 && mapAgrees(h, arena));

  // A tiny block reads at least 1; only a full cell reads 255.
  void* t = h.allocate(1, 16, 0, 0);  // 32 bytes, in cell 3
  h.occupancy(occ);
  CHECK(t != nullptr && h.cellUsed(3) == 48 && occ[3] == 1);

  // Over-aligned blocks, a 16-byte gap joining the block in front included.
  std::vector<void*> wide;
  for (size_t align : {32, 64, 128, 256}) {
    for (int i = 0; i < 6; ++i) {
      void* w = h.allocate(40 + 16 * i, align, 0, 0);
      CHECK(w != nullptr && uintptr_t(w) % align == 0);
      wide.push_back(w);
      CHECK(mapAgrees(h, arena));
    }
  }
  for (size_t i = 0; i < wide.size(); i += 2) h.release(wide[i]);
  CHECK(mapAgrees(h, arena) && h.validate());

  // init keeps the map and empties it.
  h.init(arena, sizeof arena);
  CHECK(h.hasMap() && h.cellUsed(0) == 0 && h.freeBlocks() == 1);
  CHECK(mapAgrees(h, arena));

  // A range that is no multiple of 64 cells: cells of the range / 64 rounded
  // up to 16, the last one has the rest.
  h.init(arena, 1000000);
  CHECK(h.cellBytes() == 15632 && h.mapCells() == 64);
  void* all = h.allocate(h.largestFree(), 16, 0, 0);
  h.occupancy(occ);
  CHECK(all != nullptr && allBytes(occ, sizeof occ, 255));
  CHECK(h.cellUsed(63) == 1000000 - 63 * 15632 && h.freeBlocks() == 0);
  CHECK(mapAgrees(h, arena));
  h.release(all);

  // Random allocations, slides and releases over that range: the count and
  // the map agree with the walk after every one.
  std::mt19937 rng(0xCE11u);
  std::vector<void*> live;
  bool agreed = true;
  for (int op = 0; op < 3000; ++op) {
    const unsigned dice = unsigned(rng() % 100);
    if (live.empty() || dice < 55) {
      const size_t size = 1 + rng() % (rng() % 4 == 0 ? 60000 : 3000);
      void* p = h.allocate(size, size_t(16) << (rng() % 3), 0, 0);
      if (p != nullptr) live.push_back(p);
    } else if (dice < 65) {
      void*& p = live[rng() % live.size()];
      void* moved = h.slideDown(p);
      if (moved != nullptr) p = moved;
    } else {
      const size_t i = rng() % live.size();
      h.release(live[i]);
      live[i] = live.back();
      live.pop_back();
    }
    agreed = agreed && mapAgrees(h, arena);
  }
  CHECK(agreed && h.validate());
  for (void* p : live) h.release(p);
  CHECK(h.freeBlocks() == 1 && mapAgrees(h, arena));

  // A small range has fewer cells, 16 bytes each.
  h.init(tiny, 512);
  CHECK(h.mapCells() == 32 && h.cellBytes() == 16);
  void* s = h.allocate(40, 16, 0, 0);  // a 64-byte block: four cells
  h.occupancy(occ);
  CHECK(s != nullptr && occ[0] == 255 && occ[3] == 255 && occ[4] == 0);
  CHECK(allBytes(occ + 32, 32, 0) && mapAgrees(h, tiny));
  h.release(s);

  // Taken away: no map, but still the count.
  h.attachMap(nullptr);
  void* u = h.allocate(40, 16, 0, 0);
  void* v = h.allocate(40, 16, 0, 0);
  h.release(u);
  CHECK(!h.hasMap() && h.cellUsed(0) == 0 && h.freeBlocks() == 2);
  CHECK(mapAgrees(h, tiny) && h.validate());
  h.release(v);
}

// ---- the stress run -------------------------------------------------------

struct Live {
  void* p;
  size_t size;
  size_t align;
  uint8_t fill;
  uint16_t budget;
  uint32_t owner;
};

void testStress() {
  RegionHeap::Map map;
  RegionHeap h;
  h.init(arena, sizeof arena);
  h.attachMap(&map);
  const size_t free0 = h.freeBytes(), largest0 = h.largestFree();
  std::mt19937 rng(0x5EED1234u);
  std::vector<Live> live;
  std::map<uintptr_t, size_t> spans;  // payload address -> usable bytes
  static const size_t aligns[3] = {16, 64, 128};
  size_t allocFails = 0, allocs = 0, slides = 0, slideRefused = 0, releases = 0;
  uint32_t serial = 0;

  auto insertSpan = [&](void* p) {
    const uintptr_t at = uintptr_t(p), end = at + h.usable(p);
    auto next = spans.lower_bound(at);
    CHECK(next == spans.end() || next->first >= end);  // no overlap after
    if (next != spans.begin()) {
      auto prev = std::prev(next);
      CHECK(prev->first + prev->second <= at);  // none before
    }
    spans[at] = h.usable(p);
  };

  for (int op = 0; op < 100000; ++op) {
    const unsigned allocPercent = ((op / 5000) % 2 == 0) ? 68 : 32;  // fill, then drain
    const unsigned dice = unsigned(rng() % 100);
    if (live.empty() || dice < allocPercent) {
      const unsigned bits = unsigned(rng() % 17);
      const size_t size = 1 + size_t(rng()) % (size_t(1) << bits);  // 1..65536
      const size_t align = aligns[rng() % 3];
      const uint16_t budget = uint16_t(rng() % 20);
      const uint32_t owner = ++serial;
      void* p = h.allocate(size, align, budget, owner);
      if (!p) {
        ++allocFails;
        continue;
      }
      ++allocs;
      CHECK(uintptr_t(p) % align == 0);
      CHECK(h.contains(p) && h.usable(p) >= size);
      CHECK(h.budgetOf(p) == budget && h.ownerOf(p) == owner);
      CHECK(h.slidable(p) == (align <= RegionHeap::kMinAlign));
      CHECK(h.owns(p));
      Live l = {p, size, align, uint8_t(serial * 37u + 11u), budget, owner};
      std::memset(p, l.fill, size);
      insertSpan(p);
      live.push_back(l);
    } else if (dice < allocPercent + 4) {
      Live& l = live[rng() % live.size()];
      const bool expected = h.slidable(l.p) && prevIsFree(h, l.p);
      void* moved = h.slideDown(l.p);
      CHECK((moved != nullptr) == expected);
      if (moved) {
        ++slides;
        CHECK(uintptr_t(moved) < uintptr_t(l.p));
        CHECK(uintptr_t(moved) % 16 == 0);
        CHECK(allBytes(moved, l.size, l.fill));
        CHECK(h.budgetOf(moved) == l.budget && h.ownerOf(moved) == l.owner);
        CHECK(h.owns(moved));
        spans.erase(uintptr_t(l.p));
        l.p = moved;
        insertSpan(moved);
      } else {
        ++slideRefused;
      }
    } else {
      const size_t i = rng() % live.size();
      Live l = live[i];
      CHECK(allBytes(l.p, l.size, l.fill));  // nobody wrote over it
      spans.erase(uintptr_t(l.p));
      h.release(l.p);
      ++releases;
      live[i] = live.back();
      live.pop_back();
    }
    if (op % 499 == 0) {
      CHECK(h.validate());
      CHECK(h.used() + h.freeBytes() == sizeof arena);
      CHECK(h.used() <= h.peak());
      CHECK(walkedBytes(h, false) == h.used() && walkedBytes(h, true) == h.freeBytes());
      CHECK(mapAgrees(h, arena));
    }
  }
  CHECK(allocs > 10000 && releases > 10000);
  CHECK(allocFails > 100);  // the region really did run full
  CHECK(slides > 100 && slideRefused > 100);
  CHECK(h.peak() > sizeof arena / 2);

  // drain in random order
  std::shuffle(live.begin(), live.end(), rng);
  for (const Live& l : live) {
    CHECK(allBytes(l.p, l.size, l.fill));
    h.release(l.p);
  }
  CHECK(h.used() == 0);
  CHECK(h.freeBytes() == free0 && h.largestFree() == largest0);
  CHECK(blockCount(h) == 1);
  CHECK(h.freeBlocks() == 1 && mapAgrees(h, arena));
  CHECK(h.validate());
  std::printf("stress: %zu allocations, %zu failed, %zu releases, %zu slides, peak %zu\n",
              allocs, allocFails, releases, slides, h.peak());
}

}  // namespace

int main() {
  testRoundTripAndExhaustion();
  testCoalescing();
  testSlide();
  testAccounting();
  testOwns();
  testOverAligned();
  testExactReuse();
  testChecks();
  testFreeBlocksAndMap();
  testStress();
  return finish("heap");
}
