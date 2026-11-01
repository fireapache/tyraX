// Added by TyraX: host verifier for the budget registry, the fixed-memory
// layout and the boot-time fit check (inc/memory/registry.hpp,
// src/memory/registry.cpp). Built and run by tools/verify-memory.py, once with
// TYRA_MEMORY_CHECKS=0 and once with =1.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "memory/ptr.hpp"
#include "memory/registry.hpp"
#include "verify-memory-check.hpp"

namespace {

constexpr size_t kMB = 1u << 20;

// ---- the lines printTable hands to print ----------------------------------

std::vector<std::string> g_table;

void collect(const char* line) { g_table.push_back(line); }

std::vector<std::string> tableOf(const Registry& r, const Layout& L) {
  g_table.clear();
  r.printTable(&collect, L);
  return g_table;
}

bool any(const std::vector<std::string>& lines, const char* text) {
  for (const std::string& l : lines) {
    if (l.find(text) != std::string::npos) return true;
  }
  return false;
}

bool has(const std::vector<std::string>& lines, const std::string& exact) {
  return std::find(lines.begin(), lines.end(), exact) != lines.end();
}

std::string num(size_t v) {
  return std::to_string(static_cast<unsigned long>(v));
}

// What printTable writes for a size: whole KB, rounded up.
std::string kb(size_t bytes) { return num((bytes + 1023) / 1024); }

// ---- the brief's checks ----------------------------------------------------

void testBriefChecks() {
  Registry r;
  Layout L;
  CHECK(r.fits(32u << 20, L));
  CHECK(L.systemBytes ==
        (32u << 20) - L.fixedBytes - r.settings.newlibReserveBytes);

  r.budget(Budget::FirstGame, "Probe", Kind::Logic).extensionBytes = 8192;
  PoolId p =
      r.pool({"probe", Budget::FirstGame, 100, 16, 32, 8, 4, Policy::Refuse});
  CHECK(r.poolFixedBytes(p) == r.bucketBytes(p, 32));
  CHECK(r.bucketBytes(p, 32) >= 32 * 112);

  r.budget(Budget::Renderer).regionBytes = 2u << 20;
  CHECK(r.fits(32u << 20, L));
  CHECK(L.heapBytes == 8192);
  CHECK(L.regionOffset[Budget::Renderer] % 64 == 0);

  CHECK(!r.fits(L.fixedBytes + r.settings.newlibReserveBytes - 1, L));
  CHECK(strstr(r.error(), "does not fit") != nullptr);

  Registry bad;
  bad.pool({"orphan", 40, 8, 16, 1, 1, 0, Policy::Halt});
  CHECK(!bad.fits(32u << 20, L));

  std::vector<std::string> lines = tableOf(r, L);
  CHECK(any(lines, "Renderer") && any(lines, "probe"));
}

// ---- the engine rows, the budget() call, the pool() call -------------------

void testEngineRows() {
  Registry r;
  CHECK(r.settings.ptrTableCapacity == 4096);
  CHECK(r.settings.defragBytesPerFrame == 64 * 1024);
  CHECK(r.settings.newlibReserveBytes == 256 * 1024);
  CHECK(r.settings.systemBytes == 0);
  CHECK(r.settings.refuseHeadroomBytes == 512 * 1024);
  CHECK(r.settings.refusalReportFrames == 60);
  CHECK(r.poolCount() == 0);

  struct Row {
    BudgetId id;
    const char* name;
    Kind kind;
  };
  const Row rows[] = {
      {Budget::System, "System", Kind::System},
      {Budget::FrameScratch, "FrameScratch", Kind::Scratch},
      {Budget::LoadScratch, "LoadScratch", Kind::Scratch},
      {Budget::Renderer, "Renderer", Kind::Render},
      {Budget::Skinning, "Skinning", Kind::Render},
      {Budget::BakedStreams, "BakedStreams", Kind::Render},
      {Budget::Audio, "Audio", Kind::Asset},
      {Budget::Level, "Level", Kind::Asset},
  };
  for (const Row& row : rows) {
    const BudgetDesc* d = r.findBudget(row.id);
    CHECK(d != nullptr);
    if (d == nullptr) continue;
    CHECK(std::strcmp(d->name, row.name) == 0);
    CHECK(d->kind == row.kind);
    CHECK(d->regionBytes == 0);
    CHECK(d->extensionBytes == 0);
    // Render data and assets are requests: refused, not halted on.
    CHECK(d->onFull == defaultOnFull(row.kind));
  }
  CHECK(r.findBudget(Budget::System)->onFull == OnFull::Halt);
  CHECK(r.findBudget(Budget::FrameScratch)->onFull == OnFull::Halt);
  CHECK(r.findBudget(Budget::Renderer)->onFull == OnFull::Refuse);
  CHECK(r.findBudget(Budget::Skinning)->onFull == OnFull::Refuse);
  CHECK(r.findBudget(Budget::BakedStreams)->onFull == OnFull::Refuse);
  CHECK(r.findBudget(Budget::Audio)->onFull == OnFull::Refuse);
  CHECK(r.findBudget(Budget::Level)->onFull == OnFull::Refuse);
  CHECK(defaultOnFull(Kind::Logic) == OnFull::Halt);
  CHECK(defaultOnFull(Kind::System) == OnFull::Halt);
  CHECK(defaultOnFull(Kind::Scratch) == OnFull::Halt);
  CHECK(defaultOnFull(Kind::Render) == OnFull::Refuse);
  CHECK(defaultOnFull(Kind::Asset) == OnFull::Refuse);

  // The engine's ids are the dense run 0..7; the game's start at FirstGame.
  CHECK(Budget::System == 0 && Budget::Level == 7);
  CHECK(Budget::FirstGame == 32 && Budget::Max == 64);
  CHECK(r.findBudget(8) == nullptr);
  CHECK(r.findBudget(Budget::FirstGame) == nullptr);
  CHECK(r.findBudget(Budget::Max) == nullptr);
  CHECK(r.findBudget(0xFFFF) == nullptr);

  // Every engine budget is measured: nothing is capped, nothing reserved.
  Layout L;
  CHECK(r.fits(32 * kMB, L));
  CHECK(L.heapBytes == 0);
  for (const Row& row : rows) {
    CHECK(r.findBudget(row.id)->regionBytes == 0);
  }
}

void testBudgetCall() {
  Registry r;

  // An absent id is added; the row is returned to override.
  BudgetDesc& g = r.budget(Budget::FirstGame, "Gameplay", Kind::Logic);
  CHECK(std::strcmp(g.name, "Gameplay") == 0 && g.kind == Kind::Logic);
  CHECK(g.regionBytes == 0 && g.extensionBytes == 0);
  g.regionBytes = 4096;
  CHECK(r.findBudget(Budget::FirstGame) == &g);

  // A present id answers with the same row, and a null name leaves it alone.
  BudgetDesc& again = r.budget(Budget::FirstGame);
  CHECK(&again == &g);
  CHECK(again.regionBytes == 4096);
  CHECK(std::strcmp(again.name, "Gameplay") == 0);
  BudgetDesc& renderer = r.budget(Budget::Renderer);
  CHECK(std::strcmp(renderer.name, "Renderer") == 0 &&
        renderer.kind == Kind::Render);

  // A name declares the row again: the name and the kind change - and the
  // onFull with the kind - the sizes stay.
  CHECK(g.onFull == OnFull::Halt);
  BudgetDesc& renamed = r.budget(Budget::FirstGame, "Actors", Kind::Asset);
  CHECK(&renamed == &g);
  CHECK(std::strcmp(renamed.name, "Actors") == 0 &&
        renamed.kind == Kind::Asset);
  CHECK(renamed.regionBytes == 4096);
  CHECK(renamed.onFull == OnFull::Refuse);
  // A game may say otherwise for a row, and a null name leaves that alone.
  renamed.onFull = OnFull::Halt;
  CHECK(r.budget(Budget::FirstGame).onFull == OnFull::Halt);
  r.budget(Budget::Level).onFull = OnFull::Halt;
  CHECK(r.findBudget(Budget::Level)->onFull == OnFull::Halt);

  // A new id without a name still prints as something.
  BudgetDesc& unnamed = r.budget(Budget::FirstGame + 1);
  CHECK(unnamed.name != nullptr && unnamed.name[0] != '\0');
  CHECK(unnamed.kind == Kind::Logic && unnamed.onFull == OnFull::Halt);

  // The last id is allowed; one past it is a registration error that fits()
  // reports.
  r.budget(Budget::Max - 1, "Last");
  Layout L;
  CHECK(r.fits(32 * kMB, L));
  r.budget(Budget::Max, "Beyond");
  CHECK(!r.fits(32 * kMB, L));
  CHECK(strstr(r.error(), "budget id") != nullptr);
}

void testPoolCall() {
  Registry r;
  r.budget(Budget::FirstGame, "Probe");
  PoolDesc d = {"a", Budget::FirstGame, 8, 8, 4, 0, 0, Policy::Refuse};
  CHECK(r.pool(d) == 0);
  d.name = "b";
  CHECK(r.pool(d) == 1);
  d.name = "c";
  CHECK(r.pool(d) == 2);
  CHECK(r.poolCount() == 3);
  CHECK(std::strcmp(r.poolDesc(0).name, "a") == 0);
  CHECK(std::strcmp(r.poolDesc(2).name, "c") == 0);
  CHECK(r.poolDesc(1).typeSize == 8 && r.poolDesc(1).policy == Policy::Refuse);

  // A pool id the registry never issued has no bytes.
  CHECK(r.poolFixedBytes(3) == 0);
  CHECK(r.bucketBytes(3, 100) == 0);
  CHECK(r.poolFixedBytes(kNoPool) == 0);

  // The table holds kMaxPools pools; the next is a registration error.
  Registry full;
  full.budget(Budget::FirstGame, "Probe");
  for (int i = 0; i < kMaxPools; ++i) {
    CHECK(full.pool(d) == i);
  }
  Layout L;
  CHECK(full.fits(64 * kMB, L));
  CHECK(full.pool(d) == kNoPool);
  CHECK(full.poolCount() == static_cast<uint32_t>(kMaxPools));
  CHECK(!full.fits(64 * kMB, L));
  CHECK(strstr(full.error(), "pool") != nullptr);
}

// ---- bucket bytes -----------------------------------------------------------

struct BucketCase {
  uint32_t typeSize, align, count;
  Policy policy;
  size_t base;         // entries + live bitmap
  size_t stamps;       // RecycleOldest only
  size_t generations;  // TYRA_MEMORY_CHECKS only
};

void testBucketBytes() {
  const BucketCase cases[] = {
      // stride 112: 32 * 112 = 3584, bitmap 4 -> 16
      {100, 16, 32, Policy::Refuse, 3584 + 16, 128, 64},
      // typeSize 1, align 1 -> stride 4: 5 * 4 = 20 -> 32, bitmap 16
      {1, 1, 5, Policy::Refuse, 32 + 16, 32, 16},
      // typeSize 0, align 0 -> stride 4 as well
      {0, 0, 5, Policy::Refuse, 32 + 16, 32, 16},
      // stride 24: 240, bitmap (10 + 7) / 8 = 2 -> 16
      {24, 8, 10, Policy::Refuse, 240 + 16, 48, 32},
      // align 64: stride 64, 3 * 64 = 192, bitmap 16
      {40, 64, 3, Policy::Refuse, 192 + 16, 16, 16},
      // 130 entries: bitmap (130 + 7) / 8 = 17 -> 32
      {16, 16, 130, Policy::Refuse, 130 * 16 + 32, 528, 272},
      // nothing in the bucket, nothing to pay for
      {64, 16, 0, Policy::Refuse, 0, 0, 0},
  };
  for (const BucketCase& c : cases) {
    Registry r;
    r.budget(Budget::FirstGame, "Probe");
    for (Policy policy :
         {Policy::Refuse, Policy::RecycleOldest, Policy::Halt}) {
      PoolDesc d = {
          "p", Budget::FirstGame, c.typeSize, c.align, c.count, 0, 0, policy};
      PoolId id = r.pool(d);
      size_t expected = c.base;
      if (policy == Policy::RecycleOldest) expected += c.stamps;
#if TYRA_MEMORY_CHECKS
      expected += c.generations;
#endif
      CHECK(r.bucketBytes(id, c.count) == expected);
      CHECK(r.poolFixedBytes(id) == expected);
    }
  }

  // bucketBytes takes the count as an argument: an extension is not the first
  // bucket.
  Registry r;
  r.budget(Budget::FirstGame, "Probe");
  PoolId id =
      r.pool({"probe", Budget::FirstGame, 100, 16, 32, 8, 4, Policy::Refuse});
  CHECK(r.poolFixedBytes(id) == r.bucketBytes(id, 32));
  CHECK(r.bucketBytes(id, 8) < r.bucketBytes(id, 32));
  CHECK(r.bucketBytes(id, 8) >= 8 * 112);
  CHECK(r.bucketBytes(id, 64) > r.bucketBytes(id, 32));

  // Every bucket is a multiple of 16, so the entry after it keeps its
  // alignment.
  for (uint32_t n = 0; n < 200; ++n) {
    CHECK(r.bucketBytes(id, n) % 16 == 0);
  }
}

// ---- bucket parts: the one formula, as offsets ------------------------------

struct PartsCase {
  uint32_t typeSize, align, count;
  Policy policy;
  uint32_t stride;
  size_t bitmap;  // after the entries
  size_t stamps;  // after the bitmap
  size_t
      generations;  // after the stamps (a stamps-sized gap when RecycleOldest)
  size_t stampBytes;  // the part's size when the policy is RecycleOldest
  size_t genBytes;    // the part's size with TYRA_MEMORY_CHECKS
};

void testBucketParts() {
  const PartsCase cases[] = {
      // stride 112: 32 * 112 = 3584; bitmap 4 -> 16; stamps 128; gens 64
      {100, 16, 32, Policy::Refuse, 112, 3584, 3584 + 16, 3584 + 16, 128, 64},
      {100, 16, 32, Policy::RecycleOldest, 112, 3584, 3584 + 16, 3584 + 16, 128,
       64},
      // stride 4 (typeSize 1, align 1): 20 -> 32, bitmap 16
      {1, 1, 5, Policy::Halt, 4, 32, 48, 48, 32, 16},
      // stride 24: 240, bitmap 2 -> 16
      {24, 8, 10, Policy::RecycleOldest, 24, 240, 256, 256, 48, 32},
      // align 64: stride 64
      {40, 64, 3, Policy::Refuse, 64, 192, 208, 208, 16, 16},
      // 130 entries: bitmap 17 -> 32
      {16, 16, 130, Policy::RecycleOldest, 16, 2080, 2112, 2112, 528, 272},
      // nothing in the bucket, nothing to pay for
      {64, 16, 0, Policy::RecycleOldest, 64, 0, 0, 0, 0, 0},
  };
  for (const PartsCase& c : cases) {
    Registry r;
    r.budget(Budget::FirstGame, "Probe");
    PoolId id = r.pool(
        {"p", Budget::FirstGame, c.typeSize, c.align, c.count, 0, 0, c.policy});
    const BucketParts p = r.bucketParts(id, c.count);

    const bool stamps = c.policy == Policy::RecycleOldest;
    size_t genBytes = 0;
#if TYRA_MEMORY_CHECKS
    genBytes = c.genBytes;
#endif
    CHECK(p.stride == c.stride);
    CHECK(p.entries == 0);
    CHECK(p.bitmap == c.bitmap);
    CHECK(p.stamps == c.stamps);
    // An absent part is zero-sized, and its offset is still the next part's.
    CHECK(p.generations == c.generations + (stamps ? c.stampBytes : 0));
    CHECK(p.total == p.generations + genBytes);

    // The bucket's size has one formula, and it is this one.
    CHECK(p.total == r.bucketBytes(id, c.count));
    CHECK(p.total == r.poolFixedBytes(id));

    // Every part starts on a multiple of 16, in order.
    CHECK(p.entries % 16 == 0 && p.bitmap % 16 == 0 && p.stamps % 16 == 0 &&
          p.generations % 16 == 0 && p.total % 16 == 0);
    CHECK(p.entries <= p.bitmap && p.bitmap <= p.stamps &&
          p.stamps <= p.generations && p.generations <= p.total);

    // The parts add up to the bucket: no gap, no overlap.
    CHECK((p.bitmap - p.entries) + (p.stamps - p.bitmap) +
              (p.generations - p.stamps) + (p.total - p.generations) ==
          p.total);

    // The entries part holds `count` entries at the stride; the bitmap has a
    // bit for each; the stamps and generations have their own width for each.
    CHECK(p.bitmap - p.entries >= size_t(c.count) * p.stride);
    CHECK((p.stamps - p.bitmap) * 8 >= c.count);
    CHECK(p.generations - p.stamps == (stamps ? c.stampBytes : 0));
    CHECK(p.total - p.generations == genBytes);
  }

  // An extension has its own count, so its own offsets.
  Registry r;
  r.budget(Budget::FirstGame, "Probe");
  PoolId id = r.pool(
      {"probe", Budget::FirstGame, 24, 8, 4, 100, 3, Policy::RecycleOldest});
  const BucketParts first = r.bucketParts(id, 4);
  const BucketParts ext = r.bucketParts(id, 100);
  CHECK(first.stride == ext.stride && first.stride == 24);
  CHECK(first.total == r.poolFixedBytes(id));
  CHECK(ext.total == r.bucketBytes(id, 100));
  CHECK(ext.bitmap == 2400 && ext.stamps == 2400 + 16);
  CHECK(ext.bitmap > first.bitmap && ext.total > first.total);

  // Every count: aligned, ordered, summing to bucketBytes.
  for (uint32_t n = 0; n < 200; ++n) {
    const BucketParts p = r.bucketParts(id, n);
    CHECK(p.bitmap % 16 == 0 && p.stamps % 16 == 0 && p.generations % 16 == 0 &&
          p.total % 16 == 0);
    CHECK(p.bitmap <= p.stamps && p.stamps <= p.generations &&
          p.generations <= p.total);
    CHECK(p.total == r.bucketBytes(id, n));
  }

  // The nested spelling is the same type.
  Registry::BucketParts nested = r.bucketParts(id, 4);
  CHECK(nested.total == first.total);

  // An unknown pool has no parts at all.
  const BucketParts none = r.bucketParts(7, 10);
  CHECK(none.entries == 0 && none.bitmap == 0 && none.stamps == 0 &&
        none.generations == 0 && none.total == 0 && none.stride == 0);
  const BucketParts noPool = r.bucketParts(kNoPool, 10);
  CHECK(noPool.total == 0 && noPool.stride == 0);

  // A count too big for a word saturates instead of wrapping.
  const BucketParts huge = r.bucketParts(id, 0xFFFFFFFFu);
  CHECK(huge.total >= huge.generations && huge.generations >= huge.stamps &&
        huge.stamps >= huge.bitmap && huge.bitmap >= 0xFFFFFFFFu);
}

// ---- the layout
// ----------------------------------------------------------------

struct Piece {
  size_t offset, bytes;
  std::string what;
};

// The pieces of the fixed block a layout names, sorted by offset.
std::vector<Piece> piecesOf(const Registry& r, const Layout& L) {
  std::vector<Piece> v;
  for (uint32_t i = 0; i < r.poolCount(); ++i) {
    v.push_back(
        {L.poolOffset[i], r.poolFixedBytes(static_cast<PoolId>(i)), "pool"});
  }
  for (BudgetId id = 0; id < Budget::Max; ++id) {
    const BudgetDesc* d = r.findBudget(id);
    if (d != nullptr && d->regionBytes != 0) {
      v.push_back({L.regionOffset[id], d->regionBytes, d->name});
    }
  }
  v.push_back({L.ptrOffset, L.ptrBytes, "ptr"});
  v.push_back({L.heapOffset, L.heapBytes, "heap"});
  v.push_back({L.systemOffset, L.systemBytes, "system"});
  std::stable_sort(v.begin(), v.end(), [](const Piece& a, const Piece& b) {
    return a.offset < b.offset;
  });
  return v;
}

void testLayout() {
  Registry r;
  r.budget(Budget::FirstGame, "Actors").extensionBytes = 10000;
  r.budget(Budget::FirstGame + 1, "Props", Kind::Render).extensionBytes = 5000;
  r.budget(Budget::FirstGame + 2, "Scratchy", Kind::Scratch).regionBytes =
      300 * 1024;
  r.budget(Budget::Renderer).regionBytes = 2 * kMB;
  r.budget(Budget::FrameScratch).regionBytes = 512 * 1024 + 48;
  r.budget(Budget::Level).regionBytes = 3 * kMB;
  PoolId a =
      r.pool({"a", Budget::FirstGame, 100, 16, 32, 8, 4, Policy::Refuse});
  PoolId b = r.pool(
      {"b", Budget::FirstGame + 1, 24, 8, 10, 10, 2, Policy::RecycleOldest});
  PoolId c = r.pool({"c", Budget::Renderer, 1, 1, 3, 0, 0, Policy::Halt});

  Layout L;
  CHECK(r.fits(64 * kMB, L));

  // The brief's order: pools' first buckets, regions by id, the Ptr table, the
  // movable heap, system - each piece 64-aligned.
  CHECK(L.poolOffset[a] < L.poolOffset[b]);
  CHECK(L.poolOffset[b] < L.poolOffset[c]);
  CHECK(L.poolOffset[c] < L.regionOffset[Budget::FrameScratch]);
  CHECK(L.regionOffset[Budget::FrameScratch] <
        L.regionOffset[Budget::Renderer]);
  CHECK(L.regionOffset[Budget::Renderer] < L.regionOffset[Budget::Level]);
  CHECK(L.regionOffset[Budget::Level] < L.regionOffset[Budget::FirstGame + 2]);
  CHECK(L.regionOffset[Budget::FirstGame + 2] < L.ptrOffset);
  CHECK(L.ptrOffset < L.heapOffset);
  CHECK(L.heapOffset < L.systemOffset);

  CHECK(L.poolOffset[a] == 0);
  CHECK(L.poolOffset[a] % 64 == 0 && L.poolOffset[b] % 64 == 0 &&
        L.poolOffset[c] % 64 == 0);
  for (BudgetId id = 0; id < Budget::Max; ++id) {
    const BudgetDesc* d = r.findBudget(id);
    if (d != nullptr && d->regionBytes != 0)
      CHECK(L.regionOffset[id] % 64 == 0);
  }
  CHECK(L.ptrOffset % 64 == 0 && L.heapOffset % 64 == 0 &&
        L.systemOffset % 64 == 0);
  CHECK(L.blockAlign == 64);

  // The sizes: the Ptr table's own, the heap as the sum of the budgets'.
  CHECK(L.ptrBytes == PtrTable::bytesFor(r.settings.ptrTableCapacity));
  CHECK(L.ptrBytes == 2 * static_cast<size_t>(r.settings.ptrTableCapacity) * 2 *
                          sizeof(uintptr_t));
  CHECK(L.heapBytes >= 15000 && L.heapBytes < 15000 + 64);
  CHECK(L.heapBytes % 64 == 0);

  // The pieces are disjoint, in the block, and the block ends where system
  // does.
  std::vector<Piece> pieces = piecesOf(r, L);
  for (size_t i = 0; i + 1 < pieces.size(); ++i) {
    CHECK(pieces[i].offset + pieces[i].bytes <= pieces[i + 1].offset);
  }
  CHECK(L.systemOffset + L.systemBytes == L.blockBytes);
  CHECK(L.blockBytes == 64 * kMB - r.settings.newlibReserveBytes);
  CHECK(L.fixedBytes == L.systemOffset);
  CHECK(L.fixedBytes >= r.poolFixedBytes(a) + r.poolFixedBytes(b) +
                            r.poolFixedBytes(c) +
                            (2 * kMB + 3 * kMB + 300 * 1024 + 512 * 1024 + 48) +
                            L.heapBytes + L.ptrBytes);
  CHECK(L.fixedBytes + L.systemBytes + r.settings.newlibReserveBytes ==
        64 * kMB);

  // The Ptr table follows its capacity.
  Layout small;
  r.settings.ptrTableCapacity = 16;
  CHECK(r.fits(64 * kMB, small));
  CHECK(small.ptrBytes == PtrTable::bytesFor(16));
  CHECK(small.fixedBytes < L.fixedBytes);
  r.settings.ptrTableCapacity = 0;
  CHECK(r.fits(64 * kMB, small));
  CHECK(small.ptrBytes == 0);

  // The table hashes by masking, so its capacity is a power of two (or 0).
  r.settings.ptrTableCapacity = 1000;
  CHECK(!r.fits(64 * kMB, small));
  CHECK(strstr(r.error(), "ptrTableCapacity") != nullptr &&
        strstr(r.error(), "power of two") != nullptr);
  r.settings.ptrTableCapacity = 1024;
  CHECK(r.fits(64 * kMB, small));
  CHECK(r.error() != nullptr && r.error()[0] == '\0');
}

void testLayoutAlignment() {
  // A pool aligned to more than 64 gets a first bucket on its own alignment,
  // and the block must be allocated to match.
  Registry r;
  r.budget(Budget::FirstGame, "Probe");
  r.pool({"plain", Budget::FirstGame, 24, 8, 3, 0, 0, Policy::Refuse});
  PoolId wide =
      r.pool({"wide", Budget::FirstGame, 256, 256, 2, 0, 0, Policy::Refuse});
  PoolId after =
      r.pool({"after", Budget::FirstGame, 8, 8, 1, 0, 0, Policy::Refuse});
  Layout L;
  CHECK(r.fits(16 * kMB, L));
  CHECK(L.poolOffset[wide] % 256 == 0);
  CHECK(L.poolOffset[wide] > 0);
  CHECK(L.poolOffset[after] >= L.poolOffset[wide] + r.poolFixedBytes(wide));
  CHECK(L.blockAlign == 256);

  // With no wide pool the block needs 64 only.
  Registry n;
  Layout M;
  CHECK(n.fits(16 * kMB, M));
  CHECK(M.blockAlign == 64);

  // Offsets of the regions stay 64-aligned behind odd-sized pools.
  Registry odd;
  odd.budget(Budget::FirstGame, "Probe");
  odd.budget(Budget::Renderer).regionBytes = 1000;
  odd.budget(Budget::Level).regionBytes = 1000;
  odd.pool({"odd", Budget::FirstGame, 3, 1, 7, 0, 0, Policy::Refuse});
  CHECK(odd.fits(16 * kMB, M));
  CHECK(M.regionOffset[Budget::Renderer] % 64 == 0);
  CHECK(M.regionOffset[Budget::Level] % 64 == 0);
  CHECK(M.regionOffset[Budget::Level] >=
        M.regionOffset[Budget::Renderer] + 1000);
}

void testExplicitSystemBytes() {
  Registry r;
  r.settings.systemBytes = 1 * kMB;
  r.budget(Budget::Renderer).regionBytes = 2 * kMB;
  Layout L;
  CHECK(r.fits(32 * kMB, L));
  CHECK(L.systemBytes == 1 * kMB);
  CHECK(L.fixedBytes == L.systemOffset + 1 * kMB);
  CHECK(L.blockBytes == L.fixedBytes);
  CHECK(L.blockBytes < 32 * kMB - r.settings.newlibReserveBytes);

  // The explicit system region counts as fixed memory in the fit check.
  CHECK(r.fits(L.fixedBytes + r.settings.newlibReserveBytes, L));
  CHECK(!r.fits(L.fixedBytes + r.settings.newlibReserveBytes - 1, L));
  CHECK(strstr(r.error(), "does not fit") != nullptr);
}

// ---- the fit check
// ----------------------------------------------------------------

void testFitBoundary() {
  Registry r;
  r.budget(Budget::Renderer).regionBytes = 2 * kMB;
  r.budget(Budget::FirstGame, "Probe").extensionBytes = 4096;
  r.pool({"probe", Budget::FirstGame, 64, 16, 100, 0, 0, Policy::Refuse});
  Layout L;
  CHECK(r.fits(32 * kMB, L));
  CHECK(r.error() != nullptr && r.error()[0] == '\0');

  const size_t exact = L.fixedBytes + r.settings.newlibReserveBytes;
  Layout atEdge;
  CHECK(r.fits(exact, atEdge));
  CHECK(atEdge.systemBytes == 0);
  CHECK(atEdge.fixedBytes == L.fixedBytes);
  CHECK(r.error()[0] == '\0');

  Layout over;
  CHECK(!r.fits(exact - 1, over));
  CHECK(strstr(r.error(), "does not fit") != nullptr);
  // The layout is still filled in so the table can be printed.
  CHECK(over.fixedBytes == L.fixedBytes);
  CHECK(over.regionOffset[Budget::Renderer] ==
        L.regionOffset[Budget::Renderer]);
  CHECK(over.systemBytes == 0);

  // A later success clears the error.
  CHECK(r.fits(32 * kMB, L));
  CHECK(r.error()[0] == '\0');

  CHECK(!r.fits(0, L));
  CHECK(strstr(r.error(), "does not fit") != nullptr);

  // A larger reserve moves the edge.
  r.settings.newlibReserveBytes += 4096;
  CHECK(!r.fits(exact, L));
  CHECK(r.fits(exact + 4096, L));
}

void testFitErrors() {
  Layout L;

  // A pool of a budget nobody registered.
  Registry orphan;
  orphan.pool({"orphan", 40, 8, 16, 1, 1, 0, Policy::Halt});
  CHECK(!orphan.fits(32 * kMB, L));
  CHECK(strstr(orphan.error(), "orphan") != nullptr);
  CHECK(strstr(orphan.error(), "budget") != nullptr);
  orphan.budget(40, "Late");
  CHECK(orphan.fits(32 * kMB, L));
  CHECK(orphan.error()[0] == '\0');

  // A pool that takes extensions from the movable heap may be aligned to 128,
  // no more.
  Registry r;
  r.budget(Budget::FirstGame, "Probe").extensionBytes = 1 * kMB;
  r.pool({"a128", Budget::FirstGame, 128, 128, 4, 4, 2, Policy::Refuse});
  CHECK(r.fits(32 * kMB, L));
  r.pool({"a256", Budget::FirstGame, 256, 256, 4, 4, 2, Policy::Refuse});
  CHECK(!r.fits(32 * kMB, L));
  CHECK(strstr(r.error(), "a256") != nullptr);
  CHECK(strstr(r.error(), "align") != nullptr);

  // ... but a pool that never extends lives in the fixed block only.
  Registry fixedOnly;
  fixedOnly.budget(Budget::FirstGame, "Probe");
  fixedOnly.pool(
      {"noext", Budget::FirstGame, 256, 256, 4, 0, 2, Policy::Refuse});
  fixedOnly.pool(
      {"nomax", Budget::FirstGame, 256, 256, 4, 4, 0, Policy::Refuse});
  CHECK(fixedOnly.fits(32 * kMB, L));

  // An alignment that is not a power of two has no layout.
  Registry odd;
  odd.budget(Budget::FirstGame, "Probe");
  odd.pool({"odd24", Budget::FirstGame, 24, 24, 4, 0, 0, Policy::Refuse});
  CHECK(!odd.fits(32 * kMB, L));
  CHECK(strstr(odd.error(), "odd24") != nullptr);
  CHECK(strstr(odd.error(), "power of two") != nullptr);

  // A registry too big to count in a word does not wrap around into fitting.
  Registry huge;
  huge.budget(Budget::FirstGame, "Probe");
  huge.pool({"huge", Budget::FirstGame, 0xFFFFFFFFu, 8, 0xFFFFFFFFu, 0, 0,
             Policy::RecycleOldest});
  huge.pool({"huge2", Budget::FirstGame, 0xFFFFFFFFu, 8, 0xFFFFFFFFu, 0, 0,
             Policy::RecycleOldest});
  CHECK(!huge.fits(SIZE_MAX / 2, L));
  CHECK(strstr(huge.error(), "does not fit") != nullptr);
  Registry hugeRegion;
  hugeRegion.budget(Budget::Renderer).regionBytes = 0xFFFFFFF0u;
  hugeRegion.budget(Budget::Level).regionBytes = 0xFFFFFFF0u;
  hugeRegion.budget(Budget::Audio).regionBytes = 0xFFFFFFF0u;
  CHECK(!hugeRegion.fits(1u << 30, L));
  CHECK(strstr(hugeRegion.error(), "does not fit") != nullptr);
}

void testFitDoesNotFail() {
  // The registry reports; the manager decides to fail. fits() never calls
  // fail().
  installThrowingFailure();
  Layout L;
  Registry bad;
  bad.pool({"orphan", 40, 8, 16, 1, 1, 0, Policy::Halt});
  bool threw = false;
  try {
    CHECK(!bad.fits(32 * kMB, L));
    Registry tight;
    CHECK(!tight.fits(1024, L));
    bad.budget(Budget::Max, "Beyond");
    bad.pool({"x", Budget::Max, 8, 8, 1, 0, 0, Policy::Refuse});
    CHECK(!bad.fits(32 * kMB, L));
  } catch (const ThrownFailure&) {
    threw = true;
  }
  CHECK(!threw);
  CHECK(capturedFailureLines().empty());
}

// ---- the table
// ----------------------------------------------------------------------

void testTable() {
  Registry r;
  r.budget(Budget::FirstGame, "Probe", Kind::Logic).extensionBytes = 8192;
  PoolId p =
      r.pool({"probe", Budget::FirstGame, 100, 16, 32, 8, 4, Policy::Refuse});
  r.pool(
      {"recycler", Budget::FirstGame, 24, 8, 10, 0, 0, Policy::RecycleOldest});
  r.pool({"strict", Budget::Renderer, 64, 16, 2, 0, 0, Policy::Halt});
  r.budget(Budget::Renderer).regionBytes = 2 * kMB;
  r.budget(Budget::FrameScratch).regionBytes = 100;
  Layout L;
  CHECK(r.fits(32 * kMB, L));

  std::vector<std::string> lines = tableOf(r, L);

  // Lines carry no newline.
  for (const std::string& l : lines) {
    CHECK(l.find('\n') == std::string::npos &&
          l.find('\r') == std::string::npos);
    CHECK(l.compare(0, 6, "[MEM] ") == 0);
  }

  // A budget line per budget, by id: a measured one prints region=0, and
  // each says what it does when full.
  CHECK(has(lines,
            "[MEM] budget System kind=System region=0 heap=0 full=Halt"));
  CHECK(has(lines,
            "[MEM] budget FrameScratch kind=Scratch region=1 heap=0 full=Halt"));
  CHECK(has(lines,
            "[MEM] budget LoadScratch kind=Scratch region=0 heap=0 full=Halt"));
  CHECK(has(lines,
            "[MEM] budget Renderer kind=Render region=2048 heap=0 full=Refuse"));
  CHECK(has(lines,
            "[MEM] budget Skinning kind=Render region=0 heap=0 full=Refuse"));
  CHECK(has(lines, "[MEM] budget BakedStreams kind=Render region=0 heap=0 "
                   "full=Refuse"));
  CHECK(has(lines,
            "[MEM] budget Audio kind=Asset region=0 heap=0 full=Refuse"));
  CHECK(has(lines,
            "[MEM] budget Level kind=Asset region=0 heap=0 full=Refuse"));
  CHECK(has(lines, "[MEM] budget Probe kind=Logic region=0 heap=8 full=Halt"));

  // A pool line per pool.
  const std::string fixed = kb(r.poolFixedBytes(p));
  CHECK(has(
      lines,
      "[MEM] pool probe budget=Probe first=32 ext=8x4 policy=Refuse fixed=" +
          fixed));
  CHECK(has(lines,
            "[MEM] pool recycler budget=Probe first=10 ext=0x0 "
            "policy=RecycleOldest fixed=1"));
  CHECK(has(
      lines,
      "[MEM] pool strict budget=Renderer first=2 ext=0x0 policy=Halt fixed=1"));

  // The total closes the table.
  const std::string total = "[MEM] total fixed=" + kb(L.fixedBytes) +
                            " system=" + kb(L.systemBytes) +
                            " reserve=" + kb(r.settings.newlibReserveBytes);
  CHECK(!lines.empty() && lines.back() == total);
  CHECK(total.find("reserve=256") != std::string::npos);

  // Budgets come first (by id), then pools (by id), then the total.
  size_t lastBudget = 0, firstPool = lines.size();
  for (size_t i = 0; i < lines.size(); ++i) {
    if (lines[i].compare(0, 13, "[MEM] budget ") == 0) lastBudget = i;
    if (lines[i].compare(0, 11, "[MEM] pool ") == 0 &&
        firstPool == lines.size())
      firstPool = i;
  }
  CHECK(lastBudget < firstPool);
  CHECK(lines[0] ==
        "[MEM] budget System kind=System region=0 heap=0 full=Halt");
  CHECK(lines.size() == 9 + 3 + 1);

  // A pool of a missing budget still prints (the fit check is what refuses it).
  Registry orphan;
  orphan.pool({"orphan", 40, 8, 16, 1, 1, 0, Policy::Halt});
  Layout bad;
  CHECK(!orphan.fits(32 * kMB, bad));
  std::vector<std::string> orphanLines = tableOf(orphan, bad);
  CHECK(any(orphanLines, "[MEM] pool orphan budget=?"));

  // The table of a registry that does not fit says so with real numbers.
  Layout over;
  CHECK(!r.fits(1 * kMB, over));
  std::vector<std::string> overLines = tableOf(r, over);
  CHECK(overLines.back() ==
        "[MEM] total fixed=" + kb(over.fixedBytes) + " system=0 reserve=256");
}

}  // namespace

int main() {
  testBriefChecks();
  testEngineRows();
  testBudgetCall();
  testPoolCall();
  testBucketBytes();
  testBucketParts();
  testLayout();
  testLayoutAlignment();
  testExplicitSystemBytes();
  testFitBoundary();
  testFitErrors();
  testFitDoesNotFail();
  testTable();
  return finish("registry");
}
