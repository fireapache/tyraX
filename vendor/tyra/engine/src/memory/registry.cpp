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

// Added by TyraX: the memory manager's registry (inc/memory/registry.hpp).
// Portable C++ - no PS2 header, no assumption about the size of a pointer - so
// the host verifiers run it as it is. Sizes are summed in 64 bits and saturate,
// so a registry too big for a 32-bit word does not wrap around into fitting.

#include "memory/registry.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "memory/failure.hpp"
#include "memory/ptr.hpp"

namespace Tyra {
namespace Memory {
namespace {

// Every fixed piece starts on a multiple of this; the block is allocated to
// match.
constexpr uint64_t kPieceAlign = 64;
// A pool that takes extensions from the movable heap may ask for this much
// alignment, no more.
constexpr uint32_t kMaxHeapAlign = 128;
constexpr uint64_t kSaturated = UINT64_MAX;

uint64_t satAdd(uint64_t a, uint64_t b) {
  return a > kSaturated - b ? kSaturated : a + b;
}

uint64_t satMul(uint64_t a, uint64_t b) {
  return b != 0 && a > kSaturated / b ? kSaturated : a * b;
}

uint64_t roundUp(uint64_t v, uint64_t unit) {
  if (unit <= 1) return v;
  const uint64_t s = satAdd(v, unit - 1);
  return s == kSaturated ? kSaturated : s / unit * unit;
}

size_t narrow(uint64_t v) {
  return static_cast<size_t>(std::min<uint64_t>(v, SIZE_MAX));
}

unsigned long text(uint64_t v) {
  return static_cast<unsigned long>(std::min<uint64_t>(v, ~0ul));
}

// What the table prints for a size: whole KB, rounded up so a size that is not
// zero never shows as 0.
unsigned long kb(uint64_t bytes) { return text(roundUp(bytes, 1024) / 1024); }

bool isPowerOfTwo(uint32_t v) { return v != 0 && (v & (v - 1)) == 0; }

const char* orQuestion(const char* s) { return s != nullptr ? s : "?"; }

const char* kindName(Kind k) {
  static const char* const names[] = {"System", "Logic", "Render", "Asset",
                                      "Scratch"};
  const unsigned i = static_cast<unsigned>(k);
  return i < sizeof names / sizeof names[0] ? names[i] : "?";
}

const char* onFullName(OnFull f) {
  return f == OnFull::Refuse ? "Refuse" : "Halt";
}

const char* policyName(Policy p) {
  static const char* const names[] = {"Refuse", "RecycleOldest", "Halt"};
  const unsigned i = static_cast<unsigned>(p);
  return i < sizeof names / sizeof names[0] ? names[i] : "?";
}

// The one formula for a bucket of `count` entries: the entries, each at a
// stride that keeps the type's alignment; the live bitmap (a bit per entry);
// for RecycleOldest the stamps (4 bytes per entry) that say which entry is
// oldest; with TYRA_MEMORY_CHECKS the generations (2 bytes per entry). Each
// part is padded to 16 so the next one stays aligned. Offsets are in 64 bits
// and saturate; bucketParts narrows them.
struct Parts {
  uint64_t entries, bitmap, stamps, generations, total, stride;
};

Parts partsOf(const PoolDesc& d, uint32_t count) {
  Parts p;
  p.stride = roundUp(std::max<uint32_t>(d.typeSize, 4),
                     std::max<uint32_t>(d.align, 4));
  p.entries = 0;
  p.bitmap = roundUp(satMul(count, p.stride), 16);
  p.stamps =
      satAdd(p.bitmap, roundUp((static_cast<uint64_t>(count) + 7) / 8, 16));
  p.generations = p.stamps;
  if (d.policy == Policy::RecycleOldest) {
    p.generations =
        satAdd(p.stamps, roundUp(static_cast<uint64_t>(count) * 4, 16));
  }
  p.total = p.generations;
#if TYRA_MEMORY_CHECKS
  p.total =
      satAdd(p.generations, roundUp(static_cast<uint64_t>(count) * 2, 16));
#endif
  return p;
}

uint64_t bucketBytesOf(const PoolDesc& d, uint32_t count) {
  return partsOf(d, count).total;
}

}  // namespace

Registry::Registry()
    : poolCount_(0),
      dummy_{"?", Kind::Logic, 0, 0, OnFull::Halt},
      registrationError_(nullptr) {
  for (BudgetId id = 0; id < Budget::Max; ++id) {
    budgets_[id] = BudgetDesc{nullptr, Kind::Logic, 0, 0, OnFull::Halt};
    known_[id] = false;
  }
  for (int i = 0; i < kMaxPools; ++i) pools_[i] = PoolDesc{};
  error_[0] = '\0';

  // The engine's rows. Every size is 0: each budget is served from system's
  // region until a game gives it a region of its own. Renderer, Skinning and
  // BakedStreams (Render) and Audio and Level (Asset) refuse when full; the
  // rest halt.
  budget(Budget::System, "System", Kind::System);
  budget(Budget::FrameScratch, "FrameScratch", Kind::Scratch);
  budget(Budget::LoadScratch, "LoadScratch", Kind::Scratch);
  budget(Budget::Renderer, "Renderer", Kind::Render);
  budget(Budget::Skinning, "Skinning", Kind::Render);
  budget(Budget::BakedStreams, "BakedStreams", Kind::Render);
  budget(Budget::Audio, "Audio", Kind::Asset);
  budget(Budget::Level, "Level", Kind::Asset);
}

BudgetDesc& Registry::budget(BudgetId id, const char* name, Kind kind) {
  if (id >= Budget::Max) {
    if (registrationError_ == nullptr) {
      registrationError_ = "registry: budget id is not below Budget::Max";
    }
    dummy_ = BudgetDesc{"?", Kind::Logic, 0, 0, OnFull::Halt};
    return dummy_;
  }
  BudgetDesc& row = budgets_[id];
  if (!known_[id]) {
    known_[id] = true;
    row = BudgetDesc{"unnamed", Kind::Logic, 0, 0, OnFull::Halt};
  }
  if (name != nullptr) {
    row.name = name;
    row.kind = kind;
    row.onFull = defaultOnFull(kind);
  }
  return row;
}

const BudgetDesc* Registry::findBudget(BudgetId id) const {
  return id < Budget::Max && known_[id] ? &budgets_[id] : nullptr;
}

PoolId Registry::pool(const PoolDesc& d) {
  if (poolCount_ >= static_cast<uint32_t>(kMaxPools)) {
    if (registrationError_ == nullptr) {
      registrationError_ = "registry: more pools than kMaxPools";
    }
    return kNoPool;
  }
  pools_[poolCount_] = d;
  return static_cast<PoolId>(poolCount_++);
}

uint32_t Registry::poolCount() const { return poolCount_; }

const PoolDesc& Registry::poolDesc(PoolId id) const {
  static const PoolDesc empty = {};
  return id < poolCount_ ? pools_[id] : empty;
}

const char* Registry::error() const { return error_; }

BucketParts Registry::bucketParts(PoolId id, uint32_t count) const {
  BucketParts out = {0, 0, 0, 0, 0, 0};
  if (id >= poolCount_) return out;
  const Parts p = partsOf(pools_[id], count);
  out.entries = narrow(p.entries);
  out.bitmap = narrow(p.bitmap);
  out.stamps = narrow(p.stamps);
  out.generations = narrow(p.generations);
  out.total = narrow(p.total);
  out.stride = static_cast<uint32_t>(std::min<uint64_t>(p.stride, UINT32_MAX));
  return out;
}

size_t Registry::bucketBytes(PoolId id, uint32_t count) const {
  return bucketParts(id, count).total;
}

size_t Registry::poolFixedBytes(PoolId id) const {
  return id < poolCount_ ? bucketBytes(id, pools_[id].firstCount) : 0;
}

bool Registry::fits(size_t availableBytes, Layout& out) const {
  std::memset(&out, 0, sizeof out);
  error_[0] = '\0';

  // The pieces, in this order, each on a multiple of 64: every pool's first
  // bucket, the budgets' regions by id, the pointer table, the movable heap and
  // last the system region. A pool whose type asks for more than 64 starts on
  // that multiple, and the block must be allocated to match.
  uint64_t offset = 0;
  uint64_t blockAlign = kPieceAlign;
  for (uint32_t i = 0; i < poolCount_; ++i) {
    const PoolDesc& d = pools_[i];
    const uint64_t unit = std::max<uint64_t>(d.align, kPieceAlign);
    offset = roundUp(offset, unit);
    out.poolOffset[i] = narrow(offset);
    offset = satAdd(offset, bucketBytesOf(d, d.firstCount));
    blockAlign = std::max(blockAlign, unit);
  }

  uint64_t extensionBytes = 0;
  for (BudgetId id = 0; id < Budget::Max; ++id) {
    if (!known_[id]) continue;
    extensionBytes = satAdd(extensionBytes, budgets_[id].extensionBytes);
    if (budgets_[id].regionBytes == 0) continue;
    offset = roundUp(offset, kPieceAlign);
    out.regionOffset[id] = narrow(offset);
    offset = satAdd(offset, budgets_[id].regionBytes);
  }

  const uint64_t ptrBytes = PtrTable::bytesFor(settings.ptrTableCapacity);
  offset = roundUp(offset, kPieceAlign);
  out.ptrOffset = narrow(offset);
  out.ptrBytes = narrow(ptrBytes);
  offset = satAdd(offset, ptrBytes);

  const uint64_t heapBytes = roundUp(extensionBytes, kPieceAlign);
  offset = roundUp(offset, kPieceAlign);
  out.heapOffset = narrow(offset);
  out.heapBytes = narrow(heapBytes);
  offset = satAdd(offset, heapBytes);

  offset = roundUp(offset, kPieceAlign);
  out.systemOffset = narrow(offset);

  // What the registry asks for up front. system is the rest of the RAM unless
  // the game gave it a size, which then counts as fixed.
  const uint64_t fixed = satAdd(offset, settings.systemBytes);
  const uint64_t needed = satAdd(fixed, settings.newlibReserveBytes);
  const bool roomLeft = needed <= availableBytes;
  const uint64_t systemBytes = settings.systemBytes != 0
                                   ? settings.systemBytes
                                   : (roomLeft ? availableBytes - needed : 0);
  out.fixedBytes = narrow(fixed);
  out.systemBytes = narrow(systemBytes);
  out.blockBytes = narrow(satAdd(offset, systemBytes));
  out.blockAlign = narrow(blockAlign);

  if (registrationError_ != nullptr) {
    std::snprintf(error_, sizeof error_, "%s", registrationError_);
    return false;
  }
  for (uint32_t i = 0; i < poolCount_; ++i) {
    const PoolDesc& d = pools_[i];
    if (findBudget(d.budget) == nullptr) {
      std::snprintf(error_, sizeof error_,
                    "pool '%s' names budget %u, which is not registered",
                    orQuestion(d.name), static_cast<unsigned>(d.budget));
      return false;
    }
    if (d.align > 1 && !isPowerOfTwo(d.align)) {
      std::snprintf(error_, sizeof error_,
                    "pool '%s' align %lu is not a power of two",
                    orQuestion(d.name), text(d.align));
      return false;
    }
    if (d.maxExtensions != 0 && d.extensionCount != 0 &&
        d.align > kMaxHeapAlign) {
      std::snprintf(error_, sizeof error_,
                    "pool '%s' align %lu is above %lu, the most the movable "
                    "heap serves",
                    orQuestion(d.name), text(d.align), text(kMaxHeapAlign));
      return false;
    }
  }
  if (settings.ptrTableCapacity != 0 &&
      !isPowerOfTwo(settings.ptrTableCapacity)) {
    std::snprintf(error_, sizeof error_,
                  "ptrTableCapacity %lu is not a power of two",
                  text(settings.ptrTableCapacity));
    return false;
  }
  if (!roomLeft) {
    std::snprintf(error_, sizeof error_,
                  "registry does not fit: fixed %lu + newlib reserve %lu bytes "
                  "> %lu bytes available",
                  text(fixed), text(settings.newlibReserveBytes),
                  text(availableBytes));
    return false;
  }
  return true;
}

void Registry::printTable(void (*print)(const char*),
                          const Layout& layout) const {
  if (print == nullptr) return;
  char line[224];

  for (BudgetId id = 0; id < Budget::Max; ++id) {
    if (!known_[id]) continue;
    const BudgetDesc& b = budgets_[id];
    std::snprintf(line, sizeof line,
                  "[MEM] budget %s kind=%s region=%lu heap=%lu full=%s",
                  orQuestion(b.name), kindName(b.kind), kb(b.regionBytes),
                  kb(b.extensionBytes), onFullName(b.onFull));
    print(line);
  }

  for (uint32_t i = 0; i < poolCount_; ++i) {
    const PoolDesc& d = pools_[i];
    const BudgetDesc* b = findBudget(d.budget);
    std::snprintf(line, sizeof line,
                  "[MEM] pool %s budget=%s first=%lu ext=%lux%lu policy=%s "
                  "fixed=%lu",
                  orQuestion(d.name), b != nullptr ? orQuestion(b->name) : "?",
                  text(d.firstCount), text(d.extensionCount),
                  text(d.maxExtensions), policyName(d.policy),
                  kb(poolFixedBytes(static_cast<PoolId>(i))));
    print(line);
  }

  std::snprintf(line, sizeof line,
                "[MEM] total fixed=%lu system=%lu reserve=%lu",
                kb(layout.fixedBytes), kb(layout.systemBytes),
                kb(settings.newlibReserveBytes));
  print(line);
}

}  // namespace Memory
}  // namespace Tyra
