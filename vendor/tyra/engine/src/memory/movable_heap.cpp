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

// Added by TyraX: the movable heap (inc/memory/movable_heap.hpp). Portable C++
// - no PS2 header, no assumption about the size of a pointer.
//
// Every used block of the RegionHeap is a bucket, and every bucket is in
// `buckets_`, sorted by address; so a hole in front of a bucket is exactly the
// space between the end of the bucket before it and its own header, and a walk
// of the list is a walk of the heap. A bucket keeps nothing about itself that
// the heap's block header does not hold already (its size, budget and token),
// except whether it was allocated pinned. Sizes are read from the heap when
// they are needed because they can change under the list: an over-aligned
// block whose gap is one header long is absorbed by the block in front of it.

#include "memory/movable_heap.hpp"

#include <cstring>

namespace Tyra {
namespace Memory {
namespace {

// The address the public function returns to - the code that called the heap.
// A macro, so the builtin runs in that function and not in a helper.
#define TYRA_MOVABLE_CALLER() \
  reinterpret_cast<uintptr_t>(__builtin_return_address(0))

constexpr const char* kName = "movable heap";

inline uintptr_t addressOf(const void* p) {
  return reinterpret_cast<uintptr_t>(p);
}

}  // namespace

MovableHeap::MovableHeap() : hook_{&pinThunk, &unpinThunk, this} {}

MovableHeap::~MovableHeap() {
  if (detail::g_pinHook == &hook_) detail::g_pinHook = nullptr;
}

void MovableHeap::init(void* base, size_t bytes, const uint32_t* extensionCaps,
                       PtrRegistry* ptrs) {
  heap_.init(base, bytes);
  first_ = (addressOf(base) + RegionHeap::kMinAlign - 1) &
           ~uintptr_t(RegionHeap::kMinAlign - 1);
  ptrs_ = ptrs;
  for (unsigned b = 0; b < Budget::Max; ++b) {
    caps_[b] = extensionCaps != nullptr ? extensionCaps[b] : 0;
    usedBy_[b] = 0;
  }
  count_ = 0;
  pinRows_ = 0;
  movedLast_ = 0;
}

void MovableHeap::installPinHook() { detail::g_pinHook = &hook_; }

// ---- the list of buckets ---------------------------------------------------

// How many buckets start at or before `address`.
uint32_t MovableHeap::countAtOrBefore(uintptr_t address) const {
  uint32_t lo = 0, hi = count_;
  while (lo < hi) {
    const uint32_t mid = lo + (hi - lo) / 2;
    if (addressOf(buckets_[mid].base) <= address) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return lo;
}

uint32_t MovableHeap::indexOf(const void* base) const {
  const uint32_t n = countAtOrBefore(addressOf(base));
  return n != 0 && buckets_[n - 1].base == base ? n - 1 : kMaxHeapBuckets;
}

uint32_t MovableHeap::tokenAt(const void* p) const {
  if (!heap_.contains(p)) return 0;
  const uintptr_t a = addressOf(p);
  const uint32_t n = countAtOrBefore(a);
  if (n == 0) return 0;
  const unsigned char* base = buckets_[n - 1].base;
  if (a - addressOf(base) >= heap_.usable(base)) return 0;
  return heap_.ownerOf(base);
}

uint32_t MovableHeap::tokenAtCallback(const void* p, void* heap) {
  return static_cast<const MovableHeap*>(heap)->tokenAt(p);
}

size_t MovableHeap::usedBy(BudgetId b) const {
  return b < Budget::Max ? usedBy_[b] : 0;
}

// ---- allocation ------------------------------------------------------------

void* MovableHeap::allocateBucket(size_t bytes, size_t align, bool pinned,
                                  uint32_t owner, BudgetId budget) {
#if TYRA_MEMORY_CHECKS
  const uintptr_t caller = TYRA_MOVABLE_CALLER();
#endif

  if (budget >= Budget::Max || owner == 0) {
#if TYRA_MEMORY_CHECKS
    fail(Failure{budget >= Budget::Max ? "movable heap budget out of range"
                                       : "movable heap bucket without a token",
                 kName, bytes, caller});
#endif
    return nullptr;
  }
  if (caps_[budget] == 0) return nullptr;

  // The block the request needs: a header, the payload rounded to 16, at least
  // the smallest block.
  constexpr size_t kRound = RegionHeap::kMinAlign;
  if (bytes > SIZE_MAX - RegionHeap::kHeader - kRound) return nullptr;
  size_t need = RegionHeap::kHeader + ((bytes + kRound - 1) & ~(kRound - 1));
  if (need < 2 * RegionHeap::kHeader) need = 2 * RegionHeap::kHeader;
  if (usedBy_[budget] > caps_[budget] ||
      need > caps_[budget] - usedBy_[budget]) {
    return nullptr;
  }

  if (count_ == kMaxHeapBuckets) return nullptr;  // the pool's policy decides
#if TYRA_MEMORY_CHECKS
  for (uint32_t i = 0; i < count_; ++i) {
    if (heap_.ownerOf(buckets_[i].base) == owner) {
      fail(Failure{"movable heap bucket token already in use", kName, bytes,
                   caller});
      return nullptr;
    }
  }
#endif

  const size_t usedBefore = heap_.used();
  unsigned char* p = static_cast<unsigned char*>(
      heap_.allocate(bytes, align, static_cast<uint16_t>(budget), owner));
  if (p == nullptr) return nullptr;

  // The block is charged for what it really took. A block that is over-aligned
  // by exactly one header leaves a gap too small to be a block, which the block
  // in front of it absorbs: those bytes belong to that block's budget.
  const size_t block = blockBytes(p);
  usedBy_[budget] += block;
  const uint32_t at = countAtOrBefore(addressOf(p));
  const size_t absorbed = heap_.used() - usedBefore - block;
  if (absorbed != 0 && at != 0) {
    usedBy_[heap_.budgetOf(buckets_[at - 1].base)] += absorbed;
  }

  std::memmove(&buckets_[at + 1], &buckets_[at],
               (count_ - at) * sizeof(Bucket));
  buckets_[at].base = p;
  buckets_[at].pinned = pinned;
  ++count_;
  return p;
}

void MovableHeap::releaseBucket(void* base) {
  if (base == nullptr) return;
  const uint32_t at = indexOf(base);
  if (at == kMaxHeapBuckets) {
#if TYRA_MEMORY_CHECKS
    fail(Failure{"movable heap invalid release", kName, 0,
                 TYRA_MOVABLE_CALLER()});
#endif
    return;
  }

  const uint32_t token = heap_.ownerOf(base);
  usedBy_[heap_.budgetOf(base)] -= blockBytes(buckets_[at].base);
  for (uint32_t i = 0; i < pinRows_; ++i) {  // pins on a bucket die with it
    if (pins_[i].token == token) {
      pins_[i] = pins_[--pinRows_];
      break;
    }
  }
  std::memmove(&buckets_[at], &buckets_[at + 1],
               (count_ - at - 1) * sizeof(Bucket));
  --count_;
  if (ptrs_ != nullptr) ptrs_->table().dropBucket(token);
  heap_.release(base);
}

// ---- pins ------------------------------------------------------------------

MovableHeap::PinCount* MovableHeap::findPin(uint32_t token) {
  for (uint32_t i = 0; i < pinRows_; ++i) {
    if (pins_[i].token == token) return &pins_[i];
  }
  return nullptr;
}

const MovableHeap::PinCount* MovableHeap::findPin(uint32_t token) const {
  for (uint32_t i = 0; i < pinRows_; ++i) {
    if (pins_[i].token == token) return &pins_[i];
  }
  return nullptr;
}

void MovableHeap::pin(const void* inside) {
  const uint32_t token = tokenAt(inside);
  if (token == 0) return;
  if (PinCount* row = findPin(token)) {
    ++row->count;
    return;
  }
  if (pinRows_ == kMaxHeapBuckets) {
    fail(Failure{"pin table full", kName, 0, TYRA_MOVABLE_CALLER()});
    return;
  }
  pins_[pinRows_].token = token;
  pins_[pinRows_].count = 1;
  ++pinRows_;
}

void MovableHeap::unpin(const void* inside) {
  const uint32_t token = tokenAt(inside);
  if (token == 0) return;
  PinCount* row = findPin(token);
  if (row == nullptr) return;
  if (--row->count == 0) *row = pins_[--pinRows_];
}

void MovableHeap::pinThunk(const void* raw, void* ctx) {
  static_cast<MovableHeap*>(ctx)->pin(raw);
}

void MovableHeap::unpinThunk(const void* raw, void* ctx) {
  static_cast<MovableHeap*>(ctx)->unpin(raw);
}

// ---- defragmentation -------------------------------------------------------

void MovableHeap::setMoveListener(void (*onMove)(uint32_t, void*, void*),
                                  void* ctx) {
  onMove_ = onMove;
  onMoveCtx_ = ctx;
}

bool MovableHeap::movable(const Bucket& b) const {
  return !b.pinned && heap_.slidable(b.base) &&
         findPin(heap_.ownerOf(b.base)) == nullptr;
}

size_t MovableHeap::step(size_t budgetBytes) {
  movedLast_ = 0;
  if (budgetBytes == 0 || count_ == 0) return 0;

  uint32_t moves = 0;
  size_t moved = 0;
  uintptr_t end = first_;  // where the block before bucket i ends
  for (uint32_t i = 0; i < count_; ++i) {
    unsigned char* base = buckets_[i].base;
    const bool hole = addressOf(base) - RegionHeap::kHeader > end;
    if (hole && movable(buckets_[i])) {
      const size_t block = blockBytes(base);
      // Over the budget: stop - unless this would be the step's first move,
      // which is let through however big it is.
      if (moved != 0 && (moved > budgetBytes || block > budgetBytes - moved)) {
        break;
      }
      if (moves == kMaxMovesPerStep) break;

      const uint32_t token = heap_.ownerOf(base);
      const size_t payload = heap_.usable(base);
      unsigned char* to = static_cast<unsigned char*>(heap_.slideDown(base));
      if (to != nullptr) {
        moves_[moves++] = Move{addressOf(base), payload, addressOf(to), token};
        buckets_[i].base = to;
        base = to;
        moved += block;
        if (onMove_ != nullptr) onMove_(token, to, onMoveCtx_);
      }
    }
    end = addressOf(base) + heap_.usable(base);
  }

  if (moves != 0 && ptrs_ != nullptr) ptrs_->table().resolve(moves_, moves);
  movedLast_ = moved;
  return moved;
}

// ---- self-check ------------------------------------------------------------

bool MovableHeap::validate() const {
  if (!heap_.validate()) return false;

  // The list is the heap's used blocks, in order; the counts are their sums.
  size_t sums[Budget::Max] = {};
  uint32_t seen = 0;
  bool agrees = true;
  heap_.forEachBlock([&](const RegionHeap::BlockInfo& b) {
    if (b.free) return;
    if (seen >= count_ || b.payload != buckets_[seen].base ||
        b.budget >= Budget::Max) {
      agrees = false;
      return;
    }
    sums[b.budget] += b.bytes + RegionHeap::kHeader;
    ++seen;
  });
  if (!agrees || seen != count_) return false;
  for (unsigned b = 0; b < Budget::Max; ++b) {
    if (sums[b] != usedBy_[b]) return false;
  }

  // A pin row names a live bucket, once, and counts something.
  for (uint32_t i = 0; i < pinRows_; ++i) {
    if (pins_[i].count == 0) return false;
    bool live = false;
    for (uint32_t j = 0; j < count_ && !live; ++j) {
      live = heap_.ownerOf(buckets_[j].base) == pins_[i].token;
    }
    if (!live) return false;
    for (uint32_t j = i + 1; j < pinRows_; ++j) {
      if (pins_[j].token == pins_[i].token) return false;
    }
  }
  return true;
}

}  // namespace Memory
}  // namespace Tyra
