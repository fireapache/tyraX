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

// Added by TyraX: the movable heap - the region the pools' extension buckets
// live in - and the incremental defragmentation of it.
//
// It is a RegionHeap that hands out whole buckets (BucketSource) and keeps a
// list of them in address order. A bucket's token (the owner the pool passes
// to allocateBucket, bucketToken) is stored in its block header, so the token
// of any address in the heap is one binary search away (tokenAt).
//
// step() is the defragmentation: at the start of a frame it slides buckets
// down into the free blocks in front of them, up to a number of bytes, tells
// each bucket's owner where its bucket went (setMoveListener - the pool's
// bucketMoved) and then runs ONE PtrTable::resolve for the whole batch, so
// every registered Ptr is right again before step() returns. Nothing else in
// the engine moves memory, and nothing moves it in the middle of a frame.
//
// A bucket never moves while it is pinned: allocated pinned (a render-kind
// pool's buckets and every over-aligned bucket are - they may be read by
// hardware or need an alignment a slide could not keep), held by a Pin<T>
// (installPinHook makes Pin<T> pin through this heap), or aligned to more than
// 16 bytes, which a RegionHeap never slides.
//
// One thread only: nothing here locks.
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#pragma once

#include <cstddef>
#include <cstdint>

#include "memory/failure.hpp"
#include "memory/pool.hpp"
#include "memory/ptr.hpp"
#include "memory/region_heap.hpp"
#include "memory/registry.hpp"

namespace Tyra {
namespace Memory {

class MovableHeap : public BucketSource {
 public:
  // The most buckets alive at once. A fixed table, so the heap allocates
  // nothing of its own; one more is a failure ("bucket table full"). The pin
  // table has as many rows, and a row exists only for a live bucket, so it
  // cannot overflow.
  static constexpr uint32_t kMaxBuckets = 512;

  // The most buckets one step() moves. The step ends there, as if its budget
  // were spent: resolve looks at every move for every registered Ptr, so the
  // batch stays small.
  static constexpr uint32_t kMaxMovesPerStep = 32;

  MovableHeap();
  ~MovableHeap() override;
  MovableHeap(const MovableHeap&) = delete;
  MovableHeap& operator=(const MovableHeap&) = delete;

  // Takes over [base, base + bytes) (a RegionHeap range: 16-aligned, a multiple
  // of 16) and forgets every bucket it held. `extensionCaps[b]` is the most
  // block bytes budget b's buckets may hold together (the registry's
  // BudgetDesc::extensionBytes), indexed by BudgetId, Budget::Max entries; the
  // values are copied. A cap of 0 - and a null array - means the budget may
  // take no bucket. `ptrs` is the registry that gets one resolve after a step
  // and loses the rows of a released bucket; null for none. The move listener
  // and the pin hook are not touched.
  void init(void* base, size_t bytes, const uint32_t* extensionCaps,
            PtrRegistry* ptrs);

  // Makes Pin<T> pin through this heap: sets detail::g_pinHook. The heap takes
  // it back when it is destroyed.
  void installPinHook();

  // A bucket of `bytes` bytes at `align` (a RegionHeap alignment: above 16 the
  // bucket can never be moved), charged to `budget` and tagged with `owner`,
  // its token (never 0). `pinned` keeps it where it is for good. Null - and
  // nothing changes - when the budget has no room left for it (its cap is
  // tested against the block the request needs: header, payload rounded to 16)
  // or the heap has no block that fits. The memory is not initialised.
  //
  // Fails in every build when the table of buckets is full. With
  // TYRA_MEMORY_CHECKS a budget past Budget::Max, a token of 0 or a token a
  // live bucket has already go to fail too (and answer null).
  void* allocateBucket(size_t bytes, size_t align, bool pinned, uint32_t owner,
                       BudgetId budget) override;

  // Gives back the bucket that starts at `base` (the address it has now),
  // forgets its pins and drops the rows of the Ptrs that pointed into it
  // (PtrTable::dropBucket), so a later bucket with the same token is never
  // patched for them. null is ignored; so is an address that is not a bucket's
  // start - with TYRA_MEMORY_CHECKS it goes to fail.
  void releaseBucket(void* base) override;

  // Keeps the bucket that holds `inside` where it is, once more (pins nest)
  // until the same number of unpins. An address in no bucket does nothing.
  // These are what Pin<T> calls through the hook. Fails when the pin table is
  // full, which it cannot be unless the heap is damaged.
  void pin(const void* inside);
  void unpin(const void* inside);

  // The token of the bucket that holds `p` (anywhere in its payload), 0 when it
  // is in none: in free space, a block header, outside the heap, null.
  // O(log n) over the buckets in address order.
  uint32_t tokenAt(const void* p) const;
  // tokenAt as PtrRegistry::init wants it: `heap` is the MovableHeap.
  static uint32_t tokenAtCallback(const void* p, void* heap);

  // Called once for every bucket step() moves, after its bytes are at the new
  // place and before the Ptrs are resolved: `token` is the bucket's, `newBase`
  // its new payload address. It must not touch a Ptr and must not call the
  // heap. Null: no listener.
  void setMoveListener(void (*onMove)(uint32_t token, void* newBase, void* ctx),
                       void* ctx);

  // Slides buckets down into the free blocks in front of them, in address
  // order. A bucket moves if it is not pinned, not over-aligned and has a free
  // block directly in front of it; the walk stops at the first such bucket that
  // would take the bytes moved so far past `budgetBytes` (counted in block
  // bytes, header and payload), except that a bucket bigger than the whole
  // budget moves when nothing else has - alone, in a step of its own, so it can
  // never wait forever. A step moves at most kMaxMovesPerStep buckets. A budget
  // of 0 moves nothing. Returns the block bytes moved (also movedLastStep()).
  //
  // When it returns every moved bucket's owner has been told and the Ptrs are
  // resolved; a raw pointer into a moved bucket is stale (see Ptr).
  size_t step(size_t budgetBytes);

  // Block bytes (header and payload, the heap's own accounting) budget `b`
  // holds. 0 for a budget past Budget::Max.
  size_t usedBy(BudgetId b) const;
  size_t freeBytes() const { return heap_.freeBytes(); }
  // The payload of the largest free block (RegionHeap::largestFree).
  size_t largestFree() const { return heap_.largestFree(); }
  // Block bytes the last step() moved.
  size_t movedLastStep() const { return movedLast_; }
  uint32_t bucketCount() const { return count_; }

  // Checks the heap, the list of buckets against the heap's blocks, the
  // budgets' counts against a walk and the pin table. True when all of it
  // agrees; linear.
  bool validate() const;

 private:
  struct Bucket {
    unsigned char* base;  // the payload; the list is sorted by it
    bool pinned;          // allocated pinned: never moves
  };
  struct PinCount {
    uint32_t token;
    uint32_t count;
  };

  uint32_t countAtOrBefore(uintptr_t address) const;
  // The index of the bucket that starts at `base`; kMaxBuckets if none.
  uint32_t indexOf(const void* base) const;
  PinCount* findPin(uint32_t token);
  const PinCount* findPin(uint32_t token) const;
  size_t blockBytes(const unsigned char* base) const {
    return heap_.usable(base) + RegionHeap::kHeader;
  }
  bool movable(const Bucket& b) const;
  static void pinThunk(const void* raw, void* ctx);
  static void unpinThunk(const void* raw, void* ctx);

  RegionHeap heap_;
  uintptr_t first_ = 0;  // where the heap's first block starts
  PtrRegistry* ptrs_ = nullptr;
  uint32_t caps_[Budget::Max] = {};
  size_t usedBy_[Budget::Max] = {};
  Bucket buckets_[kMaxBuckets] = {};
  uint32_t count_ = 0;
  PinCount pins_[kMaxBuckets] = {};
  uint32_t pinRows_ = 0;
  Move moves_[kMaxMovesPerStep] = {};
  void (*onMove_)(uint32_t, void*, void*) = nullptr;
  void* onMoveCtx_ = nullptr;
  size_t movedLast_ = 0;
  detail::PinHook hook_;
};

}  // namespace Memory
}  // namespace Tyra
