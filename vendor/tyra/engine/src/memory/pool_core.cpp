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

// Added by TyraX: the type-erased pool (inc/memory/pool.hpp). Portable C++ - no
// PS2 header, no assumption about the size of a pointer - so the host verifiers
// run it as it is.

#include <cstring>

#include "memory/pool.hpp"

namespace Tyra {
namespace Memory {
namespace {

// The address the public function returns to - the code that called the pool.
// A macro, so the builtin runs in that function and not in a helper.
#define TYRA_POOL_CALLER() \
  reinterpret_cast<uintptr_t>(__builtin_return_address(0))

// The free list lives in the first 4 bytes of a free entry. memcpy: the bytes
// are raw storage whose type the pool does not know.
uint32_t readLink(const unsigned char* entry) {
  uint32_t v;
  std::memcpy(&v, entry, sizeof v);
  return v;
}

void writeLink(unsigned char* entry, uint32_t v) {
  std::memcpy(entry, &v, sizeof v);
}

#if TYRA_MEMORY_CHECKS
// An Id numbers the entries of a bucket in 16 bits.
constexpr uint32_t kMaxIdEntries = 0x10000;
#endif

}  // namespace

void PoolCore::reject(const char* what, size_t size, uintptr_t caller) const {
#if TYRA_MEMORY_CHECKS
  fail(Failure{what, name_, size, caller});
#else
  (void)what;
  (void)size;
  (void)caller;
#endif
}

void PoolCore::init(PoolId id, const PoolDesc& d, const Registry& r,
                    void* firstBucket, BucketSource* src) {
  const uintptr_t caller = TYRA_POOL_CALLER();
  const char* name = d.name != nullptr ? d.name : "?";

  // The registry's row is the layout; refuse what it cannot describe before
  // touching anything.
  const BucketParts first = r.bucketParts(id, d.firstCount);
  if (first.stride == 0) {
    fail(Failure{"pool is not in the registry", name, d.typeSize, caller});
    return;
  }
  if (d.extensionCount != 0 && d.maxExtensions >= kMaxBuckets) {
    fail(
        Failure{"pool has too many extensions", name, d.maxExtensions, caller});
    return;
  }
  if (d.firstCount != 0 && firstBucket == nullptr) {
    fail(Failure{"pool has no first bucket", name, d.firstCount, caller});
    return;
  }
  const uint32_t bucketAlign = d.align > 16 ? d.align : 16;
#if TYRA_MEMORY_CHECKS
  const PoolDesc& row = r.poolDesc(id);
  if (row.typeSize != d.typeSize || row.align != d.align ||
      row.firstCount != d.firstCount ||
      row.extensionCount != d.extensionCount ||
      row.maxExtensions != d.maxExtensions || row.policy != d.policy ||
      row.budget != d.budget) {
    fail(Failure{"pool is not the registry's row", name, d.typeSize, caller});
    return;
  }
  if (reinterpret_cast<uintptr_t>(firstBucket) % bucketAlign != 0) {
    fail(Failure{"pool first bucket is misaligned", name, d.typeSize, caller});
    return;
  }
  if (d.firstCount > kMaxIdEntries || d.extensionCount > kMaxIdEntries) {
    fail(Failure{"pool bucket has more entries than ids can number", name,
                 d.typeSize, caller});
    return;
  }
#endif

  name_ = name;
  src_ = src;
  recycle_ = nullptr;
  recycleCtx_ = nullptr;
  id_ = id;
  budget_ = d.budget;
  policy_ = d.policy;
  const BudgetDesc* budget = r.findBudget(d.budget);
  pinned_ = (budget != nullptr && budget->kind == Kind::Render) || d.align > 16;
  typeSize_ = d.typeSize;
  stride_ = first.stride;
  bucketAlign_ = bucketAlign;
  firstCount_ = d.firstCount;
  // An extension size of 0 means the pool never grows, whatever the cap says.
  extCount_ = d.extensionCount;
  maxExt_ = d.extensionCount != 0 ? d.maxExtensions : 0;
  firstParts_ = first;
  extParts_ = r.bucketParts(id, d.extensionCount);
  span_ = 1;
  live_ = 0;
  capacity_ = d.firstCount;
  exhaustions_ = 0;
  stamp_ = 0;
  for (uint32_t b = 0; b < kMaxBuckets; ++b) buckets_[b] = Bucket();
#if TYRA_MEMORY_CHECKS
  for (uint32_t b = 0; b < kMaxBuckets; ++b) slotSeed_[b] = 0;
#endif

  buckets_[0].base = static_cast<unsigned char*>(firstBucket);
  if (firstBucket != nullptr) formatBucket(0, firstBucket);
}

// Makes the bucket in slot `b` at `base` empty: nothing live, every entry free,
// the generations seeded.
void PoolCore::formatBucket(uint32_t b, void* base) {
  Bucket& k = buckets_[b];
  k.base = static_cast<unsigned char*>(base);
  k.live = 0;

  const BucketParts& p = partsOf(b);
  const uint32_t n = entriesIn(b);
  std::memset(k.base + p.bitmap, 0, p.stamps - p.bitmap);
  for (uint32_t i = 0; i < n; ++i) {
    writeLink(entryPtr(b, i), i + 1 < n ? i + 1 : kNil);
  }
  k.freeHead = n != 0 ? 0 : kNil;
#if TYRA_MEMORY_CHECKS
  uint16_t* g = generations(b);
  for (uint32_t i = 0; i < n; ++i) g[i] = slotSeed_[b];
#endif
}

// The bucket whose entries contain the address p, or kNoBucket. `index` is the
// entry p lies in; `exact` whether p is that entry's first byte.
uint32_t PoolCore::locate(const void* p, uint32_t& index, bool& exact) const {
  const uintptr_t a = reinterpret_cast<uintptr_t>(p);
  for (uint32_t b = 0; b < span_; ++b) {
    const uint32_t n = entriesIn(b);
    if (n == 0 || buckets_[b].base == nullptr) continue;
    const uintptr_t lo =
        reinterpret_cast<uintptr_t>(buckets_[b].base) + partsOf(b).entries;
    if (a < lo || a - lo >= uintptr_t(n) * stride_) continue;
    index = static_cast<uint32_t>((a - lo) / stride_);
    exact = (a - lo) % stride_ == 0;
    return b;
  }
  return kNoBucket;
}

void* PoolCore::acquire() {
  const uintptr_t caller = TYRA_POOL_CALLER();

  for (uint32_t b = 0; b < span_; ++b) {
    if (buckets_[b].freeHead != kNil) return take(b, caller);
  }
  if (void* e = grow(caller)) return e;

  ++exhaustions_;
  switch (policy_) {
    case Policy::Refuse:
      return nullptr;
    case Policy::RecycleOldest:
      return recycleOldest();
    case Policy::Halt:
      fail(Failure{"pool exhausted", name_, typeSize_, caller});
      return nullptr;
  }
  return nullptr;
}

// Takes the first free entry of bucket `b`, which has one.
void* PoolCore::take(uint32_t b, uintptr_t caller) {
  Bucket& k = buckets_[b];
  const uint32_t i = k.freeHead;
  unsigned char* e = entryPtr(b, i);
  const uint32_t next = readLink(e);
#if TYRA_MEMORY_CHECKS
  // A free entry's link was overwritten: something wrote to it after release.
  if (next != kNil && next >= entriesIn(b)) {
    fail(Failure{"pool free list corrupt", name_, typeSize_, caller});
    return nullptr;
  }
#else
  (void)caller;
#endif
  k.freeHead = next;
  unsigned char& bits = bitmap(b)[i >> 3];
  bits = static_cast<unsigned char>(bits | (1u << (i & 7)));
  ++k.live;
  ++live_;
  if (policy_ == Policy::RecycleOldest) stamps(b)[i] = ++stamp_;
  return e;
}

// A new extension bucket, if the cap, the source and the memory allow; the
// first entry of it, taken.
void* PoolCore::grow(uintptr_t caller) {
  if (src_ == nullptr || extCount_ == 0) return nullptr;

  // The lowest number that is free; the cap is on the numbers (1..maxExt_).
  uint32_t slot = kNoBucket;
  for (uint32_t b = 1; b < span_; ++b) {
    if (buckets_[b].base == nullptr) {
      slot = b;
      break;
    }
  }
  if (slot == kNoBucket) {
    if (span_ > maxExt_) return nullptr;
    slot = span_;
  }

  const uint32_t token = bucketToken(id_, static_cast<uint16_t>(slot));
  void* base = src_->allocateBucket(extParts_.total, bucketAlign_, pinned_,
                                    token, budget_);
  if (base == nullptr) return nullptr;
#if TYRA_MEMORY_CHECKS
  if (reinterpret_cast<uintptr_t>(base) % bucketAlign_ != 0) {
    src_->releaseBucket(base);
    fail(Failure{"pool bucket source returned a misaligned bucket", name_,
                 extParts_.total, caller});
    return nullptr;
  }
#endif

  formatBucket(slot, base);
  capacity_ += extCount_;
  if (slot == span_) ++span_;
  return take(slot, caller);
}

// The live entry acquired longest ago, through the hook, back as new.
void* PoolCore::recycleOldest() {
  uint32_t bestBucket = kNoBucket, bestIndex = 0, bestStamp = 0;
  for (uint32_t b = 0; b < span_; ++b) {
    if (!held(b) || buckets_[b].live == 0) continue;
    const uint32_t n = entriesIn(b);
    const unsigned char* bits = bitmap(b);
    const uint32_t* st = stamps(b);
    for (uint32_t i = 0; i < n; ++i) {
      if (((bits[i >> 3] >> (i & 7)) & 1) == 0) continue;
      // Stamps count acquisitions and wrap; compare their distance, which is
      // right while the live entries are less than 2^31 acquisitions apart.
      if (bestBucket == kNoBucket ||
          static_cast<int32_t>(st[i] - bestStamp) < 0) {
        bestBucket = b;
        bestIndex = i;
        bestStamp = st[i];
      }
    }
  }
  if (bestBucket == kNoBucket) return nullptr;

  unsigned char* e = entryPtr(bestBucket, bestIndex);
  if (recycle_ != nullptr) recycle_(e, recycleCtx_);
  stamps(bestBucket)[bestIndex] = ++stamp_;
#if TYRA_MEMORY_CHECKS
  ++generations(bestBucket)[bestIndex];
#endif
  return e;
}

void PoolCore::release(void* entry) {
  const uintptr_t caller = TYRA_POOL_CALLER();

  uint32_t i = 0;
  bool exact = false;
  const uint32_t b = locate(entry, i, exact);
  if (b == kNoBucket) {
    reject("pool release outside the pool", typeSize_, caller);
    return;
  }
  if (!exact) {
    reject("pool release of a misplaced pointer", typeSize_, caller);
    return;
  }
  if (!isSet(b, i)) {
    reject("pool released twice", typeSize_, caller);
    return;
  }

  Bucket& k = buckets_[b];
  unsigned char& bits = bitmap(b)[i >> 3];
  bits = static_cast<unsigned char>(bits & ~(1u << (i & 7)));
  writeLink(static_cast<unsigned char*>(entry), k.freeHead);
  k.freeHead = i;
  --k.live;
  --live_;
#if TYRA_MEMORY_CHECKS
  ++generations(b)[i];
#endif
}

void PoolCore::setRecycle(void (*hook)(void* entry, void* ctx), void* ctx) {
  recycle_ = hook;
  recycleCtx_ = ctx;
}

// Gives the extension in slot `b` back to the source.
void PoolCore::releaseBucket(uint32_t b) {
#if TYRA_MEMORY_CHECKS
  // The next bucket in this slot starts past every generation this one reached.
  const uint32_t n = entriesIn(b);
  const uint16_t* g = generations(b);
  uint16_t top = slotSeed_[b];
  for (uint32_t i = 0; i < n; ++i) {
    if (g[i] > top) top = g[i];
  }
  slotSeed_[b] = static_cast<uint16_t>(top + 1);
#endif
  src_->releaseBucket(buckets_[b].base);
  buckets_[b] = Bucket();
  capacity_ -= extCount_;
}

void PoolCore::releaseEmptyExtensions() {
  bool spare = false;
  for (uint32_t b = 1; b < span_; ++b) {
    if (buckets_[b].base == nullptr || buckets_[b].live != 0) continue;
    if (!spare) {
      spare = true;
      continue;
    }
    releaseBucket(b);
  }
  while (span_ > 1 && buckets_[span_ - 1].base == nullptr) --span_;
}

void PoolCore::bucketMoved(uint16_t bucket, void* newBase) {
  const uintptr_t caller = TYRA_POOL_CALLER();

  if (bucket == 0 || !held(bucket) || newBase == nullptr) {
    reject("pool bucket moved wrongly", bucket, caller);
    return;
  }
#if TYRA_MEMORY_CHECKS
  if (reinterpret_cast<uintptr_t>(newBase) % bucketAlign_ != 0) {
    reject("pool bucket moved wrongly", bucket, caller);
    return;
  }
#endif
  buckets_[bucket].base = static_cast<unsigned char*>(newBase);
}

void* PoolCore::entryAt(uint16_t bucket, uint32_t index) const {
  if (!held(bucket) || index >= entriesIn(bucket)) return nullptr;
  return entryPtr(bucket, index);
}

bool PoolCore::isLive(uint16_t bucket, uint32_t index) const {
  if (!held(bucket) || index >= entriesIn(bucket)) return false;
  return isSet(bucket, index);
}

uint32_t PoolCore::countIn(uint16_t bucket) const {
  return held(bucket) ? entriesIn(bucket) : 0;
}

#if TYRA_MEMORY_CHECKS

Id PoolCore::idOf(const void* entry) const {
  uint32_t i = 0;
  bool exact = false;
  const uint32_t b = locate(entry, i, exact);
  if (b == kNoBucket || !exact) {
    fail(Failure{"pool id of a foreign pointer", name_, typeSize_,
                 TYRA_POOL_CALLER()});
    return Id{kNoPool, 0, 0, 0};
  }
  return Id{id_, static_cast<uint16_t>(b), static_cast<uint16_t>(i),
            generations(b)[i]};
}

bool PoolCore::valid(Id id, const void* raw) const {
  if (id.pool != id_ || !held(id.bucket)) return false;
  if (id.index >= entriesIn(id.bucket)) return false;
  if (!isSet(id.bucket, id.index)) return false;
  if (generations(id.bucket)[id.index] != id.generation) return false;
  return raw == entryPtr(id.bucket, id.index);
}

#endif

}  // namespace Memory
}  // namespace Tyra
