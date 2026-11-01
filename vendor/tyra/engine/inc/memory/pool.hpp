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

// Added by TyraX: pools of linked buckets. A pool holds the entries of ONE
// component type. Its first bucket is part of the fixed memory; when that is
// full the pool takes an extension bucket from a BucketSource (the movable
// heap), up to the cap its registry row gives it. When it can take no more, its
// policy decides: Refuse (acquire answers null), RecycleOldest (the oldest live
// entry is reused) or Halt (a failure, in every build).
//
// PoolCore is the type-erased part: it hands out and takes back raw entry
// memory and knows nothing of the type but its size and alignment. It does not
// construct or destroy anything.
//
// Inside a bucket the registry's layout (Registry::bucketParts) holds: the
// entries, then the live bitmap, then - for RecycleOldest - a stamp per entry
// and - with TYRA_MEMORY_CHECKS - a generation per entry. All of it is inside
// the bucket, so a bucket that is moved to another address (bucketMoved) takes
// its bookkeeping along. Free entries form a list through their first 4 bytes
// (an entry index; 0xFFFFFFFF ends it), so an entry's first 4 bytes are the
// pool's while the entry is free and the caller's while it is live. A write to
// an entry after its release can damage the list: acquire finds a link that is
// out of range or names a live entry (a failure with TYRA_MEMORY_CHECKS;
// without them the list is cut off there), but not one that names another free
// entry.
//
// A pool numbers its buckets from 0 (the first bucket); an extension takes the
// lowest number that is free, up to `maxExtensions`, so a pool has at most
// 1 + maxExtensions <= 64 buckets. One thread only: nothing here locks.
//
// Pool<T>, at the end, is the typed face of a pool the memory manager started:
// it constructs and destroys the objects and hands them out as Ptr<T>.
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#pragma once

#include <cstddef>
#include <cstdint>
#include <new>
#include <utility>

#include "memory/failure.hpp"
#include "memory/registry.hpp"

namespace Tyra {
namespace Memory {

// The most buckets one pool can have: the first and its extensions.
constexpr uint32_t kMaxBuckets = 64;

#if TYRA_MEMORY_CHECKS
// Names one entry of one pool: its bucket, its index in the bucket and the
// generation the entry was at. An entry's generation goes up every time it is
// released (or recycled), so the id of an entry that has been released is
// never valid again - not even when the same memory is handed out afresh.
// Generations are 16 bits and wrap: an id older than 65536 releases of its own
// entry is not caught.
struct Id {
  uint16_t pool, bucket, index, generation;
};
#endif

// Where a pool gets its extension buckets and gives them back: the movable
// heap. `owner` is the bucket's token (bucketToken), by which the heap tells
// the pool which bucket it moved.
class BucketSource {
 public:
  virtual ~BucketSource() = default;

  // `bytes` of memory aligned to `align`, charged to `budget`; null when the
  // budget's cap or the memory is exhausted. A `pinned` bucket must never be
  // moved.
  virtual void* allocateBucket(size_t bytes, size_t align, bool pinned,
                               uint32_t owner, BudgetId budget) = 0;
  // Gives back a bucket, by the address it has now.
  virtual void releaseBucket(void* base) = 0;
};

// The owner token of a bucket: never 0, so 0 can mean "none".
inline uint32_t bucketToken(PoolId pool, uint16_t bucket) {
  return ((uint32_t(pool) + 1) << 16) | bucket;
}

class PoolCore {
 public:
  PoolCore() = default;
  PoolCore(const PoolCore&) = delete;
  PoolCore& operator=(const PoolCore&) = delete;

  // Makes this the pool `id` of `r`, described by `d` (which must be the row
  // `r` holds for `id`), on `firstBucket`: the memory the registry's layout
  // gave its first bucket (r.poolFixedBytes(id) bytes, aligned to 16 or to the
  // type's alignment if more; it may be null when firstCount is 0). Extensions
  // come from `src`, which may be null (the pool then never grows). Forgets
  // whatever the pool held before; the memory is the caller's and is formatted
  // here, whatever it held.
  //
  // Fails (in every build) when `id` is not a pool of `r`, the pool has more
  // than 63 extensions, or it has entries and no first bucket; with
  // TYRA_MEMORY_CHECKS also when `d` is not `r`'s row, the first bucket is
  // misaligned or a bucket has more entries than an Id can number (65535).
  void init(PoolId id, const PoolDesc& d, const Registry& r, void* firstBucket,
            BucketSource* src);

  // Raw memory for one entry (typeSize bytes at the type's alignment), or
  // nullptr when the policy refuses. Takes the lowest-numbered bucket that has
  // a free entry; when every bucket is full, takes an extension from the source
  // (pinned when the budget's kind is Render or the type's alignment is above
  // 16 - such a bucket never moves); when none is possible, the policy:
  //   Refuse         exhaustions() goes up and the answer is nullptr;
  //   RecycleOldest  exhaustions() goes up, the recycle hook is called on the
  //                  live entry acquired longest ago, and that entry is the
  //                  answer (it stays live; it is a new entry now). With no
  //                  live entry at all the answer is nullptr;
  //   Halt           exhaustions() goes up and fail("pool exhausted") - the
  //                  pool is untouched.
  // The memory is not constructed; its first 4 bytes are whatever they were.
  void* acquire();

  // Gives an entry back. The caller has destroyed the object: the entry's first
  // 4 bytes are the pool's from now on. With TYRA_MEMORY_CHECKS a pointer that
  // is not an entry of this pool, not at the start of one, or not live goes to
  // fail; without, such a call is ignored.
  void release(void* entry);

  // What RecycleOldest does to the entry it takes back: `hook(entry, ctx)` runs
  // while the entry still holds the old object (destroy it there), before it is
  // returned. The hook must not call acquire or release on this pool. Null: no
  // hook.
  void setRecycle(void (*hook)(void* entry, void* ctx), void* ctx);

  // Every extension with no live entry goes back to the source, except one
  // spare (the lowest-numbered of them) kept for the next acquire. The first
  // bucket is never given back. Buckets keep their numbers: one released from
  // the middle leaves a gap that a later extension takes.
  void releaseEmptyExtensions();

  // The source moved extension `bucket` (never the first) to `newBase`, the
  // copy of its bytes: from now on the pool finds the bucket there. The bucket
  // carries its bookkeeping with it, so nothing else changes. With
  // TYRA_MEMORY_CHECKS a bucket that is not an extension of this pool or a base
  // that is not aligned goes to fail; without, an unknown bucket is ignored.
  void bucketMoved(uint16_t bucket, void* newBase);

  uint32_t live() const { return live_; }  // entries in use
  // Entries in the buckets held, live or free.
  uint32_t capacity() const { return capacity_; }
  // One past the highest bucket number held: iterate b in [0, bucketCount()).
  // A number below it may be a gap (countIn 0, entryAt null).
  uint32_t bucketCount() const { return span_; }
  // How many times acquire found the pool full with nothing to grow into,
  // whatever the policy did about it.
  uint32_t exhaustions() const { return exhaustions_; }
  const char* name() const { return name_; }

  // Entry `index` of `bucket`: its memory, whether live or free. nullptr for a
  // bucket that is not held or an index past its entries.
  void* entryAt(uint16_t bucket, uint32_t index) const;
  bool isLive(uint16_t bucket, uint32_t index) const;
  // The number of entries `bucket` holds (live or not); 0 for a bucket that is
  // not held.
  uint32_t countIn(uint16_t bucket) const;

#if TYRA_MEMORY_CHECKS
  // The id of an entry of this pool (live or free: it carries the generation
  // the entry is at now). Fails for a pointer that is not at the start of one.
  Id idOf(const void* entry) const;
  // Whether `id` names a live entry of this pool, at the generation it had, and
  // `raw` is that entry's memory.
  bool valid(Id id, const void* raw) const;
#endif

 private:
  static constexpr uint32_t kNil = 0xFFFFFFFFu;
  static constexpr uint32_t kNoBucket = 0xFFFFFFFFu;

  struct Bucket {
    unsigned char* base = nullptr;  // null: no bucket in this slot
    uint32_t live = 0;
    uint32_t freeHead = kNil;  // first free entry; kNil: none
  };

  bool held(uint32_t b) const {
    return b < span_ && (b == 0 || buckets_[b].base != nullptr);
  }
  uint32_t entriesIn(uint32_t b) const {
    return b == 0 ? firstCount_ : extCount_;
  }
  const BucketParts& partsOf(uint32_t b) const {
    return b == 0 ? firstParts_ : extParts_;
  }
  unsigned char* entryPtr(uint32_t b, uint32_t i) const {
    return buckets_[b].base + partsOf(b).entries + size_t(i) * stride_;
  }
  unsigned char* bitmap(uint32_t b) const {
    return buckets_[b].base + partsOf(b).bitmap;
  }
  uint32_t* stamps(uint32_t b) const {
    return reinterpret_cast<uint32_t*>(buckets_[b].base + partsOf(b).stamps);
  }
#if TYRA_MEMORY_CHECKS
  uint16_t* generations(uint32_t b) const {
    return reinterpret_cast<uint16_t*>(buckets_[b].base +
                                       partsOf(b).generations);
  }
#endif
  bool isSet(uint32_t b, uint32_t i) const {
    return (bitmap(b)[i >> 3] >> (i & 7)) & 1;
  }

  uint32_t locate(const void* p, uint32_t& index, bool& exact) const;
  void formatBucket(uint32_t b, void* base);
  void* take(uint32_t b, uintptr_t caller);
  void* grow(uintptr_t caller);
  void* recycleOldest();
  void releaseBucket(uint32_t b);
  void reject(const char* what, size_t size, uintptr_t caller) const;

  const char* name_ = "?";
  BucketSource* src_ = nullptr;
  void (*recycle_)(void*, void*) = nullptr;
  void* recycleCtx_ = nullptr;
  PoolId id_ = kNoPool;
  BudgetId budget_ = 0;
  Policy policy_ = Policy::Refuse;
  bool pinned_ = false;  // extensions are allocated pinned
  uint32_t typeSize_ = 0;
  uint32_t stride_ = 0;
  uint32_t bucketAlign_ = 16;
  uint32_t firstCount_ = 0, extCount_ = 0, maxExt_ = 0;
  BucketParts firstParts_ = {};
  BucketParts extParts_ = {};
  uint32_t span_ = 1;  // bucket numbers 0 .. span_ - 1 may be held
  uint32_t live_ = 0, capacity_ = 0, exhaustions_ = 0;
  uint32_t stamp_ = 0;  // the clock of RecycleOldest: counts acquisitions
  Bucket buckets_[kMaxBuckets];
#if TYRA_MEMORY_CHECKS
  // What a new bucket in a slot starts its generations at: past every
  // generation the slot's previous bucket reached, so an old id stays dead.
  uint16_t slotSeed_[kMaxBuckets] = {};
#endif
};

template <class T>
class Ptr;

namespace detail {

// The started manager's pool `id` (Manager::pool), checked against a type of
// `size` bytes aligned to `align`: fail("pool type does not match its row")
// when the row's typeSize is another size or its entries cannot give the
// alignment (above 16 the row's align must be at least it), and the manager's
// failures for a manager that has not started or an id it does not have.
PoolCore& typedPool(PoolId id, size_t size, size_t align, uintptr_t caller);

}  // namespace detail

// The pool `id` of the started memory manager, holding T. The handle is a
// pointer to the manager's pool: copies share it, and it is never needed to
// keep the pool alive.
//
//   Pool<Bullet> bullets(gameBulletPool);  // the registry's PoolId
//   Ptr<Bullet> b = bullets.acquire(origin, velocity);  // empty on Refuse
//   bullets.release(b);  // destroys it; b is empty after
//
// acquire constructs a T in a free entry with the arguments given; when the
// pool is full and can take no extension its policy decides: Refuse answers an
// empty Ptr, RecycleOldest destroys the oldest live object and constructs the
// new one in its entry (a Pool<T> installs that hook), Halt is a failure.
// Game thread only, like the pool itself.
template <class T>
class Pool {
 public:
  explicit Pool(PoolId id)
      : core_(&detail::typedPool(
            id, sizeof(T), alignof(T),
            reinterpret_cast<uintptr_t>(__builtin_return_address(0)))) {
    core_->setRecycle(&destroy, nullptr);
  }

  template <class... A>
  Ptr<T> acquire(A&&... args) {
    void* raw = core_->acquire();
    if (raw == nullptr) return Ptr<T>();
#if defined(__cpp_exceptions)
    T* object;
    try {
      object = ::new (raw) T(std::forward<A>(args)...);
    } catch (...) {
      core_->release(raw);
      throw;
    }
#else
    T* object = ::new (raw) T(std::forward<A>(args)...);
#endif
    return Ptr<T>(object);
  }

  // Destroys the object, gives its entry back and empties `p`. An empty `p`
  // does nothing. With TYRA_MEMORY_CHECKS a `p` whose object is gone already
  // fails ("ptr to a dead object").
  void release(Ptr<T>& p) {
    T* object = p.get();
    if (object == nullptr) return;
    p.reset();
    object->~T();
    core_->release(object);
  }

  // f(T&) for every live object, bucket by bucket. f must not acquire from or
  // release to this pool.
  template <class F>
  void forEach(F&& f) {
    for (uint32_t b = 0; b < core_->bucketCount(); ++b) {
      const uint16_t bucket = static_cast<uint16_t>(b);
      const uint32_t n = core_->countIn(bucket);
      for (uint32_t i = 0; i < n; ++i) {
        if (core_->isLive(bucket, i)) {
          f(*static_cast<T*>(core_->entryAt(bucket, i)));
        }
      }
    }
  }

  uint32_t live() const { return core_->live(); }

 private:
  static void destroy(void* entry, void*) { static_cast<T*>(entry)->~T(); }

  PoolCore* core_;
};

}  // namespace Memory
}  // namespace Tyra

// Pool<T> hands out Ptr<T>. (ptr.hpp includes this header for Id; whichever
// comes first, both are complete by the end of the include.)
#include "memory/ptr.hpp"
