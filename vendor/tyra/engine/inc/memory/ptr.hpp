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

// Added by TyraX: Ptr<T>, a smart pointer the memory manager can fix up when it
// moves memory, and the table that lets it.
//
// A Ptr carries the raw pointer, always. With TYRA_MEMORY_CHECKS it also
// carries the id of the pool entry it points at, so a read can be checked; in a
// build without them it is the raw pointer alone, one word.
//
// The manager moves buckets of the movable heap (and nothing else). Every Ptr
// whose target lies in the movable heap is registered, by the address of the
// Ptr itself, in a PtrTable that lives outside the Ptr. After the heap has
// moved some buckets, ONE call to PtrTable::resolve fixes every registered Ptr:
// the ones that point into a moved bucket, and the table's own record of the
// ones that live inside a moved bucket. A Ptr to fixed, routed or static
// memory is not registered - it never needs fixing - and costs one range test
// when it is made, copied or destroyed.
//
// Game thread only: nothing here locks.
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "memory/failure.hpp"
#include "memory/pool.hpp"
#include "memory/registry.hpp"

namespace Tyra {
namespace Memory {

// One bucket the heap moved: the `bytes` that were at `oldBase` are at
// `newBase` now (already copied; the ranges may overlap), and `token` is the
// bucket's owner token (bucketToken). The buckets of one resolve are distinct
// and their old ranges do not overlap.
struct Move {
  uintptr_t oldBase;
  size_t bytes;
  uintptr_t newBase;
  uint32_t token;
};

// The set of registered Ptrs: for each, the address the Ptr lives at and the
// token of the bucket it points into (0 when the target belongs to no bucket).
//
// It is an open-addressing hash set keyed by the Ptr's address - linear
// probing, a backward shift on erase, so there are no tombstones - held in two
// equal arrays of `capacity` entries inside the storage it is given: resolve
// builds the new contents in the idle array and swaps. `capacity` entries are
// the most it holds, but a lookup probes further the fuller the table is, so
// give it headroom.
class PtrTable {
 public:
  // The storage init needs for `capacity` entries: two arrays of
  // `capacity` entries { address, token }. Aligned for a pointer.
  static size_t bytesFor(uint32_t capacity);

  // Makes the table of `capacity` entries (a power of two, or 0 for a table
  // that holds nothing) in `storage`, bytesFor(capacity) bytes aligned for a
  // pointer. Forgets whatever the table held. Fails in every build when the
  // capacity is not a power of two or the storage is missing or misaligned.
  void init(void* storage, uint32_t capacity);

  // Registers the Ptr that lives at `ptrAddress`, pointing into the bucket
  // `token`. A Ptr that is registered already has its token replaced (that
  // takes no room, so it works on a full table). False - and nothing changes -
  // when the table is full or `ptrAddress` is null.
  bool insert(void* ptrAddress, uint32_t token);
  // Forgets the Ptr at `ptrAddress`; an address that is not registered is
  // ignored.
  void erase(void* ptrAddress);
  // The Ptr registered at `from` now lives at `to` (it was moved or copied
  // out): same token, new key. It needs no room, so it cannot fail for a full
  // table. A `from` that is not registered is ignored.
  void rekey(void* from, void* to);

  // The heap has moved `count` buckets and copied their bytes. Fixes the table
  // and the Ptrs in one pass: for every entry (address A, token K)
  //   - A inside a move's old range: the Ptr itself moved, so A shifts by that
  //     move's newBase - oldBase;
  //   - K equal to a move's token: the target moved, so the word the Ptr holds
  //     at A (its raw pointer, read after the first rule) shifts the same way.
  // The entries go into the idle array and the arrays swap, so an entry that
  // changed key is never visited twice. Every Ptr must be inside a live
  // object: the bytes at A are read and written.
  void resolve(const Move* moves, uint32_t count);

  // Forgets every Ptr that points into bucket `token`, for a bucket that is
  // released: the Ptrs left pointing into it are dangling, and must not be
  // patched when another bucket later gets the same token. Token 0 ignored.
  void dropBucket(uint32_t token);

  uint32_t size() const { return size_; }          // Ptrs registered
  uint32_t capacity() const { return capacity_; }  // the most it can hold

  // Calls f(void* ptrAddress, uint32_t token) for every registered Ptr, in no
  // particular order. f must not change the table.
  template <class F>
  void forEach(F f) const {
    for (uint32_t i = 0; i < capacity_; ++i) {
      if (live_[i].address != 0) {
        f(reinterpret_cast<void*>(live_[i].address), live_[i].token);
      }
    }
  }

 private:
  struct Entry {
    uintptr_t address;  // 0: the slot is empty
    uint32_t token;
  };

  uint32_t homeOf(uintptr_t address) const;
  Entry* find(uintptr_t address) const;
  void eraseAt(uint32_t slot);
  bool place(Entry* array, uintptr_t address, uint32_t token) const;

  Entry* live_ = nullptr;   // the array in use
  Entry* spare_ = nullptr;  // the idle one, for resolve
  uint32_t capacity_ = 0;
  uint32_t mask_ = 0;  // capacity_ - 1
  uint32_t size_ = 0;
};

// The manager's side of Ptr: the table, the range of the movable heap and how
// to find the bucket token of an address in it.
class PtrRegistry {
 public:
  // The table in `storage` (PtrTable::init), for Ptrs whose target lies in
  // [lo, hi) - the movable heap. `tokenAt(p, ctx)` answers the owner token of
  // the bucket that holds `p`, or 0 when it holds none. Fails in every build
  // when lo is above hi (and for what PtrTable::init fails for). It does not
  // install itself: the manager does that by setting detail::g_ptrs and the
  // range.
  void init(void* storage, uint32_t capacity, uintptr_t lo, uintptr_t hi,
            uint32_t (*tokenAt)(const void* p, void* ctx), void* ctx);

  PtrTable& table() { return table_; }
  const PtrTable& table() const { return table_; }

  uintptr_t lo() const { return lo_; }
  uintptr_t hi() const { return hi_; }
  bool contains(const void* p) const {
    const uintptr_t a = reinterpret_cast<uintptr_t>(p);
    return a >= lo_ && a < hi_;
  }
  // The token of the bucket that holds `p`; 0 when there is none (or no
  // tokenAt was given).
  uint32_t tokenAt(const void* p) const {
    return tokenAt_ != nullptr ? tokenAt_(p, ctx_) : 0;
  }

 private:
  PtrTable table_;
  uintptr_t lo_ = 0, hi_ = 0;
  uint32_t (*tokenAt_)(const void*, void*) = nullptr;
  void* ctx_ = nullptr;
};

#if TYRA_MEMORY_CHECKS
// What a Ptr that does not point at a pool entry carries (and a Ptr with no
// target): pool kNoPool.
constexpr Id kEmptyId = {kNoPool, 0, 0, 0};
inline bool isEmptyId(const Id& id) { return id.pool == kNoPool; }
#endif

namespace detail {

// Whoever owns the registry sets all three. With none set, no Ptr is ever
// registered: the range is empty.
extern PtrRegistry* g_ptrs;
extern uintptr_t g_movableLo, g_movableHi;

// Whether `p` lies in the movable heap - the one test every Ptr operation pays.
inline bool inMovable(const void* p) {
  const uintptr_t a = reinterpret_cast<uintptr_t>(p);
  return a >= g_movableLo && a < g_movableHi;
}

// Ptr calls these only for a target in the movable heap (inMovable). attach
// registers the Ptr at `self` (replacing an entry it has) with the token of
// `target`'s bucket, and fails with "ptr table full" when there is no room; it
// does nothing for a target outside the registry's range or with no registry.
// detach forgets `self`; moved says the Ptr that was at `from` is at `to`.
void attach(void* self, const void* target);
void detach(void* self);
void moved(void* from, void* to);

// Pinning keeps a bucket in place. The heap that moves buckets installs this
// hook to learn which; with none, pin and unpin do nothing.
struct PinHook {
  void (*pin)(const void* raw, void* ctx);
  void (*unpin)(const void* raw, void* ctx);
  void* ctx;
};
extern PinHook* g_pinHook;
void pin(const void* raw);
void unpin(const void* raw);

#if TYRA_MEMORY_CHECKS
// What a checked Ptr asks of the manager: which entry `raw` is, and whether an
// id still names a live entry at `raw`. With no hook installed checkLive
// answers true and idOf answers kEmptyId. `idOf` should answer kEmptyId for
// memory that is not (the start of) a pool entry.
struct IdHook {
  bool (*checkLive)(const void* raw, const Id& id, void* ctx);
  Id (*idOf)(const void* raw, void* ctx);
  void* ctx;
};
extern IdHook* g_idHook;
bool checkLive(const void* raw, const Id& id);
Id idOf(const void* raw);
// Reports "ptr to a dead object" through fail().
void deadPtr();
#endif

}  // namespace detail

// A pointer to a T that the manager can move.
//
// Copy, move, assignment and destruction keep the table in step, so a Ptr
// behaves like the raw pointer it wraps. It is as large as that pointer without
// TYRA_MEMORY_CHECKS; with them it also carries an id. A Ptr must be a real
// C++ object - constructed, and destroyed before its memory is reused - because
// the table knows it by the address it lives at; copying its bytes by hand
// (memcpy) leaves the copy unknown to the table. The heap's own move of a whole
// bucket is the one exception: resolve fixes up what it moved.
//
// A raw pointer taken out of a Ptr (get(), operator->, operator*) is good until
// the frame ends: the heap only moves memory at the start of a frame. Do not
// keep one across frames; keep the Ptr.
template <class T>
class Ptr {
 public:
  Ptr() = default;
  Ptr(std::nullptr_t) {}

  // With TYRA_MEMORY_CHECKS the Ptr asks the manager which pool entry `raw` is
  // (detail::idOf).
  explicit Ptr(T* raw) : raw_(raw) {
#if TYRA_MEMORY_CHECKS
    if (raw != nullptr) id_ = detail::idOf(raw);
#endif
    if (detail::inMovable(raw_)) detail::attach(this, raw_);
  }

  Ptr(const Ptr& o) : raw_(o.raw_) {
#if TYRA_MEMORY_CHECKS
    id_ = o.id_;
#endif
    if (detail::inMovable(raw_)) detail::attach(this, raw_);
  }

  // A move takes the target and empties `o`. It registers nothing new, so it
  // cannot find the table full.
  Ptr(Ptr&& o) noexcept : raw_(o.raw_) {
#if TYRA_MEMORY_CHECKS
    id_ = o.id_;
    o.id_ = kEmptyId;
#endif
    if (detail::inMovable(raw_)) detail::moved(&o, this);
    o.raw_ = nullptr;
  }

  // If the table is full the failure comes before anything changes.
  Ptr& operator=(const Ptr& o) {
    if (this != &o) {
      if (detail::inMovable(o.raw_)) {
        detail::attach(this, o.raw_);
      } else if (detail::inMovable(raw_)) {
        detail::detach(this);
      }
      raw_ = o.raw_;
#if TYRA_MEMORY_CHECKS
      id_ = o.id_;
#endif
    }
    return *this;
  }

  Ptr& operator=(Ptr&& o) noexcept {
    if (this != &o) {
      if (detail::inMovable(raw_)) detail::detach(this);
      raw_ = o.raw_;
#if TYRA_MEMORY_CHECKS
      id_ = o.id_;
      o.id_ = kEmptyId;
#endif
      if (detail::inMovable(raw_)) detail::moved(&o, this);
      o.raw_ = nullptr;
    }
    return *this;
  }

  ~Ptr() {
    if (detail::inMovable(raw_)) detail::detach(this);
  }

  // The raw pointer. With TYRA_MEMORY_CHECKS, a Ptr that came from a pool
  // checks first that its entry is still live and fails with "ptr to a dead
  // object" when it is not; a Ptr with no id (not from a pool) is not checked.
  T* get() const {
#if TYRA_MEMORY_CHECKS
    if (raw_ != nullptr && !isEmptyId(id_) && !detail::checkLive(raw_, id_)) {
      detail::deadPtr();
    }
#endif
    return raw_;
  }
  T* operator->() const { return get(); }
  typename std::add_lvalue_reference<T>::type operator*() const {
    return *get();
  }
  // Whether it points at anything. Not a liveness check.
  explicit operator bool() const { return raw_ != nullptr; }

  // Points at nothing, and is forgotten by the table.
  void reset() {
    if (detail::inMovable(raw_)) detail::detach(this);
    raw_ = nullptr;
#if TYRA_MEMORY_CHECKS
    id_ = kEmptyId;
#endif
  }

#if TYRA_MEMORY_CHECKS
  // The id of the pool entry it points at; kEmptyId (isEmptyId) when none.
  Id id() const { return id_; }
#endif

 private:
  // The table patches raw_ through the Ptr's address, so it stays the first
  // member.
  T* raw_ = nullptr;
#if TYRA_MEMORY_CHECKS
  Id id_ = kEmptyId;
#endif
};

// Keeps the bucket that holds a Ptr's target where it is, for as long as the
// Pin lives. A Pin of an empty Ptr, or of one that points outside the movable
// heap, does nothing. Pins may nest; a Pin is neither copied nor moved.
template <class T>
class Pin {
 public:
  explicit Pin(const Ptr<T>& p) : raw_(p.get()) {
    if (detail::inMovable(raw_)) {
      detail::pin(raw_);
      pinned_ = true;
    }
  }
  Pin(const Pin&) = delete;
  Pin& operator=(const Pin&) = delete;
  ~Pin() {
    if (pinned_) detail::unpin(raw_);
  }

 private:
  const void* raw_;
  bool pinned_ = false;
};

}  // namespace Memory
}  // namespace Tyra
