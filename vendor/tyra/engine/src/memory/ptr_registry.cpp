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

// Added by TyraX: the Ptr table and the functions Ptr<T> calls
// (inc/memory/ptr.hpp). Portable C++ - no PS2 header, no assumption about the
// size of a pointer - so the host verifiers run it as it is.

#include <algorithm>
#include <cstring>

#include "memory/ptr.hpp"

namespace Tyra {
namespace Memory {
namespace {

// The address the public function returns to - the code that called it. A
// macro, so the builtin runs in that function and not in a helper.
#define TYRA_PTR_CALLER() \
  reinterpret_cast<uintptr_t>(__builtin_return_address(0))

constexpr const char* kTableName = "ptr table";

bool isPowerOfTwo(uint32_t v) { return (v & (v - 1)) == 0; }

// The slot an address starts probing at: a 32-bit mix of the whole address, so
// Ptrs laid out at a regular stride do not pile up in a few slots.
uint32_t mix(uintptr_t address) {
  uint32_t x = static_cast<uint32_t>(address) ^
               static_cast<uint32_t>(static_cast<uint64_t>(address) >> 32);
  x ^= x >> 15;
  x *= 0x2c1b3c6dU;
  x ^= x >> 12;
  x *= 0x297a2d39U;
  x ^= x >> 15;
  return x;
}

// Raw storage whose type the table does not know: the Ptr's first word.
// memcpy, so no pointer type is aliased through an integer.
void shiftWordAt(uintptr_t address, uintptr_t delta) {
  void* word = reinterpret_cast<void*>(address);
  uintptr_t v;
  std::memcpy(&v, word, sizeof v);
  v += delta;
  std::memcpy(word, &v, sizeof v);
}

}  // namespace

// ---- PtrTable --------------------------------------------------------------

size_t PtrTable::bytesFor(uint32_t capacity) {
  const uint64_t bytes = 2ull * capacity * sizeof(Entry);
  return static_cast<size_t>(std::min<uint64_t>(bytes, SIZE_MAX));
}

void PtrTable::init(void* storage, uint32_t capacity) {
  live_ = spare_ = nullptr;
  capacity_ = mask_ = size_ = 0;
  if (!isPowerOfTwo(capacity)) {
    fail({"ptr table capacity is not a power of two", kTableName, 0,
          TYRA_PTR_CALLER()});
    return;
  }
  if (capacity == 0) return;
  if (storage == nullptr ||
      reinterpret_cast<uintptr_t>(storage) % alignof(Entry) != 0) {
    fail({"ptr table storage missing or misaligned", kTableName, 0,
          TYRA_PTR_CALLER()});
    return;
  }
  std::memset(storage, 0, bytesFor(capacity));
  live_ = static_cast<Entry*>(storage);
  spare_ = live_ + capacity;
  capacity_ = capacity;
  mask_ = capacity - 1;
}

uint32_t PtrTable::homeOf(uintptr_t address) const {
  return mix(address) & mask_;
}

// The entry for `address`, or null. The loop is bounded by the capacity so a
// miss on a full table ends.
PtrTable::Entry* PtrTable::find(uintptr_t address) const {
  uint32_t i = homeOf(address);
  for (uint32_t n = 0; n < capacity_; ++n) {
    if (live_[i].address == address) return &live_[i];
    if (live_[i].address == 0) return nullptr;
    i = (i + 1) & mask_;
  }
  return nullptr;
}

// Puts (address, token) in `array`, which has an empty slot: true when the
// address was not in it yet.
bool PtrTable::place(Entry* array, uintptr_t address, uint32_t token) const {
  uint32_t i = homeOf(address);
  for (uint32_t n = 0; n < capacity_; ++n) {
    if (array[i].address == address) {
      array[i].token = token;
      return false;
    }
    if (array[i].address == 0) {
      array[i].address = address;
      array[i].token = token;
      return true;
    }
    i = (i + 1) & mask_;
  }
  return false;
}

bool PtrTable::insert(void* ptrAddress, uint32_t token) {
  const uintptr_t address = reinterpret_cast<uintptr_t>(ptrAddress);
  if (address == 0) return false;
  uint32_t i = homeOf(address);
  for (uint32_t n = 0; n < capacity_; ++n) {
    if (live_[i].address == address) {
      live_[i].token = token;
      return true;
    }
    if (live_[i].address == 0) {  // an empty slot: the table is not full
      live_[i].address = address;
      live_[i].token = token;
      ++size_;
      return true;
    }
    i = (i + 1) & mask_;
  }
  return false;  // every slot is taken and none is this address
}

// Empties `slot` and moves back the entries after it that would otherwise be
// cut off from their home (Knuth's algorithm R). On a full table the scan stops
// before it comes round to `slot` again.
void PtrTable::eraseAt(uint32_t slot) {
  uint32_t hole = slot, j = slot;
  for (uint32_t n = 1; n < capacity_; ++n) {
    j = (j + 1) & mask_;
    if (live_[j].address == 0) break;
    const uint32_t home = homeOf(live_[j].address);
    // The entry stays where it is when its home lies after the hole and not
    // past it (cyclically): moving it into the hole would put the hole between
    // its home and it.
    const bool stays =
        hole <= j ? (hole < home && home <= j) : (hole < home || home <= j);
    if (stays) continue;
    live_[hole] = live_[j];
    hole = j;
  }
  live_[hole].address = 0;
  live_[hole].token = 0;
}

void PtrTable::erase(void* ptrAddress) {
  Entry* e = find(reinterpret_cast<uintptr_t>(ptrAddress));
  if (e == nullptr) return;
  eraseAt(static_cast<uint32_t>(e - live_));
  --size_;
}

void PtrTable::rekey(void* from, void* to) {
  if (from == to) return;
  Entry* e = find(reinterpret_cast<uintptr_t>(from));
  if (e == nullptr) return;
  const uint32_t token = e->token;
  eraseAt(static_cast<uint32_t>(e - live_));
  --size_;
  insert(to, token);  // there is room: an entry was just taken out
}

void PtrTable::resolve(const Move* moves, uint32_t count) {
  if (count == 0 || size_ == 0) return;
  std::memset(spare_, 0, capacity_ * sizeof(Entry));
  uint32_t kept = 0;
  for (uint32_t i = 0; i < capacity_; ++i) {
    uintptr_t address = live_[i].address;
    const uint32_t token = live_[i].token;
    if (address == 0) continue;

    // The Ptr itself lives in a bucket that moved.
    for (uint32_t m = 0; m < count; ++m) {
      if (address - moves[m].oldBase < moves[m].bytes) {
        address += moves[m].newBase - moves[m].oldBase;
        break;
      }
    }
    // The Ptr points into a bucket that moved. The Ptr's bytes are at
    // `address` now.
    if (token != 0) {
      for (uint32_t m = 0; m < count; ++m) {
        if (moves[m].token == token) {
          shiftWordAt(address, moves[m].newBase - moves[m].oldBase);
          break;
        }
      }
    }
    if (place(spare_, address, token)) ++kept;
  }
  std::swap(live_, spare_);
  size_ = kept;
}

// In place: erasing a slot may pull the next entry into it, so the slot is
// looked at again before moving on.
void PtrTable::dropBucket(uint32_t token) {
  if (token == 0) return;
  uint32_t i = 0;
  while (i < capacity_) {
    if (live_[i].address != 0 && live_[i].token == token) {
      eraseAt(i);
      --size_;
    } else {
      ++i;
    }
  }
}

// ---- PtrRegistry -----------------------------------------------------------

void PtrRegistry::init(void* storage, uint32_t capacity, uintptr_t lo,
                       uintptr_t hi,
                       uint32_t (*tokenAt)(const void* p, void* ctx),
                       void* ctx) {
  lo_ = hi_ = 0;
  tokenAt_ = tokenAt;
  ctx_ = ctx;
  table_.init(storage, capacity);
  if (lo > hi) {
    fail({"ptr range is reversed", kTableName, 0, TYRA_PTR_CALLER()});
    return;
  }
  lo_ = lo;
  hi_ = hi;
}

// ---- what Ptr<T> calls -----------------------------------------------------

namespace detail {

PtrRegistry* g_ptrs = nullptr;
uintptr_t g_movableLo = 0;
uintptr_t g_movableHi = 0;
PinHook* g_pinHook = nullptr;

void attach(void* self, const void* target) {
  PtrRegistry* r = g_ptrs;
  if (r == nullptr || !r->contains(target)) return;
  if (!r->table().insert(self, r->tokenAt(target))) {
    fail({"ptr table full", kTableName, 0, TYRA_PTR_CALLER()});
  }
}

void detach(void* self) {
  if (g_ptrs != nullptr) g_ptrs->table().erase(self);
}

void moved(void* from, void* to) {
  if (g_ptrs != nullptr) g_ptrs->table().rekey(from, to);
}

void pin(const void* raw) {
  const PinHook* h = g_pinHook;
  if (h != nullptr && h->pin != nullptr) h->pin(raw, h->ctx);
}

void unpin(const void* raw) {
  const PinHook* h = g_pinHook;
  if (h != nullptr && h->unpin != nullptr) h->unpin(raw, h->ctx);
}

#if TYRA_MEMORY_CHECKS
IdHook* g_idHook = nullptr;

bool checkLive(const void* raw, const Id& id) {
  const IdHook* h = g_idHook;
  return h == nullptr || h->checkLive == nullptr ||
         h->checkLive(raw, id, h->ctx);
}

Id idOf(const void* raw) {
  const IdHook* h = g_idHook;
  return h != nullptr && h->idOf != nullptr ? h->idOf(raw, h->ctx) : kEmptyId;
}

void deadPtr() { fail({"ptr to a dead object", "ptr", 0, TYRA_PTR_CALLER()}); }
#endif

}  // namespace detail
}  // namespace Memory
}  // namespace Tyra
