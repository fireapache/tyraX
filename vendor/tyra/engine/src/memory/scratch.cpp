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

// Added by TyraX: the per-frame scratch (inc/memory/scratch.hpp). Portable C++
// - no PS2 header, no assumption about the size of a pointer - so the host
// verifiers run it as it is.

#include "memory/scratch.hpp"

namespace Tyra {
namespace Scratch {
namespace {

// Constant-initialised: nothing runs before main, so no static-initialisation
// order can hide a region that is already in use.
Region g_frame;
Region g_load;

}  // namespace

Region& frame() { return g_frame; }
Region& load() { return g_load; }

void Region::init(void* base, size_t bytes) {
  base_ = static_cast<unsigned char*>(base);
  capacity_ = base != nullptr ? bytes : 0;
  cursor_ = 0;
  peak_ = 0;
  lastBlock_ = 0;
  lastBefore_ = 0;
  lastSize_ = 0;
  ++epoch_;
}

void* Region::allocate(size_t size, size_t align) {
  if (size == 0) size = 1;
  if (align == 0) align = 1;
  if ((align & (align - 1)) != 0) {
#if TYRA_MEMORY_CHECKS
    Memory::fail({"scratch alignment is not a power of two", "scratch", size,
                  reinterpret_cast<uintptr_t>(__builtin_return_address(0))});
#endif
    return nullptr;
  }

  // Align the address, not the offset: the range itself may start anywhere.
  const uintptr_t start = reinterpret_cast<uintptr_t>(base_) + cursor_;
  const size_t pad =
      static_cast<size_t>(((start + (align - 1)) & ~(align - 1)) - start);
  const size_t room = capacity_ - cursor_;
  if (pad > room || size > room - pad) return nullptr;

  lastBefore_ = cursor_;
  lastBlock_ = cursor_ + pad;
  lastSize_ = size;
  cursor_ = lastBlock_ + size;
  if (cursor_ > peak_) peak_ = cursor_;
  return base_ + lastBlock_;
}

void Region::deallocate(void* p, size_t size) {
  if (size == 0) size = 1;
  if (lastSize_ != 0 && size == lastSize_ && p == base_ + lastBlock_) {
    cursor_ = lastBefore_;
    lastSize_ = 0;
  }
}

void Region::reset() {
  cursor_ = 0;
  lastSize_ = 0;
  ++epoch_;
}

namespace detail {

void* allocateBlock(Region& region, const char* name, size_t bytes,
                    size_t align, uint32_t epoch, uintptr_t caller) {
#if TYRA_MEMORY_CHECKS
  if (epoch != region.epoch()) {
    Memory::fail({"scratch kept past its frame", name, bytes, caller});
    return nullptr;
  }
#else
  (void)epoch;
#endif
  void* p = region.allocate(bytes, align);
  if (p == nullptr) Memory::fail({"scratch full", name, bytes, caller});
  return p;
}

void releaseBlock(Region& region, const char* name, void* p, size_t bytes,
                  uint32_t epoch, uintptr_t caller) {
#if TYRA_MEMORY_CHECKS
  if (epoch != region.epoch()) {
    Memory::fail({"scratch kept past its frame", name, bytes, caller});
    return;
  }
#else
  (void)epoch;
  (void)name;
  (void)caller;
#endif
  region.deallocate(p, bytes);
}

}  // namespace detail

}  // namespace Scratch
}  // namespace Tyra
