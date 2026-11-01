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

// Added by TyraX: the per-frame scratch. A Region is a bump allocator over one
// byte range: allocate() moves a cursor forward, reset() puts it back to the
// start and nothing is erased, so the memory is simply reused by the next
// frame. frame() is the region reset every frame, load() the one reset when a
// level is loaded. Allocator<T> / LoadAllocator<T> are std allocators over
// them, and the aliases below put the std containers on them:
//
//   Tyra::Scratch::vector<int> v;       // instead of std::vector<int>
//
// A scratch container must be gone (or at least never touched again) by the
// time its region is reset. With TYRA_MEMORY_CHECKS an allocator remembers the
// reset count (the epoch) it was made in, and every allocate / deallocate in
// another epoch reports "scratch kept past its frame". Without checks the
// allocator is an empty class and nothing is checked.
//
// A full region is reported through Memory::fail ("scratch full"), never by
// returning null from an allocator. One thread only - the one that runs the
// frame: nothing here locks.
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "memory/failure.hpp"

namespace Tyra {
namespace Scratch {

// A bump allocator over the range given to init(). Not thread-safe.
class Region {
 public:
  constexpr Region() = default;

  // Starts over on [base, base + bytes): cursor 0, peak 0, a new epoch. The
  // range is the caller's and is never touched by init. A region that was
  // never given a range has no room: every allocation answers null.
  void init(void* base, size_t bytes);

  // `size` bytes aligned to `align` (a power of two; the alignment is of the
  // address, whatever the range's own). nullptr when the rest of the range is
  // too small - and then nothing has changed. The padding that the alignment
  // needs counts as used. A request of 0 bytes is served as 1.
  void* allocate(size_t size, size_t align);

  // Gives a block back. The cursor moves back, over the block and the padding
  // before it, only when `p` is the block allocated last and `size` is the
  // size it was allocated with; any other block stays used until reset().
  void deallocate(void* p, size_t size);

  // Cursor to 0 and the epoch up by one. Nothing is erased: the bytes stay as
  // they were until something is allocated over them.
  void reset();

  size_t used() const { return cursor_; }  // the cursor, padding included
  size_t peak() const { return peak_; }  // the most used() has been since init
  size_t capacity() const { return capacity_; }
  uint32_t epoch() const { return epoch_; }  // counts init() and reset() calls

 private:
  unsigned char* base_ = nullptr;
  size_t capacity_ = 0;
  size_t cursor_ = 0;
  size_t peak_ = 0;
  // The block allocated last, for deallocate(): where it starts, the cursor it
  // started from (before its alignment padding) and its size. 0: none.
  size_t lastBlock_ = 0;
  size_t lastBefore_ = 0;
  size_t lastSize_ = 0;
  uint32_t epoch_ = 0;
};

// The two regions. Nothing initialises them until the memory manager starts
// (or a verifier calls init itself); until then they have no room.
Region& frame();  // reset at the start of every frame
Region& load();   // reset when a level begins to load

// The least alignment a container's allocator asks the region for: what
// std::allocator gives (the default new alignment, 16 on the EE), so that
// swapping std::vector for Scratch::vector never hands a byte buffer an odd
// address - a file read into a Load::vector<uint8_t> and cast to a header
// struct must not trap. A type that wants more (alignof(T) > 16) gets that.
// Region::allocate itself honours exactly the alignment it is given.
constexpr size_t kMinContainerAlign = 16;

namespace detail {

// What the allocators call: the checks and the failures live here, once, not in
// every instantiation. `epoch` is only compared with TYRA_MEMORY_CHECKS.
void* allocateBlock(Region& region, const char* name, size_t bytes,
                    size_t align, uint32_t epoch, uintptr_t caller);
void releaseBlock(Region& region, const char* name, void* p, size_t bytes,
                  uint32_t epoch, uintptr_t caller);

struct FrameTag {
  static Region& region() { return frame(); }
  static const char* name() { return "frame"; }
};

struct LoadTag {
  static Region& region() { return load(); }
  static const char* name() { return "load"; }
};

// The epoch an allocator was made in: kept with TYRA_MEMORY_CHECKS, an empty
// class (so the allocator is one too) without.
#if TYRA_MEMORY_CHECKS
class EpochStamp {
 public:
  explicit EpochStamp(const Region& r) noexcept : epoch_(r.epoch()) {}
  uint32_t value() const noexcept { return epoch_; }

 private:
  uint32_t epoch_;
};
#else
class EpochStamp {
 public:
  explicit EpochStamp(const Region&) noexcept {}
  uint32_t value() const noexcept { return 0; }
};
#endif

template <class T, class Tag>
class RegionAllocator : private EpochStamp {
 public:
  using value_type = T;
  using size_type = size_t;
  using difference_type = std::ptrdiff_t;
#if !TYRA_MEMORY_CHECKS
  using is_always_equal = std::true_type;
#endif

  template <class U>
  struct rebind {
    using other = RegionAllocator<U, Tag>;
  };

  // Made in the region's current epoch.
  RegionAllocator() noexcept : EpochStamp(Tag::region()) {}
  RegionAllocator(const RegionAllocator&) noexcept = default;
  RegionAllocator& operator=(const RegionAllocator&) noexcept = default;

  // A container rebinds its allocator for its nodes and keeps the epoch.
  template <class U>
  RegionAllocator(const RegionAllocator<U, Tag>& other) noexcept
      : EpochStamp(static_cast<const EpochStamp&>(other)) {}

  // What every block of this allocator is aligned to, at least.
  static constexpr size_t kAlign = alignof(T) > kMinContainerAlign
                                       ? alignof(T)
                                       : kMinContainerAlign;

  T* allocate(size_t n) {
    // The return address is the code that asked, as far as the call is not
    // inlined away; it goes to the failure line.
    const uintptr_t caller =
        reinterpret_cast<uintptr_t>(__builtin_return_address(0));
    if (n > static_cast<size_t>(-1) / sizeof(T)) {
      Memory::fail(
          {"scratch full", Tag::name(), static_cast<size_t>(-1), caller});
      return nullptr;
    }
    return static_cast<T*>(allocateBlock(
        Tag::region(), Tag::name(), n * sizeof(T), kAlign, value(), caller));
  }

  void deallocate(T* p, size_t n) {
    const uintptr_t caller =
        reinterpret_cast<uintptr_t>(__builtin_return_address(0));
    releaseBlock(Tag::region(), Tag::name(), p, n * sizeof(T), value(), caller);
  }

  // Equal when the memory of one can be given back through the other: always
  // without checks, the same epoch with them.
  template <class U>
  bool operator==(const RegionAllocator<U, Tag>& other) const noexcept {
    return value() == static_cast<const EpochStamp&>(other).value();
  }
  template <class U>
  bool operator!=(const RegionAllocator<U, Tag>& other) const noexcept {
    return !(*this == other);
  }

 private:
  template <class U, class W>
  friend class RegionAllocator;
};

}  // namespace detail

// The std allocator over frame() / load().
template <class T>
using Allocator = detail::RegionAllocator<T, detail::FrameTag>;
template <class T>
using LoadAllocator = detail::RegionAllocator<T, detail::LoadTag>;

// The std containers on the frame scratch: swap `std::vector` for
// `Tyra::Scratch::vector`.
template <class T>
using vector = std::vector<T, Allocator<T>>;
template <class T>
using deque = std::deque<T, Allocator<T>>;
using string = std::basic_string<char, std::char_traits<char>, Allocator<char>>;
using stringstream =
    std::basic_stringstream<char, std::char_traits<char>, Allocator<char>>;
template <class K, class V, class C = std::less<K>>
using map = std::map<K, V, C, Allocator<std::pair<const K, V>>>;
template <class K, class V, class H = std::hash<K>, class E = std::equal_to<K>>
using unordered_map =
    std::unordered_map<K, V, H, E, Allocator<std::pair<const K, V>>>;

// The same on the load scratch.
namespace Load {

template <class T>
using vector = std::vector<T, LoadAllocator<T>>;
using string =
    std::basic_string<char, std::char_traits<char>, LoadAllocator<char>>;

}  // namespace Load

}  // namespace Scratch
}  // namespace Tyra
