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

// Added by TyraX: the arena-pair policy of a patched baked stream
// (docs/baked-vif-stream.md). An entry that is patched owns TWO arenas,
// `current` (what the frame's REFs name) and `spare`; a patch writes the spare
// and swaps them. This header is the part of that which needs no PS2 header,
// so tools/verify-stapip-arena-pair.cpp can model the whole entry sequence
// against the real ImmutableSpanTable on a host.
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <utility>

namespace Tyra {

// A baked stream's storage: `raw` is the allocation, `data` is `raw` rounded up
// to 16 bytes (what a REF names). A null `raw` is a REFUSED arena - the heap had
// no block of that size - and is false.
struct StaPipArena {
  std::unique_ptr<uint8_t[]> raw;
  void* data = nullptr;
  uint32_t qwords = 0;

  StaPipArena() = default;
  // A moved-from arena is empty: no `data` or `qwords` left to be read by mistake.
  StaPipArena(StaPipArena&& other) noexcept
      : raw(std::move(other.raw)), data(other.data), qwords(other.qwords) {
    other.data = nullptr;
    other.qwords = 0;
  }
  StaPipArena& operator=(StaPipArena&& other) noexcept {
    if (this != &other) {
      raw = std::move(other.raw);
      data = other.data;
      qwords = other.qwords;
      other.data = nullptr;
      other.qwords = 0;
    }
    return *this;
  }

  explicit operator bool() const { return raw != nullptr; }
};

// `qwords` quadwords, 16-byte aligned. Taken with new (std::nothrow): a short or
// fragmented heap REFUSES the arena (an empty StaPipArena) rather than halting
// the game, and the caller keeps what it had. Zero quadwords, or a size that
// does not fit size_t, is refused too.
inline StaPipArena makeStaPipArena(uint32_t qwords) {
  StaPipArena arena;
  if (qwords == 0 || qwords > (SIZE_MAX - 15) / 16) return arena;
  arena.raw.reset(new (std::nothrow) uint8_t[static_cast<size_t>(qwords) * 16 + 15]);
  if (!arena.raw) return arena;
  arena.data = reinterpret_cast<void*>(
      (reinterpret_cast<uintptr_t>(arena.raw.get()) + 15u) & ~uintptr_t(15));
  arena.qwords = qwords;
  return arena;
}

// What a patch does with the entry's spare arena.
enum class StaPipSpareUse : uint8_t {
  Reuse,     // write it: same size, and no recorded frame still reads it
  Allocate,  // the entry has no spare yet: make one (once in the entry's life)
  Replace,   // a spare a recorded frame still reads, or of another size: make a fresh arena
};

// `spareCollected` is Vif1Queue::collectRetiredImmutableSpan(spare.data): true
// once no bank reads the spare, and it REMOVES the span from the registry. So
// the caller asks it only when the sizes match (a spare it will not reuse keeps
// its span until the graveyard collects it).
constexpr StaPipSpareUse chooseSpareUse(bool hasSpare, uint32_t spareQwords,
                                        uint32_t wantQwords,
                                        bool spareCollected) {
  return !hasSpare ? StaPipSpareUse::Allocate
         : (spareQwords == wantQwords && spareCollected)
             ? StaPipSpareUse::Reuse
             : StaPipSpareUse::Replace;
}

}  // namespace Tyra
