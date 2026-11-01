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

// Added by TyraX: the memory manager's telemetry block (docs/memory-manager.md,
// "The telemetry block"). `tyra_memory_budgets` is a fixed-layout block in .data
// - its magic is in the ELF's file image, so a tool finds its address without
// symbols - that publishTelemetry() rewrites every 10th frame with the
// manager's totals and one row per budget, for a tool that reads EE RAM while
// the game runs (an emulator's PINE, a debugger). A seqlock guards it: `seq` is
// odd while the block is being rewritten; a reader takes a copy whose `seq` is
// even and the same before and after.
//
// Without TYRA_MEMORY_MANAGER this file is empty.

#if TYRA_MEMORY_MANAGER

#include <tamtypes.h>

#include <cstddef>
#include <cstdint>

#include "memory/manager.hpp"

// Outside any namespace and C linkage: the name a debugger and a person type.
struct TyraMemoryBudgetRow {
  char name[12];  // the budget's name, cut to 12 characters; zero-padded when
                  // shorter, no terminator when it fills them
  u32 cap;        // bytes: the region's, 0 for a budget with none
  u32 used;
  u32 peak;
  u32 exhaustions;  // summed over the budget's pools
  u32 refused;      // requests a refusing budget answered null
};

struct TyraMemoryBudgets {
  char magic[8];  // "TYRAMEM2", no terminator
  u32 seq;        // odd while the block is being rewritten
  u32 count;      // rows filled, in budget id order
  u32 frame;      // publishTelemetry calls so far (frames), this one included
  u32 systemFree;
  u32 heapFree;
  u32 heapLargest;
  u32 movedLastFrame;
  u32 allocsLastFrame;
  u32 ptrs;
  u32 reserved[1];
  TyraMemoryBudgetRow rows[48];
};

static_assert(sizeof(TyraMemoryBudgetRow) == 32, "a row is 32 bytes");
static_assert(offsetof(TyraMemoryBudgets, rows) == 48,
              "the header is 48 bytes");
static_assert(sizeof(TyraMemoryBudgets) == 1584,
              "the block is 1584 bytes (docs/memory-manager.md)");

// Initialised, so it is in .data and its magic is in the ELF file.
extern "C" {
__attribute__((used)) volatile TyraMemoryBudgets tyra_memory_budgets = {
    {'T', 'Y', 'R', 'A', 'M', 'E', 'M', '2'}};
}

namespace Tyra {
namespace Memory {
namespace {

constexpr u32 kPublishFrames = 10;
constexpr u32 kRows = sizeof(tyra_memory_budgets.rows) /
                      sizeof(tyra_memory_budgets.rows[0]);
constexpr u32 kNameBytes = sizeof(tyra_memory_budgets.rows[0].name);

u32 g_calls = 0;

void barrier() { asm volatile("" : : : "memory"); }

// Sizes are 32-bit on the EE: a field holds any of them.
u32 word(size_t bytes) { return static_cast<u32>(bytes); }

}  // namespace

void publishTelemetry() {
  const u32 call = g_calls++;
  if (call % kPublishFrames != 0) return;
  const Manager& m = Manager::instance();
  if (!m.started()) return;
  const Manager::Totals t = m.totals();
  const u32 budgets = m.budgetCount();
  const u32 count = budgets < kRows ? budgets : kRows;

  volatile TyraMemoryBudgets& b = tyra_memory_budgets;
  b.seq = b.seq + 1;
  barrier();
  b.count = count;
  b.frame = call + 1;
  b.systemFree = word(t.systemFree);
  b.heapFree = word(t.heapFree);
  b.heapLargest = word(t.heapLargest);
  b.movedLastFrame = word(t.movedLastFrame);
  b.allocsLastFrame = t.allocsLastFrame;
  b.ptrs = t.ptrs;
  for (u32 i = 0; i < count; ++i) {
    const Manager::BudgetStats s = m.budgetStats(i);
    volatile TyraMemoryBudgetRow& row = b.rows[i];
    const char* name = s.name != nullptr ? s.name : "";
    u32 c = 0;
    for (; c < kNameBytes && name[c] != '\0'; ++c) row.name[c] = name[c];
    for (; c < kNameBytes; ++c) row.name[c] = '\0';
    row.cap = word(s.cap);
    row.used = word(s.used);
    row.peak = word(s.peak);
    row.exhaustions = s.exhaustions;
    row.refused = s.refused;
  }
  barrier();
  b.seq = b.seq + 1;
}

}  // namespace Memory
}  // namespace Tyra

#endif  // TYRA_MEMORY_MANAGER
