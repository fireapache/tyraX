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

// Added by TyraX: the layout of the memory manager's telemetry block, version 2
// (docs/memory-manager.md, "The telemetry block"), and the function that fills
// one from a Manager. On the PS2 the block is `tyra_memory_budgets`
// (src/memory/ps2/telemetry.cpp), rewritten every 10th frame for a tool that
// reads EE RAM while the game runs (an emulator's PINE, a debugger): the
// totals, a row per budget, the last 64 frames' defragmentation, a row per
// region with its fragmentation and its occupancy map, and the movable heap's
// health. A seqlock guards it: `seq` is odd while the block is being rewritten;
// a reader keeps a copy whose `seq` is even and the same before and after.
//
// Every field is a little-endian integer of a fixed width, so the layout is the
// same on the EE and on a 64-bit host (the verifiers check it there).
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#pragma once

#include <cstddef>
#include <cstdint>

#include "memory/manager.hpp"

namespace Tyra {
namespace Memory {

// "TYRAMEM3": version 2 of the layout. "TYRAMEM2" is version 1 (a 48-byte
// header and 48 rows of 32 bytes, 1584 bytes); "TYRAMEM1" came before it.
constexpr uint32_t kTelemetryVersion = 2;
constexpr uint32_t kTelemetryBudgetRows = 48;
constexpr uint32_t kTelemetryRingFrames = Manager::kDefragRingFrames;
constexpr uint32_t kTelemetryRegionRows = Manager::kMaxRegionRows;

struct TelemetryBudgetRow {
  char name[12];  // the budget's name, cut to 12 characters; zero-padded when
                  // shorter, no terminator when it fills them
  uint16_t id;    // its BudgetId
  uint8_t kind;   // Kind
  uint8_t onFull; // OnFull: 0 halts, 1 refuses
  uint32_t cap;   // bytes: its region's, 0 for a budget with none (measured)
  uint32_t used;
  uint32_t peak;
  uint32_t exhaustions;  // summed over its pools
  uint32_t refused;      // requests it answered null
};

struct TelemetryDefragFrame {  // Manager::DefragFrame
  uint32_t bytes;     // block bytes the frame's step moved
  uint16_t buckets;   // buckets it moved
  uint16_t oversize;  // of them, bigger than defragBytesPerFrame (moved alone)
};

struct TelemetryRegionRow {  // Manager::RegionStats
  uint16_t id;     // the budget whose region it is; 0xFFFF the movable heap
  uint16_t cells;  // the cells of `map` that cover the region
  uint32_t capacity;
  uint32_t freeBytes;
  uint32_t largestFree;     // the largest allocation it can serve now
  uint32_t freeBlocks;      // the pieces its free bytes are in
  uint32_t largestRequest;  // the largest request it got since the last publish
  uint8_t map[64];          // used fraction per cell: 0 empty .. 255 full
};

struct TelemetryBlock {
  char magic[8];  // "TYRAMEM3", no terminator
  uint32_t seq;   // odd while the block is being rewritten
  uint32_t count; // budget rows filled, in budget id order
  uint32_t frame; // publishes so far, the one that wrote the block included
  uint32_t systemFree;
  uint32_t heapFree;
  uint32_t heapLargest;
  uint32_t movedLastFrame;
  uint32_t allocsLastFrame;
  uint32_t ptrs;
  // Version 2 from here on (version 1 had one reserved word, 0, here).
  uint32_t version;     // kTelemetryVersion
  uint32_t blockBytes;  // sizeof(TelemetryBlock)
  uint32_t defragBytesPerFrame;
  uint32_t largestRequest;  // the largest allocate size since the last publish
  uint32_t ringHead;        // the newest frame in `ring` (Manager::frame())
  uint32_t regionCount;     // region rows filled
  uint32_t heapDebt;        // MovableHeap::Health
  uint32_t heapBlocked;
  uint32_t heapPinned;
  uint32_t heapOversize;       // oversize buckets that still have to move
  uint32_t heapOversizeBytes;
  uint32_t heapBuckets;
  uint32_t heapFramesToCompact;  // steps left; 0xFFFFFFFF never (budget 0)
  TelemetryBudgetRow budgets[kTelemetryBudgetRows];
  // Frame f is at [f % kTelemetryRingFrames]: (ringHead - 64, ringHead].
  TelemetryDefragFrame ring[kTelemetryRingFrames];
  TelemetryRegionRow regions[kTelemetryRegionRows];
};

static_assert(sizeof(TelemetryBudgetRow) == 36, "a budget row is 36 bytes");
static_assert(sizeof(TelemetryDefragFrame) == 8, "a ring entry is 8 bytes");
static_assert(sizeof(TelemetryRegionRow) == 88, "a region row is 88 bytes");
static_assert(offsetof(TelemetryBlock, version) == 44,
              "version 1's reserved word becomes the version");
static_assert(offsetof(TelemetryBlock, budgets) == 96,
              "the header is 96 bytes");
static_assert(offsetof(TelemetryBlock, ring) == 1824,
              "the ring follows 48 budget rows");
static_assert(offsetof(TelemetryBlock, regions) == 2336,
              "the region rows follow 64 ring entries");
static_assert(sizeof(TelemetryBlock) == 3744,
              "the block is 3744 bytes (docs/memory-manager.md)");
static_assert(sizeof(Manager::DefragFrame) == sizeof(TelemetryDefragFrame),
              "the ring is copied entry by entry");

// Rewrites `b` from `m` under the seqlock - `frame` goes in the header - then
// starts the next window of the largest requests (resetLargestRequests).
// Nothing before start. Game thread: it walks the movable heap's buckets.
void writeTelemetry(TelemetryBlock& b, Manager& m, uint32_t frame);

}  // namespace Memory
}  // namespace Tyra
