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

// Added by TyraX: fills the memory manager's telemetry block, version 2
// (inc/memory/telemetry.hpp). src/memory/ps2/telemetry.cpp owns the block on
// the EE and calls this every 10th frame; the verifiers call it on the host.
//
// What it costs the game is bounded: the manager keeps every number as it goes
// (the ring, the free-block counts, the occupancy maps, the largest requests),
// so this copies them, sums each map's 64 cells and walks the movable heap's
// buckets (at most MovableHeap::kMaxHeapBuckets) - it never walks a routed
// region.
//
// The block is ordinary memory, written in order between the two `seq` bumps:
// a signal fence keeps the compiler from moving a store across them, which is
// all a reader on the same machine (an emulator thread, a debugger) needs.
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#include "memory/telemetry.hpp"

#include <atomic>

namespace Tyra {
namespace Memory {
namespace {

// Sizes are 32-bit on the EE: a field holds any of them. On a 64-bit host a
// bigger one saturates instead of wrapping.
uint32_t word(size_t bytes) {
  return bytes > 0xFFFFFFFFu ? 0xFFFFFFFFu : static_cast<uint32_t>(bytes);
}

void fence() { std::atomic_signal_fence(std::memory_order_seq_cst); }

}  // namespace

void writeTelemetry(TelemetryBlock& b, Manager& m, uint32_t frame) {
  if (!m.started()) return;
  const Manager::Totals t = m.totals();
  const MovableHeap::Health h = m.heapHealth();
  const uint32_t budgets = m.budgetCount() < kTelemetryBudgetRows
                               ? m.budgetCount()
                               : kTelemetryBudgetRows;
  const uint32_t regions = m.regionCount() < kTelemetryRegionRows
                               ? m.regionCount()
                               : kTelemetryRegionRows;

  b.seq = b.seq + 1;
  fence();
  b.count = budgets;
  b.frame = frame;
  b.systemFree = word(t.systemFree);
  b.heapFree = word(t.heapFree);
  b.heapLargest = word(t.heapLargest);
  b.movedLastFrame = word(t.movedLastFrame);
  b.allocsLastFrame = t.allocsLastFrame;
  b.ptrs = t.ptrs;
  b.version = kTelemetryVersion;
  b.blockBytes = sizeof(TelemetryBlock);
  b.defragBytesPerFrame = m.registry().settings.defragBytesPerFrame;
  b.largestRequest = word(m.largestRequest());
  b.ringHead = m.frame();
  b.regionCount = regions;
  b.heapDebt = word(h.debt);
  b.heapBlocked = word(h.blocked);
  b.heapPinned = word(h.pinned);
  b.heapOversize = h.oversize;
  b.heapOversizeBytes = word(h.oversizeBytes);
  b.heapBuckets = h.buckets;
  b.heapFramesToCompact = h.framesToCompact;

  for (uint32_t i = 0; i < budgets; ++i) {
    const Manager::BudgetStats s = m.budgetStats(i);
    TelemetryBudgetRow& row = b.budgets[i];
    const char* name = s.name != nullptr ? s.name : "";
    uint32_t c = 0;
    for (; c < sizeof row.name && name[c] != '\0'; ++c) row.name[c] = name[c];
    for (; c < sizeof row.name; ++c) row.name[c] = '\0';
    row.id = s.id;
    row.kind = static_cast<uint8_t>(s.kind);
    row.onFull = static_cast<uint8_t>(s.onFull);
    row.cap = word(s.cap);
    row.used = word(s.used);
    row.peak = word(s.peak);
    row.exhaustions = s.exhaustions;
    row.refused = s.refused;
  }

  const Manager::DefragFrame* ring = m.defragRing();
  for (uint32_t i = 0; i < kTelemetryRingFrames; ++i) {
    b.ring[i].bytes = ring[i].bytes;
    b.ring[i].buckets = ring[i].buckets;
    b.ring[i].oversize = ring[i].oversize;
  }

  for (uint32_t i = 0; i < regions; ++i) {
    const Manager::RegionStats s = m.regionStats(i);
    TelemetryRegionRow& row = b.regions[i];
    row.id = s.id;
    row.cells = static_cast<uint16_t>(s.cells);
    row.capacity = word(s.capacity);
    row.freeBytes = word(s.freeBytes);
    row.largestFree = word(s.largestFree);
    row.freeBlocks = s.freeBlocks;
    row.largestRequest = word(s.largestRequest);
    for (uint32_t c = 0; c < sizeof row.map; ++c) row.map[c] = s.map[c];
  }
  fence();
  b.seq = b.seq + 1;

  m.resetLargestRequests();
}

}  // namespace Memory
}  // namespace Tyra
