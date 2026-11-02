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

// Added by TyraX: the memory manager's telemetry block on the EE
// (docs/memory-manager.md, "The telemetry block"). `tyra_memory_budgets` is a
// fixed-layout block in .data - its magic, version and size are in the ELF's
// file image, so a tool finds its address without symbols - that
// publishTelemetry() rewrites every 10th frame (writeTelemetry, the layout in
// inc/memory/telemetry.hpp), for a tool that reads EE RAM while the game runs
// (an emulator's PINE, a debugger). A seqlock guards it: `seq` is odd while the
// block is being rewritten; a reader takes a copy whose `seq` is even and the
// same before and after.
//
// Without TYRA_MEMORY_MANAGER this file is empty.

#if TYRA_MEMORY_MANAGER

#include <cstdint>

#include "memory/manager.hpp"
#include "memory/telemetry.hpp"

// Outside any namespace and C linkage: the name a debugger and a person type.
// Initialised, so it is in .data and its magic is in the ELF file.
extern "C" {
__attribute__((used)) Tyra::Memory::TelemetryBlock tyra_memory_budgets = {
    {'T', 'Y', 'R', 'A', 'M', 'E', 'M', '3'},
    0, 0, 0, 0, 0, 0, 0, 0, 0,             // seq .. ptrs
    Tyra::Memory::kTelemetryVersion,       // version
    sizeof(Tyra::Memory::TelemetryBlock),  // blockBytes
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,       // defragBytesPerFrame .. reserved
    {}, {}, {}};
}

namespace Tyra {
namespace Memory {
namespace {

constexpr uint32_t kPublishFrames = 10;

uint32_t g_calls = 0;

}  // namespace

void publishTelemetry() {
  const uint32_t call = g_calls++;
  if (call % kPublishFrames != 0) return;
  writeTelemetry(tyra_memory_budgets, Manager::instance(), call + 1);
}

}  // namespace Memory
}  // namespace Tyra

#endif  // TYRA_MEMORY_MANAGER
