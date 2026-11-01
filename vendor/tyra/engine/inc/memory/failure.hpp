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

// Added by TyraX: the one way the memory manager reports that it cannot do what
// it was asked - a budget that is full, a pool that is exhausted, a region that
// does not fit. fail() prints one line, tells the game's handler, then halts.
//
// This header and everything in src/memory/ outside ps2/ is portable C++: no
// PS2 header, no assumption about the size of a pointer, so the host verifiers
// (tools/verify-memory.py) run the same code the game does.

#pragma once

#include <cstddef>
#include <cstdint>

// 0 (the default): no ids, generations or integrity checks. 1: the manager
// checks what it can and reports through fail().
#ifndef TYRA_MEMORY_CHECKS
#define TYRA_MEMORY_CHECKS 0
#endif

namespace Tyra {
namespace Memory {

struct Failure {
  const char* what;  // the kind of failure, e.g. "budget full"
  const char* name;  // what it happened to: a budget, a pool, a region
  size_t size;       // the request that failed, in bytes (0 when it has none)
  uintptr_t caller;  // the return address of the code that asked, or 0
};

// Where fail() writes and how it stops. `print` receives one line, without a
// newline, and must not allocate from the manager; null means "write the line
// and a newline to stderr". `halt` must not return; null means abort(). The
// engine's PS2 platform installs its own pair; a host verifier installs one
// that throws.
void setFailureOutput(void (*print)(const char* line), void (*halt)());

// The game's handler, called after the line is printed and before the halt
// (draw a message, flush a log). Null removes it. A fail() raised from inside
// the handler skips the handler and goes straight to print and halt.
void setFailureHandler(void (*handler)(const Failure&));

// Formats
//   [MEM] FAIL <what> name=<name> size=<n> caller=0x<hex>
// into a stack buffer (nothing is allocated; a text too long for the line is
// cut), prints it, calls the handler, calls halt.
void fail(const Failure& f);

}  // namespace Memory
}  // namespace Tyra
