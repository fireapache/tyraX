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

// Added by TyraX: the memory manager's EE binding (docs/memory-manager.md).
//
// With TYRA_MEMORY_MANAGER, vendor/tyra/Makefile.base links with the linker's
// --wrap for malloc, free, calloc, realloc, memalign, aligned_alloc,
// posix_memalign, operator new / new[] (_Znwj, _Znaj) and their nothrow forms
// (_ZnwjRKSt9nothrow_t, _ZnajRKSt9nothrow_t), so every call of those in the
// link - the engine, the game, libstdc++, ps2sdk - lands in the __wrap_
// function of the same name below, and __real_<name> is the C library's own.
// newlib's internal allocations (_malloc_r: stdio's buffers, strdup) are not
// calls of those names and stay newlib's.
//
// The C functions and the nothrow new take a refusal: a budget that refuses
// (BudgetDesc::onFull) answers them null. The throwing new cannot - it is
// Manager::allocateOrFail, which fails instead. The nothrow new must be wrapped
// itself: libstdc++'s calls the throwing one inside a try, which would halt.
//
// Until install() has run, every hook goes straight to the C library: static
// constructors and libstdc++'s start-up allocate before the engine exists, and
// so may the first call of Manager::instance() (its guard), so the switch is a
// plain global that install() sets last.
//
// Also here: the EE Platform the manager runs on, and the weak, empty
// registerGameBudgets a game replaces with its own.
//
// Without TYRA_MEMORY_MANAGER this file is empty.

#if TYRA_MEMORY_MANAGER

#include <kernel.h>
#include <malloc.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>

#include "info/info.hpp"
#include "memory/manager.hpp"

extern "C" {
void* __real_malloc(size_t size);
void __real_free(void* p);
void* __real_calloc(size_t count, size_t size);
void* __real_realloc(void* p, size_t size);
void* __real_memalign(size_t align, size_t size);
void* __real_aligned_alloc(size_t align, size_t size);
void* __real__Znwj(size_t size);
void* __real__Znaj(size_t size);
void* __real__ZnwjRKSt9nothrow_t(size_t size, const std::nothrow_t& tag);
void* __real__ZnajRKSt9nothrow_t(size_t size, const std::nothrow_t& tag);
}

namespace Tyra {
namespace Memory {
namespace {

// What malloc and new give: the C library's 16 (RegionHeap::kMinAlign).
constexpr size_t kAlign = 16;

// The manager once install() has run, else null. A plain global: a hook asks
// nothing before install() that could allocate.
Manager* g_manager = nullptr;

// The thread inside eePrint, or -1. Its allocations go to the C library, so
// writing a line - fopen allocates - never re-enters the manager, not even for
// the line that says System is full.
volatile s32 g_printThread = -1;

// Where eePrint appends, decided once by install(); empty for stdout. The same
// rule as TyraDebug::writeInLogFile: never on a read-only medium, where opening
// cdrom0:LOG.TXT;1 for write wedges the CDVD driver.
char g_logPath[256];

inline uintptr_t callerOf(void* returnAddress) {
  return reinterpret_cast<uintptr_t>(returnAddress);
}

// Whether the manager serves this call: once installed, and not from inside
// eePrint.
inline bool routed() {
  return g_manager != nullptr &&
         (g_printThread < 0 || GetThreadId() != g_printThread);
}

// Whether `p` is the manager's whoever frees it - a block it served stays its
// own while eePrint runs too.
inline bool managerOwns(const void* p) {
  return g_manager != nullptr && g_manager->owns(p);
}

// ---- the Platform ----------------------------------------------------------

void* eeRawAlloc(size_t align, size_t size) {
  return __real_memalign(align, size);
}

void eeRawFree(void* p) { __real_free(p); }

size_t eeRawUsable(void* p) { return malloc_usable_size(p); }

// What the C library can still hand out: the free bytes inside the arena it
// has (mallinfo's fordblks, its top chunk included) and everything sbrk can
// still give below EndOfHeap() - the two reads Info::getFreeRAMSize makes.
// Neither allocates.
size_t eeAvailable() {
  const struct mallinfo mi = mallinfo();
  const uintptr_t brk = reinterpret_cast<uintptr_t>(sbrk(0));
  const uintptr_t top = reinterpret_cast<uintptr_t>(EndOfHeap());
  return static_cast<size_t>(mi.fordblks) + (top > brk ? top - brk : 0);
}

uint32_t eeThreadId() { return static_cast<uint32_t>(GetThreadId()); }

// Interrupts off: no other thread runs and no handler cuts in. DIntr answers
// whether they were on, so a lock taken with them off leaves them off.
uint32_t eeLock() { return static_cast<uint32_t>(DIntr()); }

void eeUnlock(uint32_t wasOn) {
  if (wasOn != 0) EIntr();
}

// One line, to the log file when the engine writes one (Info::writeLogsToFile)
// and it can be opened, else to stdout - flushed, as TyraDebug::emit does.
void eePrint(const char* line) {
  g_printThread = GetThreadId();
  bool written = false;
  if (Info::writeLogsToFile && g_logPath[0] != '\0') {
    FILE* f = fopen(g_logPath, "a");
    if (f != nullptr) {
      fputs(line, f);
      fputs("\n", f);
      fclose(f);
      written = true;
    }
  }
  if (!written) {
    printf("%s\n", line);
    fflush(stdout);
  }
  g_printThread = -1;
}

void eeHalt() { __builtin_trap(); }

// The log file's path, as FileUtils::fromCwd("log.txt") makes it.
void computeLogPath() {
  g_logPath[0] = '\0';
  char cwd[224];
  if (getcwd(cwd, sizeof cwd) == nullptr) return;
  if (strncmp(cwd, "cdrom", 5) == 0) return;  // read-only medium
  const size_t n = strlen(cwd);
  const char last = n != 0 ? cwd[n - 1] : '\0';
  const bool separated = n == 0 || last == '/' || last == '\\' || last == ':';
  snprintf(g_logPath, sizeof g_logPath, "%s%slog.txt", cwd,
           separated ? "" : "/");
}

// The throwing operator new's block: never null (a budget with no room is a
// failure, which halts - in a refusing budget too).
void* newBlock(size_t size, uintptr_t caller) {
  void* p = g_manager->allocateOrFail(size != 0 ? size : 1, kAlign, caller);
  if (p == nullptr) {
    fail(Failure{"operator new got no memory", "new", size, caller});
  }
  return p;
}

}  // namespace

__attribute__((weak)) void registerGameBudgets(Registry&) {}

void install() {
  if (g_manager != nullptr) return;
  computeLogPath();
  // Its first call builds the manager, and the guard of that may allocate:
  // still the C library's, since g_manager is null.
  Manager& m = Manager::instance();
  Platform p = {};
  p.rawAlloc = &eeRawAlloc;
  p.rawFree = &eeRawFree;
  p.rawUsable = &eeRawUsable;
  p.available = &eeAvailable;
  p.threadId = &eeThreadId;
  p.lock = &eeLock;
  p.unlock = &eeUnlock;
  p.print = &eePrint;
  p.halt = &eeHalt;
  m.setPlatform(p);
  g_manager = &m;  // last: from here on the hooks route
}

}  // namespace Memory
}  // namespace Tyra

// ---- the hooks -------------------------------------------------------------

using Tyra::Memory::callerOf;
using Tyra::Memory::g_manager;
using Tyra::Memory::kAlign;
using Tyra::Memory::managerOwns;
using Tyra::Memory::routed;

extern "C" {

void* __wrap_malloc(size_t size) {
  if (!routed()) return __real_malloc(size);
  return g_manager->allocate(size, kAlign,
                             callerOf(__builtin_return_address(0)));
}

void __wrap_free(void* p) {
  if (routed() || managerOwns(p)) {
    g_manager->release(p);
  } else {
    __real_free(p);
  }
}

void* __wrap_calloc(size_t count, size_t size) {
  if (!routed()) return __real_calloc(count, size);
  if (size != 0 && count > SIZE_MAX / size) {
    errno = ENOMEM;
    return nullptr;
  }
  const size_t bytes = count * size;
  void* p = g_manager->allocate(bytes, kAlign,
                                callerOf(__builtin_return_address(0)));
  if (p != nullptr) memset(p, 0, bytes);
  return p;
}

void* __wrap_realloc(void* p, size_t size) {
  if (routed() || managerOwns(p)) {
    return g_manager->reallocate(p, size,
                                 callerOf(__builtin_return_address(0)));
  }
  return __real_realloc(p, size);
}

void* __wrap_memalign(size_t align, size_t size) {
  if (!routed()) return __real_memalign(align, size);
  return g_manager->allocate(size, align,
                             callerOf(__builtin_return_address(0)));
}

void* __wrap_aligned_alloc(size_t align, size_t size) {
  if (!routed()) return __real_aligned_alloc(align, size);
  return g_manager->allocate(size, align,
                             callerOf(__builtin_return_address(0)));
}

// newlib has no posix_memalign, so before install() it is memalign.
int __wrap_posix_memalign(void** out, size_t align, size_t size) {
  if (align == 0 || (align & (align - 1)) != 0 || align % sizeof(void*) != 0) {
    return EINVAL;
  }
  void* p = routed() ? g_manager->allocate(
                           size, align, callerOf(__builtin_return_address(0)))
                     : __real_memalign(align, size);
  if (p == nullptr) return ENOMEM;
  *out = p;
  return 0;
}

// operator new / new[] (unsigned int: the EE's size_t). The caller is the code
// that said `new`, not libstdc++'s operator new.
void* __wrap__Znwj(size_t size) {
  if (!routed()) return __real__Znwj(size);
  return Tyra::Memory::newBlock(size, callerOf(__builtin_return_address(0)));
}

void* __wrap__Znaj(size_t size) {
  if (!routed()) return __real__Znaj(size);
  return Tyra::Memory::newBlock(size, callerOf(__builtin_return_address(0)));
}

// new (std::nothrow): null when the budget refuses, as the standard says.
void* __wrap__ZnwjRKSt9nothrow_t(size_t size, const std::nothrow_t& tag) {
  if (!routed()) return __real__ZnwjRKSt9nothrow_t(size, tag);
  return g_manager->allocate(size != 0 ? size : 1, kAlign,
                             callerOf(__builtin_return_address(0)));
}

void* __wrap__ZnajRKSt9nothrow_t(size_t size, const std::nothrow_t& tag) {
  if (!routed()) return __real__ZnajRKSt9nothrow_t(size, tag);
  return g_manager->allocate(size != 0 ? size : 1, kAlign,
                             callerOf(__builtin_return_address(0)));
}

}  // extern "C"

#endif  // TYRA_MEMORY_MANAGER
