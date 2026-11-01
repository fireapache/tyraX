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

// Added by TyraX: the memory manager - the piece that owns every other one. At
// start it takes ONE block from the C library and lays the registry out in it:
// the pools' first buckets, a region per budget that has one, the Ptr table,
// the movable heap and, last, the system region. From then on every routed
// allocation (malloc, new, ... - the platform binding sends them here) is
// served from a region and charged to a budget:
//
//   - before start, to the C library (rawAlloc), as without a manager;
//   - from a thread other than the one that called start, to System;
//   - with no budget scope open, to System;
//   - else to the innermost scope's budget: from its own region when it has
//     one (regionBytes != 0), from System's region - charged to it, uncapped -
//     when it has none (measure mode).
//
// A block's budget is written in its header, so release and reallocate find it
// there; a reallocate stays in its block's budget, wherever it is called from.
// A budget with no room does what its row says (BudgetDesc::onFull): one that
// halts is a failure ("budget full"); one that refuses answers null - render
// data is a request, and its caller keeps what it has, draws something cheaper
// or nothing this frame, and asks again the next - and the refusal is counted
// and reported. A refusing budget in measure mode also refuses what would
// leave System's region under settings.refuseHeadroomBytes.
//
// frameStart runs first thing in a frame: it resets the frame scratch, gives
// the pools' empty extensions back, runs one defragmentation step (which
// resolves the Ptrs) and rolls the per-frame counters.
//
// The manager allocates nothing through the system allocator - everything it
// keeps is in this object or in the block. Region operations run under the
// platform's lock (any thread may allocate); scopes, scratch, pools and Ptrs
// belong to the game thread.
//
// Portable C++, no PS2 header, no assumption about the size of a pointer.

#pragma once

#include <cstddef>
#include <cstdint>

#include "memory/failure.hpp"
#include "memory/movable_heap.hpp"
#include "memory/pool.hpp"
#include "memory/ptr.hpp"
#include "memory/region_heap.hpp"
#include "memory/registry.hpp"

namespace Tyra {
namespace Memory {

// What the manager needs from the machine. None of these may allocate through
// the manager.
struct Platform {
  // The C library's allocator, under the routing: memalign, free and the
  // usable size of one of its blocks.
  void* (*rawAlloc)(size_t align, size_t size);
  void (*rawFree)(void* p);
  size_t (*rawUsable)(void* p);
  // The bytes the C library can still hand out (what the fit check gets).
  size_t (*available)();
  // The running thread.
  uint32_t (*threadId)();
  // Excludes every other thread (on the EE: interrupts off) and answers the
  // state unlock() restores. Not re-entered by the manager.
  uint32_t (*lock)();
  void (*unlock)(uint32_t state);
  // One line, no newline, not allocating; and a halt that does not return.
  // setPlatform makes them fail()'s output too.
  void (*print)(const char* line);
  void (*halt)();
};

class Manager {
 public:
  // The most budget scopes open at once.
  static constexpr uint32_t kMaxScopeDepth = 16;
  // How many unscoped call sites the checks remember (and print once each).
  static constexpr uint32_t kMaxUnscopedSites = 256;
  // How many refusing call sites, and how many assets noteFallback names, the
  // manager remembers to keep each to a line per refusalReportFrames; past
  // that a newcomer takes a remembered one's place.
  static constexpr uint32_t kMaxReportKeys = 64;

  Manager();
  // Takes back the globals it installed (the Ptr registry, the hooks). The
  // block is not given back: a manager lives as long as the program.
  ~Manager();
  Manager(const Manager&) = delete;
  Manager& operator=(const Manager&) = delete;

  // The one the platform binding routes to. Built on first use and never
  // destroyed, so static destructors that free memory still find it.
  static Manager& instance();

  // Before anything else - the first allocation included. Also makes the
  // platform's print and halt fail()'s output (setFailureOutput).
  void setPlatform(const Platform& p);

  // Lays `r` out in one block of the C library and starts routing. The
  // registry is copied; `r` may go away after. Fails (fail(), after printing
  // the table) when the platform is incomplete, the manager was started
  // already, the registry does not fit available(), gives System a
  // regionBytes (its region is sized by settings.systemBytes), has a region
  // below RegionHeap's smallest block, a pool with more than 63 extensions or
  // more extensions in all than MovableHeap::kMaxHeapBuckets, or when the C
  // library refuses the block. True when it started.
  bool start(Registry& r);
  bool started() const { return started_; }

  // malloc, memalign, the nothrow operator new: `size` bytes at `align` (a
  // power of two; below 16 means 16, anything else answers null). `caller` is
  // the return address of the code that asked, for the failure line, the
  // refusal line and the unscoped report. Routed as the header comment says. A
  // budget with no room: one that halts is fail("budget full"); one that
  // refuses answers null - so does one in measure mode that would leave
  // System's region under settings.refuseHeadroomBytes - counts the refusal
  // (BudgetStats::refused) and prints
  //   [MEM] refused name=<budget> size=<n> caller=0x<hex> frame=<f>
  // once per call site per settings.refusalReportFrames frames at most.
  void* allocate(size_t size, size_t align, uintptr_t caller);

  // The throwing operator new / new[]: as allocate, but never null. A refusing
  // budget with no room counts the refusal and is
  // fail("throwing new in a refusing budget") - the call site asks for render
  // memory it cannot do without, a bug to convert to the nothrow new.
  void* allocateOrFail(size_t size, size_t align, uintptr_t caller);

  // free: null is ignored, an address outside the block goes to rawFree (a
  // block from before start, or one the C library made for itself). With
  // TYRA_MEMORY_CHECKS memory of the block that is not a live routed block
  // goes to fail.
  void release(void* p);

  // realloc: same budget as p's block, contents kept up to the smaller size,
  // 16-byte alignment (as the C library's). A shrink keeps the block. A block
  // from before start is copied into the budget an allocate would use now
  // (its size from rawUsable) and given back to the C library. null is
  // allocate; a size of 0 releases p and answers null, as newlib does. No room
  // leaves p as it was and is what the budget does when full: fail("budget
  // full"), or null (a refusal, counted and reported as allocate's).
  void* reallocate(void* p, size_t size, uintptr_t caller);

  // First thing in a frame, on the game thread: resets the frame scratch,
  // gives every pool's empty extensions back (releaseEmptyExtensions), runs
  // the defragmentation step with settings.defragBytesPerFrame, rolls the
  // per-frame counters, prints a line per pool that was exhausted since the
  // last frame and, with TYRA_MEMORY_CHECKS, checks that every registered
  // Ptr's id still names a live entry. Nothing before start.
  void frameStart();

  // When a level begins to load: resets the load scratch.
  void loadStarted();

  // The frames started so far (frameStart calls since start).
  uint32_t frame() const { return frame_; }

  // A render site that did without - a refused request, drawn cheaper or not
  // at all - says so for the log:
  //   [MEM] fallback <what> asset=<asset> lod=<lod> frame=<f>
  // through the platform's print (nothing allocated), once per `key` (the
  // asset: any address that stays the same for it) per
  // settings.refusalReportFrames frames at most. Game thread; nothing before
  // start.
  void noteFallback(const char* what, const char* asset, int lod,
                    const void* key);

  // The game thread's budget scopes (MemScope). A push of an id that is not
  // registered, of a scratch budget or past kMaxScopeDepth, and a pop with
  // none open, are failures. From any other thread both do nothing: other
  // threads are charged to System.
  void pushScope(BudgetId id);
  void popScope();

  // The pool `id` of the registry the manager started with; fail("pool is not
  // in the registry") for any other id.
  PoolCore& pool(PoolId id);

  // Whether p lies in the manager's block.
  bool owns(const void* p) const;
  // The budget whose region holds p - System's region, a budget's own region
  // or a scratch region - or -1 (outside the block, or in a part of it that is
  // not a region: the pools' first buckets, the Ptr table, the movable heap).
  int regionOf(const void* p) const;

  struct BudgetStats {
    const char* name;
    BudgetId id;
    Kind kind;
    // The region's bytes; 0 for a budget with no region (measured, uncapped).
    size_t cap;
    // What its blocks were charged (scratch: the region's cursor) and the most
    // that has been, in bytes.
    size_t used, peak;
    // Summed over the budget's pools.
    uint32_t exhaustions;
    // The requests it refused (OnFull::Refuse) since start.
    uint32_t refused;
  };
  // The registered budgets, by ascending id: i in [0, budgetCount()). 0 before
  // start.
  uint32_t budgetCount() const;
  BudgetStats budgetStats(uint32_t i) const;

  struct Totals {
    size_t fixedBytes;      // Layout::fixedBytes
    size_t systemFree;      // System's region, free
    size_t heapFree;        // the movable heap, free
    size_t heapLargest;     // its largest free block's payload
    size_t movedLastFrame;  // bytes the last frame's step moved
    // The blocks the C library served the program before start that have not
    // come back through release / reallocate (sizes from rawUsable). A block
    // the C library made for itself and the program frees through the manager
    // is taken off too (never below 0), so it can read low.
    size_t newlibBytes;
    uint32_t allocsLastFrame;  // allocate + reallocate calls, every thread
    uint32_t ptrs;             // registered Ptrs
    uint32_t unscopedSites;    // with TYRA_MEMORY_CHECKS; 0 without
  };
  Totals totals() const;

  // The registry's table, then a line per budget
  //   [MEM] budget <name> cap=<KB> used=<KB> peak=<KB> exhausted=<n>
  //   refused=<n>
  // then "[MEM] newlib ..." and "[MEM] heap ..." - through the platform's
  // print, nothing allocated.
  void printTable() const;

  // The copy of the registry the manager started with.
  const Registry& registry() const { return registry_; }

 private:
  struct Span {
    uintptr_t lo, hi;
    BudgetId id;
    bool routed;  // a RegionHeap: System's or a budget's own
  };

  uint32_t lock() const;
  void unlock(uint32_t state) const;
  void print(const char* line) const;
  uint32_t thread() const;
  bool startFails(const Registry& r, const Layout& l, const Failure& f,
                  const char* detail = nullptr);
  void installPtrs();
  const Span* spanOf(const void* p) const;
  const char* nameOf(BudgetId id) const;
  BudgetId route(uintptr_t caller);
  // `nullable`: whether the caller takes a refusal (allocate) or not
  // (allocateOrFail).
  void* serve(BudgetId id, size_t size, size_t align, uintptr_t caller,
              bool count, bool nullable);
  bool refuses(BudgetId id) const;
  bool starves(BudgetId id, const RegionHeap& heap, size_t size) const;
  void refuse(BudgetId id, size_t size, uintptr_t caller);
  // Whether `key` is due a line now (and, if so, remembers that it had one).
  bool reportDue(uintptr_t* keys, uint32_t* frames, uintptr_t key);
  void charge(BudgetId id, size_t bytes);
  void discharge(BudgetId id, size_t bytes);
  void* bootAllocate(size_t size, size_t align);
  void bootRelease(void* p);
  void* bootCopy(void* p, size_t size, uintptr_t caller);
#if TYRA_MEMORY_CHECKS
  void noteUnscoped(uintptr_t caller);
#endif
  void sweepPtrs();
  void reportExhaustions();
  static void onBucketMoved(uint32_t token, void* newBase, void* ctx);
#if TYRA_MEMORY_CHECKS
  bool entryOf(const void* raw, PoolId& pool) const;
  static bool checkLiveHook(const void* raw, const Id& id, void* ctx);
  static Id idOfHook(const void* raw, void* ctx);
#endif

  Platform platform_ = {};
  bool started_ = false;
  uint32_t gameThread_ = 0;
  Registry registry_;
  Layout layout_ = {};
  unsigned char* block_ = nullptr;

  // The regions: a RegionHeap per budget id (System's and those with
  // regionBytes != 0), `home_[id]` the one a budget is served from, `spans_`
  // every region of the block sorted by address.
  RegionHeap regions_[Budget::Max];
  BudgetId home_[Budget::Max] = {};
  Span spans_[Budget::Max] = {};
  uint32_t spanCount_ = 0;
  BudgetId order_[Budget::Max] = {};  // the registered ids, ascending
  uint32_t budgetCount_ = 0;
  size_t used_[Budget::Max] = {};
  size_t peak_[Budget::Max] = {};
  uint32_t refused_[Budget::Max] = {};
  OnFull onFull_[Budget::Max] = {};

  PoolCore pools_[kMaxPools];
  PoolCore none_;  // what pool() answers for an id it refuses
  uint32_t poolCount_ = 0;
  uint32_t seenExhaustions_[kMaxPools] = {};
#if TYRA_MEMORY_CHECKS
  uintptr_t firstLo_[kMaxPools] = {};  // each pool's first-bucket entries
  uintptr_t firstHi_[kMaxPools] = {};
  uint32_t stride_[kMaxPools] = {};
  detail::IdHook idHook_ = {};
#endif

  MovableHeap heap_;
  PtrRegistry ptrs_;

  BudgetId scopes_[kMaxScopeDepth] = {};
  uint32_t depth_ = 0;

  uint32_t allocsThisFrame_ = 0, allocsLastFrame_ = 0;
  size_t movedLastFrame_ = 0;
  size_t newlibBytes_ = 0;
  uint32_t frame_ = 0;

  // The call sites (refusals) and the assets (fallbacks) that had a line, and
  // the frame it was printed in: open addressing on the key, 0 = empty.
  uintptr_t refusalSites_[kMaxReportKeys] = {};
  uint32_t refusalFrames_[kMaxReportKeys] = {};
  uintptr_t fallbackKeys_[kMaxReportKeys] = {};
  uint32_t fallbackFrames_[kMaxReportKeys] = {};

#if TYRA_MEMORY_CHECKS
  uintptr_t sites_[kMaxUnscopedSites] = {};  // open addressing; 0 = empty
  bool siteZero_ = false;                    // caller 0 is a site too
  uint32_t siteCount_ = 0;
  bool reporting_ = false;  // print may allocate: not a site of its own
#endif
};

// Opens a budget scope for its lifetime: every allocation the game thread
// makes meanwhile is charged to `id` (Manager::pushScope / popScope).
class MemScope {
 public:
  explicit MemScope(BudgetId id) { Manager::instance().pushScope(id); }
  ~MemScope() { Manager::instance().popScope(); }
  MemScope(const MemScope&) = delete;
  MemScope& operator=(const MemScope&) = delete;
};

// The game's budgets and pools. With TYRA_MEMORY_MANAGER the engine calls it
// once, first thing in its constructor, on a registry that holds the engine's
// rows, then starts Manager::instance() with it. The engine's own definition
// is weak and empty - the engine's rows as they are: one heap, every budget
// measured - so a game defines this function only to size budgets and add
// its own.
void registerGameBudgets(Registry& r);

#if TYRA_MEMORY_MANAGER
// The EE binding (src/memory/ps2/hooks.cpp). Gives Manager::instance() the
// EE's Platform (which also makes its print and halt fail()'s output) and
// from then on lets the routing hooks through: until it runs, every routed
// call (malloc, free, operator new, ...) goes straight to the C library.
// The engine calls it first thing in its constructor.
void install();

// Copies the stats into the tyra_memory_budgets block a tool reads while the
// game runs (src/memory/ps2/telemetry.cpp), every 10th call. The engine calls
// it right after frameStart.
void publishTelemetry();
#endif

}  // namespace Memory
}  // namespace Tyra
