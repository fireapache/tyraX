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

// Added by TyraX: the memory manager (inc/memory/manager.hpp). Portable C++ -
// no PS2 header, no assumption about the size of a pointer - so the host
// verifiers run it as it is.
//
// Accounting. A routed block's header holds the budget it is charged to and,
// as its owner, the bytes it was charged: what the region's used() grew by
// when it was allocated. That is the block (header and payload) and, for an
// over-aligned block whose 16-byte gap joined the block in front of it, those
// 16 bytes too. A release takes back exactly what its block was charged, so a
// budget's count never drifts, whichever blocks share a region and in
// whichever order they go. (A budget that owns its region could read the
// region's used(); every budget is counted the one way instead.)
//
// Nothing here allocates: the lines are formatted into stack buffers, the
// state is in the Manager and in the block.

#include "memory/manager.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <new>

#include "memory/scratch.hpp"

namespace Tyra {
namespace Memory {
namespace {

// The address the public function returns to - the code that called the
// manager. A macro, so the builtin runs in that function and not in a helper.
#define TYRA_MANAGER_CALLER() \
  reinterpret_cast<uintptr_t>(__builtin_return_address(0))

constexpr const char* kName = "manager";
constexpr const char* kScopeName = "budget scope";
constexpr size_t kAlign = RegionHeap::kMinAlign;
// RegionHeap's smallest block: a region below it can hold nothing.
constexpr size_t kMinRegion = 2 * RegionHeap::kHeader;
constexpr size_t kLineBytes = 224;

inline uintptr_t addressOf(const void* p) {
  return reinterpret_cast<uintptr_t>(p);
}

size_t roundDown16(size_t bytes) { return bytes & ~(kAlign - 1); }

// What the table prints for a size: whole KB, rounded up so a size that is not
// zero never shows as 0 (as Registry::printTable does).
unsigned long kb(size_t bytes) {
  return static_cast<unsigned long>((static_cast<uint64_t>(bytes) + 1023) /
                                    1024);
}

const char* policyName(Policy p) {
  static const char* const names[] = {"Refuse", "RecycleOldest", "Halt"};
  const unsigned i = static_cast<unsigned>(p);
  return i < sizeof names / sizeof names[0] ? names[i] : "?";
}

// The two budgets whose regions are the scratch regions, not region heaps.
bool isScratch(BudgetId id) {
  return id == Budget::FrameScratch || id == Budget::LoadScratch;
}

// Lowercase hex digits of v, no leading zeros, into `out` (17 bytes at least):
// an address prints the same on the EE and on a 64-bit host.
void hexText(uint64_t v, char* out) {
  char digits[16];
  size_t n = 0;
  do {
    digits[n++] = "0123456789abcdef"[v & 15];
    v >>= 4;
  } while (v != 0);
  for (size_t i = 0; i < n; ++i) out[i] = digits[n - 1 - i];
  out[n] = '\0';
}

// The slot a call site (or a report key) starts probing at.
uint32_t siteHome(uintptr_t caller) {
  uint32_t x = static_cast<uint32_t>(caller) ^
               static_cast<uint32_t>(static_cast<uint64_t>(caller) >> 32);
  x ^= x >> 16;
  x *= 0x45d9f3bU;
  x ^= x >> 16;
  return x;
}

}  // namespace

// ---- life ------------------------------------------------------------------

Manager::Manager() = default;

Manager::~Manager() {
  if (detail::g_ptrs == &ptrs_) {
    detail::g_ptrs = nullptr;
    detail::g_movableLo = detail::g_movableHi = 0;
  }
#if TYRA_MEMORY_CHECKS
  if (detail::g_idHook == &idHook_) detail::g_idHook = nullptr;
#endif
}

Manager& Manager::instance() {
  // Placement-new into static storage: never destroyed, and building it
  // allocates nothing - its first call may come from inside malloc.
  alignas(Manager) static unsigned char storage[sizeof(Manager)];
  static Manager* const manager = ::new (storage) Manager();
  return *manager;
}

void Manager::setPlatform(const Platform& p) {
  platform_ = p;
  setFailureOutput(p.print, p.halt);
}

uint32_t Manager::lock() const {
  return platform_.lock != nullptr ? platform_.lock() : 0;
}

void Manager::unlock(uint32_t state) const {
  if (platform_.unlock != nullptr) platform_.unlock(state);
}

void Manager::print(const char* line) const {
  if (platform_.print != nullptr) platform_.print(line);
}

uint32_t Manager::thread() const {
  return platform_.threadId != nullptr ? platform_.threadId() : 0;
}

const char* Manager::nameOf(BudgetId id) const {
  const BudgetDesc* d = id < Budget::Max ? registry_.findBudget(id) : nullptr;
  return d != nullptr && d->name != nullptr ? d->name : "?";
}

// ---- start -----------------------------------------------------------------

// The table of `r` laid out as `l`, an optional line of detail, the failure.
bool Manager::startFails(const Registry& r, const Layout& l, const Failure& f,
                         const char* detail) {
  r.printTable(platform_.print, l);
  if (detail != nullptr) print(detail);
  fail(f);
  return false;
}

bool Manager::start(Registry& r) {
  const uintptr_t caller = TYRA_MANAGER_CALLER();
  const Platform& p = platform_;
  if (p.rawAlloc == nullptr || p.rawFree == nullptr || p.rawUsable == nullptr ||
      p.available == nullptr || p.threadId == nullptr || p.lock == nullptr ||
      p.unlock == nullptr) {
    fail(Failure{"manager has no platform (setPlatform)", kName, 0, caller});
    return false;
  }
  if (started_) {
    fail(Failure{"manager started twice", kName, 0, caller});
    return false;
  }

  // Everything that can be refused is refused before the block is taken.
  Layout l;
  if (!r.fits(p.available(), l)) {
    char line[kLineBytes];
    std::snprintf(line, sizeof line, "[MEM] %s", r.error());
    return startFails(
        r, l, Failure{"registry does not fit", kName, l.fixedBytes, caller},
        line);
  }
  const BudgetDesc* system = r.findBudget(Budget::System);
  if (system != nullptr && system->regionBytes != 0) {
    return startFails(
        r, l,
        Failure{"system budget has a regionBytes: size it with "
                "settings.systemBytes",
                nameOf(Budget::System), system->regionBytes, caller});
  }
  if (roundDown16(l.systemBytes) < kMinRegion) {
    return startFails(
        r, l,
        Failure{"budget region too small", "System", l.systemBytes, caller});
  }
  for (BudgetId id = 0; id < Budget::Max; ++id) {
    const BudgetDesc* d = r.findBudget(id);
    if (d == nullptr || id == Budget::System || isScratch(id) ||
        d->regionBytes == 0 || roundDown16(d->regionBytes) >= kMinRegion) {
      continue;
    }
    return startFails(
        r, l,
        Failure{"budget region too small", d->name, d->regionBytes, caller});
  }
  uint32_t extensions = 0;
  for (uint32_t i = 0; i < r.poolCount(); ++i) {
    const PoolDesc& d = r.poolDesc(static_cast<PoolId>(i));
    if (d.extensionCount == 0) continue;
    if (d.maxExtensions >= kMaxBuckets) {
      return startFails(r, l,
                        Failure{"pool has too many extensions", d.name,
                                d.maxExtensions, caller});
    }
    extensions += d.maxExtensions;
  }
  if (extensions > MovableHeap::kMaxHeapBuckets) {
    return startFails(r, l,
                      Failure{"pools take more extensions than the movable "
                              "heap has buckets",
                              kName, extensions, caller});
  }
  for (uint32_t i = 0; i < depth_; ++i) {  // scopes opened before start
    if (r.findBudget(scopes_[i]) == nullptr) {
      return startFails(r, l,
                        Failure{"scope on an unknown budget", kScopeName,
                                scopes_[i], caller});
    }
  }

  void* block = p.rawAlloc(l.blockAlign, l.blockBytes);
  if (block == nullptr) {
    return startFails(
        r, l,
        Failure{"registry does not fit", "the C library refused the block",
                l.blockBytes, caller});
  }

  registry_ = r;
  layout_ = l;
  block_ = static_cast<unsigned char*>(block);

  // The regions, in address order: the budgets' by id, then System's.
  uint32_t caps[Budget::Max] = {};
  spanCount_ = 0;
  budgetCount_ = 0;
  Scratch::frame().init(nullptr, 0);
  Scratch::load().init(nullptr, 0);
  for (BudgetId id = 0; id < Budget::Max; ++id) {
    used_[id] = peak_[id] = 0;
    refused_[id] = 0;
    onFull_[id] = OnFull::Halt;
    home_[id] = Budget::System;
    const BudgetDesc* d = registry_.findBudget(id);
    if (d == nullptr) continue;
    onFull_[id] = d->onFull;
    order_[budgetCount_++] = id;
    caps[id] = d->extensionBytes;
    if (id == Budget::System || d->regionBytes == 0) continue;
    unsigned char* base = block_ + l.regionOffset[id];
    size_t bytes = d->regionBytes;
    if (isScratch(id)) {
      (id == Budget::FrameScratch ? Scratch::frame() : Scratch::load())
          .init(base, bytes);
    } else {
      bytes = roundDown16(bytes);
      regions_[id].init(base, bytes);
      home_[id] = id;
    }
    spans_[spanCount_++] =
        Span{addressOf(base), addressOf(base) + bytes, id, !isScratch(id)};
  }
  unsigned char* systemBase = block_ + l.systemOffset;
  const size_t systemBytes = roundDown16(l.systemBytes);
  regions_[Budget::System].init(systemBase, systemBytes);
  spans_[spanCount_++] =
      Span{addressOf(systemBase), addressOf(systemBase) + systemBytes,
           Budget::System, true};

  // The movable heap and the Ptr registry over exactly its range.
  unsigned char* heapBase = block_ + l.heapOffset;
  ptrs_.init(block_ + l.ptrOffset, registry_.settings.ptrTableCapacity,
             addressOf(heapBase), addressOf(heapBase) + l.heapBytes,
             &MovableHeap::tokenAtCallback, &heap_);
  heap_.init(heapBase, l.heapBytes, caps, &ptrs_);
  heap_.setMoveListener(&onBucketMoved, this);

  // The pools, on their first buckets.
  poolCount_ = registry_.poolCount();
  for (uint32_t i = 0; i < poolCount_; ++i) {
    const PoolId id = static_cast<PoolId>(i);
    const PoolDesc& d = registry_.poolDesc(id);
    unsigned char* first = block_ + l.poolOffset[i];
    pools_[i].init(id, d, registry_, first, &heap_);
    seenExhaustions_[i] = 0;
#if TYRA_MEMORY_CHECKS
    stride_[i] = registry_.bucketParts(id, d.firstCount).stride;
    firstLo_[i] = addressOf(first);
    firstHi_[i] = firstLo_[i] + size_t(d.firstCount) * stride_[i];
#endif
  }

  heap_.installPinHook();
  installPtrs();
#if TYRA_MEMORY_CHECKS
  idHook_ = detail::IdHook{&checkLiveHook, &idOfHook, this};
  detail::g_idHook = &idHook_;
#endif

  gameThread_ = p.threadId();
  allocsThisFrame_ = allocsLastFrame_ = 0;
  movedLastFrame_ = 0;
  frame_ = 0;
  std::fill(refusalSites_, refusalSites_ + kMaxReportKeys, uintptr_t(0));
  std::fill(fallbackKeys_, fallbackKeys_ + kMaxReportKeys, uintptr_t(0));
  started_ = true;
  return true;
}

// The globals every Ptr reads, set in one place from the registry's own range,
// so the range a Ptr tests and the range the table serves cannot disagree.
void Manager::installPtrs() {
  if (ptrs_.lo() == 0) {
    fail(Failure{"ptr range starts at address 0", kName, layout_.heapBytes, 0});
    return;
  }
  detail::g_ptrs = &ptrs_;
  detail::g_movableLo = ptrs_.lo();
  detail::g_movableHi = ptrs_.hi();
}

// ---- where an address is ---------------------------------------------------

bool Manager::owns(const void* p) const {
  const uintptr_t a = addressOf(p), lo = addressOf(block_);
  return block_ != nullptr && a >= lo && a - lo < layout_.blockBytes;
}

const Manager::Span* Manager::spanOf(const void* p) const {
  const uintptr_t a = addressOf(p);
  uint32_t lo = 0, hi = spanCount_;  // the spans are in address order
  while (lo < hi) {
    const uint32_t mid = lo + (hi - lo) / 2;
    if (spans_[mid].lo <= a) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  if (lo == 0 || a >= spans_[lo - 1].hi) return nullptr;
  return &spans_[lo - 1];
}

int Manager::regionOf(const void* p) const {
  if (!owns(p)) return -1;
  const Span* s = spanOf(p);
  return s != nullptr ? static_cast<int>(s->id) : -1;
}

// ---- routing ---------------------------------------------------------------

BudgetId Manager::route(uintptr_t caller) {
  if (thread() != gameThread_) return Budget::System;
  if (depth_ != 0) return scopes_[depth_ - 1];
#if TYRA_MEMORY_CHECKS
  noteUnscoped(caller);
#else
  (void)caller;
#endif
  return Budget::System;
}

void Manager::charge(BudgetId id, size_t bytes) {
  if (id >= Budget::Max) return;
  used_[id] += bytes;
  if (used_[id] > peak_[id]) peak_[id] = used_[id];
}

void Manager::discharge(BudgetId id, size_t bytes) {
  if (id >= Budget::Max) return;
  used_[id] -= std::min(bytes, used_[id]);
}

bool Manager::refuses(BudgetId id) const {
  return id < Budget::Max && onFull_[id] == OnFull::Refuse;
}

// Whether a request of `size` for `id`, served from `heap`, is refused before
// it is tried: a refusing budget in measure mode (served from System's region)
// leaves System its headroom. Under the lock.
bool Manager::starves(BudgetId id, const RegionHeap& heap, size_t size) const {
  if (!refuses(id) || id == Budget::System || home_[id] != Budget::System) {
    return false;
  }
  const size_t free = heap.freeBytes();
  return size > free || free - size < registry_.settings.refuseHeadroomBytes;
}

// A block for `id` from the region it is served from. No room: a halting
// budget fails; a refusing one counts the refusal, then answers null when the
// caller takes that (`nullable`) and fails when it cannot.
void* Manager::serve(BudgetId id, size_t size, size_t align, uintptr_t caller,
                     bool count, bool nullable) {
  RegionHeap& heap = regions_[home_[id]];
  const bool refusing = refuses(id);
  const uint32_t state = lock();
  if (count) ++allocsThisFrame_;
  void* p = nullptr;
  if (!starves(id, heap, size)) {
    const size_t before = heap.used();
    p = heap.allocate(size, align, id, 0);
    if (p != nullptr) {
      const size_t charged = heap.used() - before;
      heap.setOwner(p, static_cast<uint32_t>(charged));
      charge(id, charged);
    }
  }
  if (p == nullptr && refusing) ++refused_[id];
  unlock(state);
  if (p != nullptr) return p;
  if (!refusing) {
    fail(Failure{"budget full", nameOf(id), size, caller});
  } else if (!nullable) {
    fail(Failure{"throwing new in a refusing budget", nameOf(id), size,
                 caller});
  } else {
    refuse(id, size, caller);
  }
  return nullptr;
}

// The line for a refusal: once per call site per refusalReportFrames.
void Manager::refuse(BudgetId id, size_t size, uintptr_t caller) {
  const uint32_t state = lock();
  const bool due = reportDue(refusalSites_, refusalFrames_, caller);
  unlock(state);
  if (!due) return;
  char hex[17];
  char line[kLineBytes];
  hexText(caller, hex);
  std::snprintf(line, sizeof line,
                "[MEM] refused name=%s size=%lu caller=0x%s frame=%lu",
                nameOf(id), static_cast<unsigned long>(size), hex,
                static_cast<unsigned long>(frame_));
  print(line);
}

// Open addressing on the key (kept as key + 1, so 0 is a key too); when every
// slot holds another key the newcomer takes its home slot. Under the lock.
bool Manager::reportDue(uintptr_t* keys, uint32_t* frames, uintptr_t key) {
  const uintptr_t k = key + 1 != 0 ? key + 1 : 1;
  const uint32_t interval = registry_.settings.refusalReportFrames;
  const uint32_t home = siteHome(k) & (kMaxReportKeys - 1);
  uint32_t i = home;
  for (uint32_t n = 0; n < kMaxReportKeys; ++n) {
    if (keys[i] == k) {
      if (frame_ - frames[i] < interval) return false;
      frames[i] = frame_;
      return true;
    }
    if (keys[i] == 0) break;
    i = (i + 1) & (kMaxReportKeys - 1);
  }
  if (keys[i] != 0) i = home;  // full
  keys[i] = k;
  frames[i] = frame_;
  return true;
}

void* Manager::allocate(size_t size, size_t align, uintptr_t caller) {
  if (align < kAlign) align = kAlign;
  if ((align & (align - 1)) != 0) return nullptr;
  if (!started_) return bootAllocate(size, align);
  return serve(route(caller), size, align, caller, true, true);
}

void* Manager::allocateOrFail(size_t size, size_t align, uintptr_t caller) {
  if (align < kAlign) align = kAlign;
  if ((align & (align - 1)) != 0) return nullptr;
  if (!started_) return bootAllocate(size, align);
  return serve(route(caller), size, align, caller, true, false);
}

void Manager::release(void* p) {
  if (p == nullptr) return;
  if (!owns(p)) {
    bootRelease(p);
    return;
  }
#if TYRA_MEMORY_CHECKS
  const uintptr_t caller = TYRA_MANAGER_CALLER();
#endif
  const Span* s = spanOf(p);
  if (s == nullptr || !s->routed) {
#if TYRA_MEMORY_CHECKS
    fail(Failure{"free of manager memory that is not a routed block",
                 s != nullptr ? nameOf(s->id) : kName, 0, caller});
#endif
    return;
  }
  RegionHeap& heap = regions_[s->id];
  const uint32_t state = lock();
#if TYRA_MEMORY_CHECKS
  if (!heap.owns(p)) {
    unlock(state);
    fail(Failure{"free of a block that is not live", nameOf(s->id), 0, caller});
    return;
  }
#endif
  discharge(heap.budgetOf(p), heap.ownerOf(p));
  heap.release(p);
  unlock(state);
}

void* Manager::reallocate(void* p, size_t size, uintptr_t caller) {
  if (p == nullptr) return allocate(size, kAlign, caller);
  if (started_) {
    const uint32_t state = lock();
    ++allocsThisFrame_;
    unlock(state);
  }
  if (size == 0) {
    release(p);
    return nullptr;
  }
  if (!owns(p)) return bootCopy(p, size, caller);

  const Span* s = spanOf(p);
  if (s == nullptr || !s->routed) {
#if TYRA_MEMORY_CHECKS
    fail(Failure{"realloc of manager memory that is not a routed block",
                 s != nullptr ? nameOf(s->id) : kName, size, caller});
#endif
    return nullptr;
  }
  RegionHeap& heap = regions_[s->id];
  uint32_t state = lock();
#if TYRA_MEMORY_CHECKS
  if (!heap.owns(p)) {
    unlock(state);
    fail(Failure{"realloc of a block that is not live", nameOf(s->id), size,
                 caller});
    return nullptr;
  }
#endif
  // The block's own budget, in the region it is in: a realloc never moves a
  // block to another budget, wherever it is called from.
  const BudgetId id = heap.budgetOf(p);
  const size_t have = heap.usable(p);
  if (size <= have) {
    unlock(state);
    return p;
  }
  void* q = nullptr;
  if (!starves(id, heap, size)) {
    const size_t before = heap.used();
    q = heap.allocate(size, kAlign, id, 0);
    if (q != nullptr) {
      const size_t charged = heap.used() - before;
      heap.setOwner(q, static_cast<uint32_t>(charged));
      charge(id, charged);
    }
  }
  const bool refusing = refuses(id);
  if (q == nullptr && refusing) ++refused_[id];
  unlock(state);
  if (q == nullptr) {
    if (refusing) {
      refuse(id, size, caller);  // p stays as it was, as realloc promises
    } else {
      fail(Failure{"budget full", nameOf(id), size, caller});
    }
    return nullptr;
  }
  // The copy runs unlocked: nobody else touches p or q meanwhile.
  std::memcpy(q, p, have);
  state = lock();
  discharge(id, heap.ownerOf(p));
  heap.release(p);
  unlock(state);
  return q;
}

// ---- the C library's blocks ------------------------------------------------

void* Manager::bootAllocate(size_t size, size_t align) {
  if (platform_.rawAlloc == nullptr) return nullptr;
  void* p = platform_.rawAlloc(align, size);
  if (p != nullptr && platform_.rawUsable != nullptr) {
    const size_t bytes = platform_.rawUsable(p);
    const uint32_t state = lock();
    newlibBytes_ += bytes;
    unlock(state);
  }
  return p;
}

void Manager::bootRelease(void* p) {
  if (platform_.rawFree == nullptr) return;
  const size_t bytes =
      platform_.rawUsable != nullptr ? platform_.rawUsable(p) : 0;
  const uint32_t state = lock();
  newlibBytes_ -= std::min(bytes, newlibBytes_);
  unlock(state);
  platform_.rawFree(p);
}

// A realloc of a block the C library made: before start into another of its
// blocks, after start into the budget an allocate would use now.
void* Manager::bootCopy(void* p, size_t size, uintptr_t caller) {
  const size_t have =
      platform_.rawUsable != nullptr ? platform_.rawUsable(p) : 0;
  void* q = started_ ? serve(route(caller), size, kAlign, caller, false, true)
                     : bootAllocate(size, kAlign);
  if (q == nullptr) return nullptr;  // p is left as it was
  std::memcpy(q, p, std::min(have, size));
  bootRelease(p);
  return q;
}

// ---- scopes ----------------------------------------------------------------

void Manager::pushScope(BudgetId id) {
  if (started_ && thread() != gameThread_) return;
  const uintptr_t caller = TYRA_MANAGER_CALLER();
  // Before start the game's ids are not known yet: start checks them.
  if (id >= Budget::Max || (started_ && registry_.findBudget(id) == nullptr)) {
    fail(Failure{"scope on an unknown budget", kScopeName, id, caller});
    return;
  }
  if (isScratch(id)) {
    fail(Failure{"scope on a scratch budget", nameOf(id), id, caller});
    return;
  }
  if (depth_ == kMaxScopeDepth) {
    fail(Failure{"budget scope too deep", nameOf(id), depth_, caller});
    return;
  }
  scopes_[depth_++] = id;
}

void Manager::popScope() {
  if (started_ && thread() != gameThread_) return;
  if (depth_ == 0) {
    fail(Failure{"budget scope underflow", kScopeName, 0,
                 TYRA_MANAGER_CALLER()});
    return;
  }
  --depth_;
}

#if TYRA_MEMORY_CHECKS
// Remembers an unscoped call site the first time it allocates, and says so.
void Manager::noteUnscoped(uintptr_t caller) {
  if (reporting_) return;  // the print itself allocating
  if (caller == 0) {
    if (siteZero_) return;
  } else {
    uint32_t i = siteHome(caller) & (kMaxUnscopedSites - 1);
    for (uint32_t n = 0; n < kMaxUnscopedSites; ++n) {
      if (sites_[i] == caller) return;
      if (sites_[i] == 0) break;
      i = (i + 1) & (kMaxUnscopedSites - 1);
    }
  }
  if (siteCount_ == kMaxUnscopedSites) return;

  if (caller == 0) {
    siteZero_ = true;
  } else {
    uint32_t i = siteHome(caller) & (kMaxUnscopedSites - 1);
    while (sites_[i] != 0) i = (i + 1) & (kMaxUnscopedSites - 1);
    sites_[i] = caller;
  }
  ++siteCount_;

  reporting_ = true;
  char hex[17];
  char line[64];
  hexText(caller, hex);
  std::snprintf(line, sizeof line, "[MEM] unscoped caller=0x%s", hex);
  print(line);
  if (siteCount_ == kMaxUnscopedSites) {
    std::snprintf(line, sizeof line,
                  "[MEM] unscoped callers: the set is full (%lu)",
                  static_cast<unsigned long>(kMaxUnscopedSites));
    print(line);
  }
  reporting_ = false;
}
#endif

// ---- frames ----------------------------------------------------------------

void Manager::frameStart() {
  if (!started_) return;
  ++frame_;
  Scratch::frame().reset();
  for (uint32_t i = 0; i < poolCount_; ++i) pools_[i].releaseEmptyExtensions();
  const size_t moved = heap_.step(registry_.settings.defragBytesPerFrame);

  const uint32_t state = lock();
  allocsLastFrame_ = allocsThisFrame_;
  allocsThisFrame_ = 0;
  movedLastFrame_ = moved;
  unlock(state);

  reportExhaustions();
#if TYRA_MEMORY_CHECKS
  sweepPtrs();
#endif
}

void Manager::loadStarted() { Scratch::load().reset(); }

void Manager::noteFallback(const char* what, const char* asset, int lod,
                           const void* key) {
  if (!started_ || platform_.print == nullptr) return;
  const uint32_t state = lock();
  const bool due = reportDue(fallbackKeys_, fallbackFrames_, addressOf(key));
  unlock(state);
  if (!due) return;
  char line[kLineBytes];
  std::snprintf(line, sizeof line,
                "[MEM] fallback %s asset=%s lod=%d frame=%lu",
                what != nullptr ? what : "?", asset != nullptr ? asset : "?",
                lod, static_cast<unsigned long>(frame_));
  print(line);
}

// A line for every pool that was exhausted since the last frame start.
void Manager::reportExhaustions() {
  for (uint32_t i = 0; i < poolCount_; ++i) {
    const uint32_t now = pools_[i].exhaustions();
    if (now == seenExhaustions_[i]) continue;
    char line[kLineBytes];
    std::snprintf(
        line, sizeof line, "[MEM] pool %s exhausted +%lu policy=%s",
        pools_[i].name(), static_cast<unsigned long>(now - seenExhaustions_[i]),
        policyName(registry_.poolDesc(static_cast<PoolId>(i)).policy));
    seenExhaustions_[i] = now;
    print(line);
  }
}

#if TYRA_MEMORY_CHECKS
// Every registered Ptr whose id names a pool entry must still name a live one.
// A checked Ptr is its raw pointer followed by its id; the table knows each by
// the address it lives at.
void Manager::sweepPtrs() {
  static_assert(sizeof(Ptr<char>) == sizeof(void*) + sizeof(Id),
                "a checked Ptr is its raw pointer, then its id");
  uintptr_t deadAt = 0, deadTarget = 0;
  ptrs_.table().forEach([&](void* at, uint32_t) {
    if (deadAt != 0) return;
    const unsigned char* bytes = static_cast<const unsigned char*>(at);
    void* raw;
    Id id;
    std::memcpy(&raw, bytes, sizeof raw);
    std::memcpy(&id, bytes + sizeof(void*), sizeof id);
    if (raw == nullptr || isEmptyId(id)) return;
    if (!checkLiveHook(raw, id, this)) {
      deadAt = addressOf(at);
      deadTarget = addressOf(raw);
    }
  });
  if (deadAt == 0) return;
  char at[17], target[17], line[96];
  hexText(deadAt, at);
  hexText(deadTarget, target);
  std::snprintf(line, sizeof line, "[MEM] dead ptr at 0x%s target=0x%s", at,
                target);
  print(line);
  fail(Failure{"ptr to a dead object", "ptr sweep", 0, 0});
}
#else
void Manager::sweepPtrs() {}
#endif

// ---- pools -----------------------------------------------------------------

PoolCore& Manager::pool(PoolId id) {
  if (id >= poolCount_) {
    fail(Failure{"pool is not in the registry", kName, id,
                 TYRA_MANAGER_CALLER()});
    return none_;
  }
  return pools_[id];
}

// The heap moved a bucket: its token says whose (bucketToken: pool + 1 in the
// high half, the bucket in the low).
void Manager::onBucketMoved(uint32_t token, void* newBase, void* ctx) {
  Manager* m = static_cast<Manager*>(ctx);
  const uint32_t pool = token >> 16;
  if (pool == 0 || pool - 1 >= m->poolCount_) return;
  m->pools_[pool - 1].bucketMoved(static_cast<uint16_t>(token & 0xFFFF),
                                  newBase);
}

#if TYRA_MEMORY_CHECKS
// Whether `raw` is the first byte of an entry of one of the pools (live or
// free), and which pool's. Never fails: idOf asks it of any pointer.
bool Manager::entryOf(const void* raw, PoolId& pool) const {
  if (!owns(raw)) return false;
  const uintptr_t a = addressOf(raw);
  uint16_t bucket = 0;
  if (ptrs_.contains(raw)) {  // an extension in the movable heap
    const uint32_t token = heap_.tokenAt(raw);
    if (token == 0 || (token >> 16) - 1 >= poolCount_) return false;
    pool = static_cast<PoolId>((token >> 16) - 1);
    bucket = static_cast<uint16_t>(token & 0xFFFF);
  } else {  // a first bucket: they are laid out in pool order
    uint32_t lo = 0, hi = poolCount_;
    while (lo < hi) {
      const uint32_t mid = lo + (hi - lo) / 2;
      if (firstLo_[mid] <= a) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    if (lo == 0 || a >= firstHi_[lo - 1]) return false;
    pool = static_cast<PoolId>(lo - 1);
  }
  const PoolCore& core = pools_[pool];
  const void* first = core.entryAt(bucket, 0);
  if (first == nullptr || a < addressOf(first)) return false;
  const uintptr_t offset = a - addressOf(first);
  return offset % stride_[pool] == 0 &&
         offset / stride_[pool] < core.countIn(bucket);
}

bool Manager::checkLiveHook(const void* raw, const Id& id, void* ctx) {
  const Manager* m = static_cast<const Manager*>(ctx);
  return id.pool < m->poolCount_ && m->pools_[id.pool].valid(id, raw);
}

Id Manager::idOfHook(const void* raw, void* ctx) {
  const Manager* m = static_cast<const Manager*>(ctx);
  PoolId pool = 0;
  if (!m->entryOf(raw, pool)) return kEmptyId;
  return m->pools_[pool].idOf(raw);
}
#endif

namespace detail {

PoolCore& typedPool(PoolId id, size_t size, size_t align, uintptr_t caller) {
  Manager& m = Manager::instance();
  if (!m.started()) {
    fail(Failure{"pool used before the manager started", "pool", size, caller});
  }
  PoolCore& core = m.pool(id);
  if (id < m.registry().poolCount()) {
    const PoolDesc& row = m.registry().poolDesc(id);
    const size_t gives = row.align > kAlign ? row.align : kAlign;
    if (row.typeSize != size || align > gives) {
      fail(Failure{"pool type does not match its row",
                   row.name != nullptr ? row.name : "?", size, caller});
    }
  }
  return core;
}

}  // namespace detail

// ---- stats -----------------------------------------------------------------

uint32_t Manager::budgetCount() const { return started_ ? budgetCount_ : 0; }

Manager::BudgetStats Manager::budgetStats(uint32_t i) const {
  BudgetStats s = {};
  if (i >= budgetCount()) return s;
  const BudgetId id = order_[i];
  const BudgetDesc* d = registry_.findBudget(id);
  s.name = d->name;
  s.id = id;
  s.kind = d->kind;
  if (isScratch(id)) {
    const Scratch::Region& r =
        id == Budget::FrameScratch ? Scratch::frame() : Scratch::load();
    s.cap = r.capacity();
    s.used = r.used();
    s.peak = r.peak();
  } else {
    s.cap = home_[id] == id ? regions_[id].capacity() : 0;
    const uint32_t state = lock();
    s.used = used_[id];
    s.peak = peak_[id];
    s.refused = refused_[id];
    unlock(state);
  }
  for (uint32_t p = 0; p < poolCount_; ++p) {
    if (registry_.poolDesc(static_cast<PoolId>(p)).budget == id) {
      s.exhaustions += pools_[p].exhaustions();
    }
  }
  return s;
}

Manager::Totals Manager::totals() const {
  Totals t = {};
  const uint32_t state = lock();
  t.newlibBytes = newlibBytes_;
  if (started_) {
    t.fixedBytes = layout_.fixedBytes;
    t.systemFree = regions_[Budget::System].freeBytes();
    t.allocsLastFrame = allocsLastFrame_;
  }
  unlock(state);
  if (started_) {
    t.heapFree = heap_.freeBytes();
    t.heapLargest = heap_.largestFree();
    t.movedLastFrame = movedLastFrame_;
    t.ptrs = ptrs_.table().size();
  }
#if TYRA_MEMORY_CHECKS
  t.unscopedSites = siteCount_;
#endif
  return t;
}

void Manager::printTable() const {
  if (platform_.print == nullptr) return;
  char line[kLineBytes];
  if (!started_) {
    std::snprintf(line, sizeof line, "[MEM] manager not started newlib=%lu",
                  kb(newlibBytes_));
    print(line);
    return;
  }
  registry_.printTable(platform_.print, layout_);
  for (uint32_t i = 0; i < budgetCount_; ++i) {
    const BudgetStats s = budgetStats(i);
    std::snprintf(line, sizeof line,
                  "[MEM] budget %s cap=%lu used=%lu peak=%lu exhausted=%lu "
                  "refused=%lu",
                  s.name != nullptr ? s.name : "?", kb(s.cap), kb(s.used),
                  kb(s.peak), static_cast<unsigned long>(s.exhaustions),
                  static_cast<unsigned long>(s.refused));
    print(line);
  }
  const Totals t = totals();
  std::snprintf(line, sizeof line, "[MEM] newlib boot=%lu free=%lu reserve=%lu",
                kb(t.newlibBytes),
                kb(platform_.available != nullptr ? platform_.available() : 0),
                kb(registry_.settings.newlibReserveBytes));
  print(line);
  std::snprintf(line, sizeof line,
                "[MEM] heap cap=%lu free=%lu largest=%lu moved=%lu ptrs=%lu",
                kb(layout_.heapBytes), kb(t.heapFree), kb(t.heapLargest),
                kb(t.movedLastFrame), static_cast<unsigned long>(t.ptrs));
  print(line);
}

}  // namespace Memory
}  // namespace Tyra
