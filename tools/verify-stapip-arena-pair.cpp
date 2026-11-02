// Host verifier for the baked arena-pair policy (stapip_arena_pair.hpp, docs/baked-vif-stream.md).
//
// A payload patch writes the entry's SPARE arena and swaps it with `current`. This models the
// entry sequence of a patch against the REAL ImmutableSpanTable (the registry the frame recorder
// borrows from), and a frame the way Vif1Queue runs it: flip the bank, releaseBanks(1 << bank),
// then every REF a recorded frame makes borrow()s the arena it names for that bank. The sequence:
//
//   choose  -> Reuse:    the spare was collected (retired and unread): nothing to allocate or charge
//              Allocate: charge, then makeStaPipArena (a null arena is REFUSED: refund, nothing else)
//              Replace:  makeStaPipArena FIRST (a refusal leaves the busy spare where it is),
//                        then the busy spare goes to the graveyard and the budget moves to the fresh one
//   write   -> add (register) -> retire the old current and keep it as the spare -> swap
//
// A frame draws an entry from its current arena - unless a refused patch is pending, and then it
// draws the bag the ordinary way (no REF of its arenas), as beginBakedBag does: a pending patch's
// current arena holds a stale payload, so no recorded frame may read it (drawEntry).
//
// The allocation failure is not a seam in the header: this file replaces the global
// `operator new[](size_t, nothrow_t)` and refuses it once on demand, so the null the policy sees is
// the null of the real `new (std::nothrow)` inside makeStaPipArena.
//
// Run it with `python tools/verify-tyrax2-host.py --only arena-pair`: that runner writes the host
// `tamtypes.h` type shim frame_chain_arena.hpp's HardwareTrace include needs and builds with
// -DTYRA_HARDWARE_TRACE=0. By hand, with such a shim in <shim>:
//   g++ -std=c++20 -Wall -Werror -DTYRA_HARDWARE_TRACE=0 -I<shim> -Ivendor/tyra/engine/inc
//       tools/verify-stapip-arena-pair.cpp -o build/verify-stapip-arena-pair
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>
#include "renderer/core/paths/path1/frame_chain_arena.hpp"
#include "renderer/3d/pipeline/static/core/stapip_arena_pair.hpp"

namespace {

// Every arena is a `new[]`; nothing else in this program is, so these are the arena books.
std::atomic<unsigned long> liveArrays{0}, arrayNews{0}, injected{0}, lastArraySize{0};
std::atomic<bool> failNextArray{false};

// The array allocator hands out blocks that are only 8-byte aligned, like the console's heap:
// a policy that forgot to round its arena up to 16 bytes would show here (a 64-bit host's
// malloc is 16-aligned and would hide it). The original block's address sits just before.
void* arrayAllocate(std::size_t size) {
  void* block = std::malloc(size + 32);
  if (!block) return nullptr;
  uintptr_t aligned = (reinterpret_cast<uintptr_t>(block) + 15) & ~uintptr_t(15);
  void* p = reinterpret_cast<void*>(aligned + 8);
  static_cast<void**>(p)[-1] = block;
  ++liveArrays;
  ++arrayNews;
  lastArraySize = size;
  return p;
}
void arrayFree(void* p) {
  if (!p) return;
  --liveArrays;
  std::free(static_cast<void**>(p)[-1]);
}

unsigned checks = 0, failed = 0, groupChecks = 0, groupFailed = 0;
const char* group = "";

#define CHECK(x)                                                              \
  do {                                                                        \
    ++checks;                                                                 \
    if (!(x)) {                                                               \
      ++failed;                                                               \
      if (failed <= 25)                                                       \
        std::fprintf(stderr, "FAIL [%s] line %d: %s\n", group, __LINE__, #x); \
    }                                                                         \
  } while (0)

void beginGroup(const char* name) {
  group = name;
  groupChecks = checks;
  groupFailed = failed;
}
void endGroup() {
  std::printf("  %-34s %8u checks, %u failed\n", group, checks - groupChecks,
              failed - groupFailed);
}

}  // namespace

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  if (failNextArray.exchange(false)) {
    ++injected;
    return nullptr;
  }
  return arrayAllocate(size);
}
void* operator new[](std::size_t size) {
  void* p = arrayAllocate(size);
  if (!p) throw std::bad_alloc();
  return p;
}
void operator delete[](void* p) noexcept { arrayFree(p); }
void operator delete[](void* p, std::size_t) noexcept { arrayFree(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { arrayFree(p); }

namespace {

using Tyra::StaPipArena;
using Tyra::StaPipSpareUse;

// The registry keys spans by a 32-bit address; the host's arenas are 64-bit pointers, so each
// arena gets a model address. A freed arena's address is handed out again first (LIFO), which is
// what an allocator does and what proves a freed arena left no span behind.
constexpr uint32_t kStride = 0x1000, kFirstBase = 0x10000, kDecoyBase = 0x80000000u;
constexpr uint32_t kRegistryCapacity = 1024;  // ImmutableSpanTable::kCapacity

struct Entry {
  StaPipArena current, spare;
  uint32_t qwords = 0;
  bool pending = false;  // a refused patch is asked for again the next frame
};

enum class Outcome { Patched, Refused, RegisterFailed };

struct Read {
  uint32_t base;
  const void* data;
  uint32_t qwords;
  uint64_t stamp;
};

struct World {
  Tyra::ImmutableSpanTable table;
  uint32_t bank = 1;
  uint32_t charged = 0;  // the budget: qwords of every arena an entry owns
  uint64_t serial = 0;   // one per write
  unsigned reuse = 0, allocate = 0, replace = 0, refused = 0, registerFailed = 0;
  unsigned unbaked = 0, stale = 0;  // frames drawn the ordinary way / from a pending entry's arena
  uint32_t nextBase = kFirstBase;
  std::vector<uint32_t> freeBases, decoys;
  std::unordered_map<const void*, uint32_t> baseOfData;
  std::unordered_map<uint32_t, uint32_t> readers;  // an oracle apart from the table: bank bits
  std::vector<Read> reads[2];                      // what each bank's recorded frames saw
  std::vector<StaPipArena> graveyard;              // retired, unbudgeted, freed once unread

  // ---- arenas -----------------------------------------------------------------------------
  StaPipArena make(uint32_t qwords) {
    StaPipArena a = Tyra::makeStaPipArena(qwords);
    if (!a) return a;
    uint32_t b;
    if (!freeBases.empty()) {
      b = freeBases.back();
      freeBases.pop_back();
    } else {
      b = nextBase;
      nextBase += kStride;
      CHECK(nextBase < kDecoyBase);
    }
    CHECK(!table.hasBase(b));  // the arena that had this address left no span in the registry
    baseOfData[a.data] = b;
    return a;
  }
  uint32_t base(const StaPipArena& a) const { return baseOfData.at(a.data); }
  uint32_t readersOf(uint32_t b) const {
    const auto it = readers.find(b);
    return it == readers.end() ? 0u : it->second;
  }
  static uint64_t stampOf(const void* data) {
    uint64_t s;
    std::memcpy(&s, data, 8);
    return s;
  }
  static bool intact(const Read& r) {
    uint64_t last;
    std::memcpy(&last, static_cast<const uint8_t*>(r.data) + r.qwords * 16 - 8, 8);
    return stampOf(r.data) == r.stamp && last == r.stamp;
  }

  // The one place an arena is written: the lease, not the age, says it may be.
  void write(const StaPipArena& a) {
    const uint32_t b = base(a);
    CHECK(!table.hasBase(b) || table.reclaimable(b));  // the registry says no recorded frame reads it
    CHECK(readersOf(b) == 0);                          // and so does the bookkeeping beside it
    const uint64_t stamp = ++serial;
    const size_t bytes = size_t(a.qwords) * 16;
    std::memset(a.data, int(stamp & 0xFF) ^ 0x5A, bytes);
    std::memcpy(a.data, &stamp, 8);
    std::memcpy(static_cast<uint8_t*>(a.data) + bytes - 8, &stamp, 8);
  }

  // The arena is freed after this: it must be unread, and its address handed back.
  void release(const StaPipArena& a) {
    const uint32_t b = base(a);
    CHECK(readersOf(b) == 0);
    for (auto& bankReads : reads)
      for (Read& r : bankReads)
        if (r.base == b) r.data = nullptr;
    readers.erase(b);
    baseOfData.erase(a.data);
    freeBases.push_back(b);
  }

  // ---- the frame --------------------------------------------------------------------------
  void beginFrame(bool releaseBank = true) {
    bank ^= 1;
    if (!releaseBank) return;  // a recorded frame is kept: its bank is not released
    table.releaseBanks(1u << bank);
    for (auto& r : readers) r.second &= ~(1u << bank);
    reads[bank].clear();
  }
  // A REF to `current` recorded this frame: after the entry's patch, as the draw does.
  void drawRef(const StaPipArena& a) {
    const uint32_t b = base(a);
    CHECK(table.borrow(b, a.qwords, 1u << bank));
    readers[b] |= 1u << bank;
    reads[bank].push_back({b, a.data, a.qwords, stampOf(a.data)});
  }
  // What a frame draws for an entry, as beginBakedBag decides it: the current arena, or - while a
  // refused patch is pending - the bag the ordinary way, no REF of its arenas.
  void drawEntry(const Entry& e) {
    if (e.pending) {
      ++unbaked;
      return;
    }
    drawRef(e.current);
  }
  // The old rule, for the check that it is gone: a pending entry's arena drawn is a stale payload.
  void countStaleDraws(const Entry& e, unsigned draws) {
    if (e.pending) stale += draws;
  }
  bool inFlightIntact() const {
    for (const auto& bankReads : reads)
      for (const Read& r : bankReads)
        if (r.data && !intact(r)) return false;
    return true;
  }
  void sweep() {
    for (size_t i = 0; i < graveyard.size();) {
      if (table.collectRetired(base(graveyard[i]))) {
        release(graveyard[i]);
        graveyard[i] = std::move(graveyard.back());
        graveyard.pop_back();
      } else {
        ++i;
      }
    }
  }
  void endFrame() {
    sweep();
    CHECK(inFlightIntact());  // nothing a recorded frame reads was written since it read it
  }

  // ---- an entry ---------------------------------------------------------------------------
  bool bake(Entry& e, uint32_t qwords) {
    e = Entry{};
    e.qwords = qwords;
    StaPipArena a = make(qwords);
    if (!a) return false;
    write(a);
    if (!table.add(base(a), qwords)) {
      release(a);
      return false;
    }
    charged += qwords;
    e.current = std::move(a);
    return true;
  }

  // Eviction, expiry, clear(): both arenas to the graveyard, the budget returns both.
  void retireEntry(Entry& e) {
    for (StaPipArena* a : {&e.current, &e.spare}) {
      if (!*a) continue;
      table.retire(base(*a));
      charged -= a->qwords;
      graveyard.push_back(std::move(*a));
    }
    e = Entry{};
  }

  Outcome patch(Entry& e) {
    const uint32_t want = e.qwords;
    const bool hasSpare = static_cast<bool>(e.spare);
    // collecting removes the span, so it is asked only when the spare could be reused
    const bool collected =
        hasSpare && e.spare.qwords == want && table.collectRetired(base(e.spare));
    const StaPipSpareUse use = Tyra::chooseSpareUse(
        hasSpare, hasSpare ? e.spare.qwords : 0, want, collected);

    StaPipArena fresh;
    if (use == StaPipSpareUse::Allocate) {
      charged += want;
      fresh = make(want);
      if (!fresh) {  // refused: nothing is left charged, nothing was written
        charged -= want;
        e.pending = true;
        ++refused;
        return Outcome::Refused;
      }
      ++allocate;
    } else if (use == StaPipSpareUse::Replace) {
      fresh = make(want);
      if (!fresh) {  // refused: the busy spare stays where it is, the entry is untouched
        e.pending = true;
        ++refused;
        return Outcome::Refused;
      }
      table.retire(base(e.spare));
      charged = charged - e.spare.qwords + want;
      graveyard.push_back(std::move(e.spare));
      ++replace;
    } else {
      fresh = std::move(e.spare);  // already collected: out of the registry, unread
      ++reuse;
    }

    write(fresh);
    if (!table.add(base(fresh), fresh.qwords)) {
      // The registry is full: the entry keeps its current arena, still published; the written
      // arena stays its (unpublished) spare, charged and owned.
      e.spare = std::move(fresh);
      ++registerFailed;
      return Outcome::RegisterFailed;
    }
    table.retire(base(e.current));
    e.spare = std::move(e.current);
    e.current = std::move(fresh);
    e.pending = false;
    return Outcome::Patched;
  }

  // ---- invariants -------------------------------------------------------------------------
  // The budget is the footprint; every arena is owned exactly once; the registry holds an active
  // span for each current and none for a spare or a graveyard arena.
  bool balanced(const Entry* first, size_t count) const {
    uint64_t sum = 0, arenas = graveyard.size();
    bool registry = true;
    for (size_t i = 0; i < count; ++i) {
      const Entry& e = first[i];
      if (e.current) {
        sum += e.current.qwords;
        ++arenas;
        registry = registry && table.contains(base(e.current), e.current.qwords);
      }
      if (e.spare) {
        sum += e.spare.qwords;
        ++arenas;
        registry = registry && !table.contains(base(e.spare), e.spare.qwords);
      }
    }
    for (const StaPipArena& g : graveyard)
      registry = registry && !table.contains(base(g), g.qwords);
    return sum == charged && arenas == liveArrays.load() && registry;
  }

  void fillRegistry() {
    while (table.add(kDecoyBase + uint32_t(decoys.size()) * 0x100, 1))
      decoys.push_back(kDecoyBase + uint32_t(decoys.size()) * 0x100);
  }
  void freeDecoy() {
    table.remove(decoys.back());
    decoys.pop_back();
  }
  // Proves the registry holds no span: all of its slots are free.
  void expectEmptyRegistry() {
    uint32_t added = 0;
    while (table.add(kDecoyBase + added * 0x100, 1)) ++added;
    CHECK(added == kRegistryCapacity);
    for (uint32_t i = 0; i < added; ++i) table.remove(kDecoyBase + i * 0x100);
  }
};

// All of the entries retired, two frames pass (both banks released): nothing is left.
void finish(World& w, std::vector<Entry>& entries) {
  for (Entry& e : entries) w.retireEntry(e);
  for (int i = 0; i < 2; ++i) {
    w.beginFrame();
    w.endFrame();
  }
  CHECK(w.charged == 0);
  CHECK(w.graveyard.empty());
  CHECK(liveArrays.load() == 0);
  w.expectEmptyRegistry();
}

// Frames in which the one entry is patched and drawn.
void steadyFrames(World& w, Entry& e, unsigned frames) {
  for (unsigned i = 0; i < frames; ++i) {
    w.beginFrame();
    w.patch(e);
    w.drawRef(e.current);
    w.endFrame();
  }
}

void drawFirstFrame(World& w, std::vector<Entry>& entries) {
  w.beginFrame();
  for (Entry& e : entries) w.drawRef(e.current);
  w.endFrame();
}

// ---- the checks -----------------------------------------------------------------------------

void truthTable() {
  beginGroup("truth_table");
  using U = StaPipSpareUse;
  static_assert(Tyra::chooseSpareUse(false, 0, 8, false) == U::Allocate);
  static_assert(Tyra::chooseSpareUse(true, 8, 8, true) == U::Reuse);
  CHECK(Tyra::chooseSpareUse(false, 0, 8, false) == U::Allocate);
  CHECK(Tyra::chooseSpareUse(true, 8, 8, true) == U::Reuse);
  CHECK(Tyra::chooseSpareUse(true, 8, 8, false) == U::Replace);
  CHECK(Tyra::chooseSpareUse(true, 4, 8, true) == U::Replace);
  // no spare means Allocate whatever the other arguments say; a size mismatch is never Reuse
  CHECK(Tyra::chooseSpareUse(false, 8, 8, true) == U::Allocate);
  CHECK(Tyra::chooseSpareUse(false, 0, 0, false) == U::Allocate);
  CHECK(Tyra::chooseSpareUse(true, 8, 4, true) == U::Replace);
  CHECK(Tyra::chooseSpareUse(true, 4, 8, false) == U::Replace);
  endGroup();
}

void arenaAllocation() {
  beginGroup("arena_allocation");
  CHECK(liveArrays.load() == 0);
  for (uint32_t qw = 1; qw <= 70; ++qw) {
    StaPipArena a = Tyra::makeStaPipArena(qw);
    CHECK(static_cast<bool>(a) && a.qwords == qw);
    CHECK((reinterpret_cast<uintptr_t>(a.data) & 15) == 0);
    CHECK((reinterpret_cast<uintptr_t>(a.raw.get()) & 15) == 8);  // this heap is not 16-aligned...
    CHECK(a.data == a.raw.get() + 8);                               // ...so data was rounded up
    CHECK(lastArraySize.load() == qw * 16 + 15 &&
          static_cast<uint8_t*>(a.data) + qw * 16 <= a.raw.get() + lastArraySize.load());
    std::memset(a.data, 0xA5, qw * 16);  // the whole range is writable (AddressSanitizer checks)
  }
  CHECK(liveArrays.load() == 0);

  // A refused allocation is the real nothrow new[]'s null: an empty arena, and nothing leaks.
  const unsigned long newsBefore = arrayNews.load(), injectedBefore = injected.load();
  failNextArray = true;
  StaPipArena refused = Tyra::makeStaPipArena(8);
  CHECK(!refused && !refused.raw && refused.data == nullptr && refused.qwords == 0);
  CHECK(!failNextArray.load() && injected.load() == injectedBefore + 1);
  CHECK(arrayNews.load() == newsBefore && liveArrays.load() == 0);
  StaPipArena again = Tyra::makeStaPipArena(8);  // the refusal was once
  CHECK(static_cast<bool>(again) && liveArrays.load() == 1);

  // Nothing to size, or a size no heap could hold: refused without asking the heap.
  const unsigned long asked = arrayNews.load();
  CHECK(!Tyra::makeStaPipArena(0));
  CHECK(arrayNews.load() == asked);

  // A moved-from arena is empty, not a stale pointer.
  StaPipArena moved = std::move(again);
  CHECK(static_cast<bool>(moved) && !again && again.data == nullptr && again.qwords == 0);
  StaPipArena assigned;
  assigned = std::move(moved);
  CHECK(static_cast<bool>(assigned) && !moved && moved.data == nullptr && moved.qwords == 0);
  assigned = StaPipArena{};  // dropping the last owner frees it
  CHECK(liveArrays.load() == 0);
  endGroup();
}

// Random frames over 32 entries; shared by writes_only_unread and the held-lease variant.
void randomFrames(const char* name, unsigned frames, bool heldFrames, uint32_t seed) {
  beginGroup(name);
  CHECK(liveArrays.load() == 0);
  {
    auto w = std::make_unique<World>();
    std::mt19937 rng(seed);
    static const unsigned period[3] = {1, 2, 4};  // patched 1/1, 1/2, 1/4 of the frames
    std::vector<Entry> entries(32);
    for (Entry& e : entries) CHECK(w->bake(e, 4 + rng() % 61));
    drawFirstFrame(*w, entries);
    for (unsigned f = 0; f < frames; ++f) {
      w->beginFrame(!(heldFrames && rng() % 48 == 0));
      for (size_t i = 0; i < entries.size(); ++i) {
        Entry& e = entries[i];
        if (e.pending || rng() % period[i % 3] == 0) w->patch(e);
        w->drawRef(e.current);
        if (rng() % 4 == 0) w->drawRef(e.current);  // a second view replays the same arena
      }
      w->endFrame();
      CHECK(w->balanced(entries.data(), entries.size()));
    }
    CHECK(w->allocate == 32);  // one spare each, the only allocation in an entry's life
    CHECK(w->reuse > 0);
    if (heldFrames) CHECK(w->replace > 0);
    else CHECK(w->replace == 0);  // no lease outlives its frame, so no spare is ever busy
    finish(*w, entries);
  }
  CHECK(liveArrays.load() == 0);
  endGroup();
}

void everyFramePatchReuses() {
  beginGroup("every_frame_patch_reuses");
  {
    auto w = std::make_unique<World>();
    std::vector<Entry> entries(1);
    CHECK(w->bake(entries[0], 16));
    drawFirstFrame(*w, entries);
    const unsigned long newsBefore = arrayNews.load();
    steadyFrames(*w, entries[0], 1000);
    CHECK(w->allocate == 1);
    CHECK(w->replace == 0);
    CHECK(w->reuse == 999);
    CHECK(arrayNews.load() - newsBefore == 1);  // the spare; no patch after it asked the heap
    CHECK(w->charged == 32);                    // 2 x its size
    CHECK(w->balanced(entries.data(), 1));
    finish(*w, entries);
  }
  CHECK(liveArrays.load() == 0);
  endGroup();
}

void heldLeaseReplaces() {
  beginGroup("held_lease_replaces");
  {
    auto w = std::make_unique<World>();
    std::vector<Entry> entries(1);
    Entry& e = entries[0];
    CHECK(w->bake(e, 16));
    drawFirstFrame(*w, entries);
    steadyFrames(*w, e, 6);
    CHECK(w->allocate == 1 && w->replace == 0 && w->reuse == 5);

    // The frame that read the spare is kept: its bank is not released this frame.
    const void* held = e.spare.data;
    const uint32_t heldBase = w->base(e.spare);
    const uint64_t heldStamp = World::stampOf(held);
    const unsigned long newsBefore = arrayNews.load();
    w->beginFrame(false);
    CHECK(w->readersOf(heldBase) != 0);
    CHECK(!w->table.reclaimable(heldBase));
    CHECK(w->patch(e) == Outcome::Patched);
    CHECK(w->replace == 1 && w->reuse == 5);  // a fresh arena, once
    CHECK(arrayNews.load() == newsBefore + 1);
    CHECK(e.current.data != held && e.spare.data != held);
    CHECK(World::stampOf(held) == heldStamp);  // the held arena was never written
    CHECK(w->graveyard.size() == 1 && w->graveyard[0].data == held);
    CHECK(w->table.hasBase(heldBase) && !w->table.contains(heldBase, 16));
    CHECK(w->charged == 32);  // the busy one left the budget, the fresh one is in it
    w->drawRef(e.current);
    w->endFrame();
    CHECK(w->graveyard.size() == 1 && World::stampOf(held) == heldStamp);
    CHECK(w->balanced(entries.data(), 1));

    // The lease ends with the next release of that bank; every patch after is a Reuse again.
    steadyFrames(*w, e, 2);
    CHECK(w->graveyard.empty());
    steadyFrames(*w, e, 1000);
    CHECK(w->allocate == 1 && w->replace == 1);

    // A spare of another size is not Reuse even when it is free.
    steadyFrames(*w, e, 2);
    e.qwords = 24;
    w->beginFrame();
    CHECK(w->patch(e) == Outcome::Patched);
    CHECK(w->replace == 2 && e.current.qwords == 24);
    w->drawRef(e.current);
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));
    finish(*w, entries);
  }
  CHECK(liveArrays.load() == 0);
  endGroup();
}

void budgetAndRegistryBalance() {
  beginGroup("budget_and_registry_balance");
  CHECK(liveArrays.load() == 0);
  const unsigned long injectedBefore = injected.load();
  {
    auto w = std::make_unique<World>();
    std::mt19937 rng(20261101);
    static const unsigned period[4] = {1, 2, 4, 0};  // the last is never patched
    std::vector<Entry> entries(24);
    for (Entry& e : entries) CHECK(w->bake(e, 4 + rng() % 61));
    drawFirstFrame(*w, entries);
    CHECK(w->balanced(entries.data(), entries.size()));
    for (unsigned f = 0; f < 4000; ++f) {
      w->beginFrame(rng() % 40 != 0);
      if (rng() % 12 == 0) {  // an eviction, mid-frame, and a new entry baked in its place
        Entry& victim = entries[rng() % entries.size()];
        w->retireEntry(victim);
        CHECK(w->bake(victim, 4 + rng() % 61));
      }
      for (size_t i = 0; i < entries.size(); ++i) {
        Entry& e = entries[i];
        const unsigned p = period[i % 4];
        if (e.pending || (p && rng() % p == 0)) {
          if (rng() % 48 == 0) failNextArray = true;  // the heap refuses this patch's arena
          w->patch(e);
          failNextArray = false;
        }
        w->drawEntry(e);
      }
      w->endFrame();
      CHECK(w->balanced(entries.data(), entries.size()));
    }
    CHECK(w->replace > 0 && w->refused > 0 && w->reuse > 0);
    CHECK(w->unbaked == w->refused);  // every refused patch's frame was drawn the ordinary way
    finish(*w, entries);  // every entry retired, two frames: the budget and the registry are empty
  }
  CHECK(injected.load() > injectedBefore);
  CHECK(liveArrays.load() == 0);
  endGroup();
}

void registerFailureKeepsCurrent() {
  beginGroup("register_failure_keeps_current");
  CHECK(liveArrays.load() == 0);
  {
    // An entry's first patch (Allocate) finds the registry full.
    auto w = std::make_unique<World>();
    std::vector<Entry> entries(1);
    Entry& e = entries[0];
    CHECK(w->bake(e, 16));
    drawFirstFrame(*w, entries);
    steadyFrames(*w, e, 0);
    w->fillRegistry();
    const void* oldCurrent = e.current.data;
    const uint64_t oldStamp = World::stampOf(oldCurrent);
    w->beginFrame();
    CHECK(w->patch(e) == Outcome::RegisterFailed);
    CHECK(e.current.data == oldCurrent && World::stampOf(oldCurrent) == oldStamp);
    CHECK(w->table.borrow(w->base(e.current), 16, 1u << w->bank));  // still published, borrowable
    w->readers[w->base(e.current)] |= 1u << w->bank;
    CHECK(static_cast<bool>(e.spare) && !w->table.hasBase(w->base(e.spare)));
    CHECK(World::stampOf(e.spare.data) != oldStamp);  // the new payload was written
    CHECK(w->charged == 32 && liveArrays.load() == 2);  // charged and owned: no leak
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));

    // Asked again with the registry still full: it reuses that spare, it does not allocate.
    const unsigned long newsBefore = arrayNews.load();
    w->beginFrame();
    CHECK(w->patch(e) == Outcome::RegisterFailed);
    w->drawRef(e.current);
    w->endFrame();
    CHECK(arrayNews.load() == newsBefore && w->allocate == 1 && w->reuse == 1);
    CHECK(w->balanced(entries.data(), 1));

    // A slot frees: the patch publishes the spare and swaps.
    w->freeDecoy();
    w->beginFrame();
    CHECK(w->patch(e) == Outcome::Patched);
    CHECK(e.current.data != oldCurrent && e.spare.data == oldCurrent);
    w->drawRef(e.current);
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));
    w->decoys.clear();
    for (uint32_t i = 0; i < kRegistryCapacity; ++i) w->table.remove(kDecoyBase + i * 0x100);
    finish(*w, entries);
  }
  {
    // A Replace (the spare is kept busy) finds the registry full.
    auto w = std::make_unique<World>();
    std::vector<Entry> entries(1);
    Entry& e = entries[0];
    CHECK(w->bake(e, 16));
    drawFirstFrame(*w, entries);
    steadyFrames(*w, e, 5);
    w->fillRegistry();
    const void* oldCurrent = e.current.data;
    const void* busy = e.spare.data;
    const uint64_t oldStamp = World::stampOf(oldCurrent), busyStamp = World::stampOf(busy);
    w->beginFrame(false);
    CHECK(w->patch(e) == Outcome::RegisterFailed);
    CHECK(w->replace == 1);
    CHECK(e.current.data == oldCurrent && World::stampOf(oldCurrent) == oldStamp);
    CHECK(World::stampOf(busy) == busyStamp);
    CHECK(w->table.borrow(w->base(e.current), 16, 1u << w->bank));
    w->readers[w->base(e.current)] |= 1u << w->bank;
    CHECK(static_cast<bool>(e.spare) && e.spare.data != busy && !w->table.hasBase(w->base(e.spare)));
    CHECK(w->charged == 32 && liveArrays.load() == 3);  // current, spare, the busy one in the graveyard
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));
    // The lease ends, the graveyard frees the busy arena and its slot; the next patch publishes.
    for (int i = 0; i < 3; ++i) {
      w->beginFrame();
      w->drawRef(e.current);
      w->endFrame();
    }
    CHECK(w->graveyard.empty() && liveArrays.load() == 2);
    w->beginFrame();
    CHECK(w->patch(e) == Outcome::Patched);
    w->drawRef(e.current);
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));
    w->decoys.clear();
    for (uint32_t i = 0; i < kRegistryCapacity; ++i) w->table.remove(kDecoyBase + i * 0x100);
    finish(*w, entries);
  }
  {
    // A Reuse never fails on a full registry: collecting the spare freed the slot it takes back.
    auto w = std::make_unique<World>();
    std::vector<Entry> entries(1);
    Entry& e = entries[0];
    CHECK(w->bake(e, 16));
    drawFirstFrame(*w, entries);
    steadyFrames(*w, e, 4);
    w->fillRegistry();
    w->beginFrame();
    const unsigned reuseBefore = w->reuse;
    CHECK(w->patch(e) == Outcome::Patched);
    CHECK(w->reuse == reuseBefore + 1 && w->registerFailed == 0);
    w->drawRef(e.current);
    w->endFrame();
    CHECK(!w->table.add(kDecoyBase - 0x100, 1));  // and the registry is as full as before
    w->decoys.clear();
    for (uint32_t i = 0; i < kRegistryCapacity; ++i) w->table.remove(kDecoyBase + i * 0x100);
    finish(*w, entries);
  }
  CHECK(liveArrays.load() == 0);
  endGroup();
}

void refusedAllocateKeepsCurrent() {
  beginGroup("refused_allocate_keeps_current");
  CHECK(liveArrays.load() == 0);
  {
    // Allocate refused: the entry has only its current arena.
    auto w = std::make_unique<World>();
    std::vector<Entry> entries(1);
    Entry& e = entries[0];
    CHECK(w->bake(e, 16));
    drawFirstFrame(*w, entries);
    const void* current = e.current.data;
    const uint64_t stamp = World::stampOf(current);
    const uint32_t chargedBefore = w->charged;
    const uint64_t serialBefore = w->serial;
    const unsigned long newsBefore = arrayNews.load(), injectedBefore = injected.load();
    w->beginFrame();
    failNextArray = true;
    CHECK(w->patch(e) == Outcome::Refused);
    CHECK(!failNextArray.load() && injected.load() == injectedBefore + 1);  // the real allocation refused
    CHECK(e.current.data == current && World::stampOf(current) == stamp);
    CHECK(!e.spare && e.pending);
    CHECK(w->charged == chargedBefore);           // charges nothing
    CHECK(w->serial == serialBefore);             // writes nothing
    CHECK(arrayNews.load() == newsBefore && liveArrays.load() == 1);
    CHECK(w->table.contains(w->base(e.current), 16));  // still published, still borrowable
    const size_t readsBefore = w->reads[w->bank].size();
    w->drawEntry(e);  // but the frame draws the bag the ordinary way: its payload is stale
    CHECK(w->unbaked == 1 && w->reads[w->bank].size() == readsBefore);
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));
    // The next frame's patch succeeds.
    w->beginFrame();
    CHECK(w->patch(e) == Outcome::Patched);
    CHECK(!e.pending && e.spare.data == current && w->allocate == 1);
    w->drawRef(e.current);
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));
    finish(*w, entries);
  }
  {
    // Replace refused: the busy spare stays where it is, and the entry is exactly as it was.
    auto w = std::make_unique<World>();
    std::vector<Entry> entries(1);
    Entry& e = entries[0];
    CHECK(w->bake(e, 16));
    drawFirstFrame(*w, entries);
    steadyFrames(*w, e, 6);
    const void* current = e.current.data;
    const void* spare = e.spare.data;
    const uint64_t currentStamp = World::stampOf(current), spareStamp = World::stampOf(spare);
    const uint32_t chargedBefore = w->charged;
    const uint64_t serialBefore = w->serial;
    const unsigned long newsBefore = arrayNews.load();
    w->beginFrame(false);
    failNextArray = true;
    CHECK(w->patch(e) == Outcome::Refused);
    CHECK(!failNextArray.load());
    CHECK(e.current.data == current && e.spare.data == spare && e.pending);
    CHECK(World::stampOf(current) == currentStamp && World::stampOf(spare) == spareStamp);
    CHECK(w->charged == chargedBefore && w->serial == serialBefore);
    CHECK(arrayNews.load() == newsBefore && w->graveyard.empty() && w->replace == 0);
    const size_t readsBefore = w->reads[w->bank].size();
    w->drawEntry(e);  // drawn the ordinary way: neither arena is named this frame
    CHECK(w->unbaked == 1 && w->reads[w->bank].size() == readsBefore);
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));
    // The next frame asks again (the spare's bank is still held, so it is a Replace) and succeeds.
    w->beginFrame();
    CHECK(w->patch(e) == Outcome::Patched);
    CHECK(!e.pending && w->replace == 1 && w->graveyard.size() == 1);
    w->drawRef(e.current);
    w->endFrame();
    CHECK(w->balanced(entries.data(), 1));
    steadyFrames(*w, e, 4);
    CHECK(w->graveyard.empty() && w->allocate == 1 && w->replace == 1);
    finish(*w, entries);
  }
  CHECK(liveArrays.load() == 0);
  endGroup();
}

// A refused patch is never replayed: over random frames with refusals, the draws beginBakedBag
// makes (drawEntry) never name a pending entry's arena, so no recorded frame reads a stale
// payload - and the old rule (replay the current arena) would have, which the model counts too.
void refusedPatchNeverReplaysStale() {
  beginGroup("refused_patch_never_replays_stale");
  CHECK(liveArrays.load() == 0);
  {
    auto w = std::make_unique<World>();
    std::mt19937 rng(20261102);
    std::vector<Entry> entries(16);
    for (Entry& e : entries) CHECK(w->bake(e, 4 + rng() % 61));
    drawFirstFrame(*w, entries);
    unsigned longestPending = 0;
    std::vector<unsigned> run(entries.size(), 0);
    for (unsigned f = 0; f < 3000; ++f) {
      w->beginFrame(rng() % 24 != 0);
      for (size_t i = 0; i < entries.size(); ++i) {
        Entry& e = entries[i];
        if (e.pending || rng() % 2 == 0) {
          // a run of refusals: the heap stays short for a while, as on the console
          if (rng() % 8 == 0) failNextArray = true;
          w->patch(e);
          failNextArray = false;
        }
        const unsigned views = 1 + rng() % 2;  // split screen draws it twice
        w->countStaleDraws(e, views);
        const size_t before = w->reads[w->bank].size();  // a kept frame's reads stay in the bank
        for (unsigned v = 0; v < views; ++v) w->drawEntry(e);
        const size_t after = w->reads[w->bank].size();
        CHECK(after - before == (e.pending ? 0u : views));  // no REF of a pending entry's arena
        run[i] = e.pending ? run[i] + 1 : 0;
        if (run[i] > longestPending) longestPending = run[i];
      }
      w->endFrame();
      CHECK(w->balanced(entries.data(), entries.size()));
    }
    CHECK(w->refused > 0 && w->unbaked > 0);
    CHECK(w->stale > 0);         // the old rule would have replayed stale payloads...
    CHECK(longestPending > 1);   // ...for runs of frames (refusals in a row)
    finish(*w, entries);
  }
  CHECK(liveArrays.load() == 0);
  endGroup();
}

}  // namespace

int main() {
  std::printf("verify-stapip-arena-pair\n");
  truthTable();
  arenaAllocation();
  randomFrames("writes_only_unread", 10000, false, 917);
  randomFrames("writes_only_unread_held_frames", 10000, true, 1031);
  everyFramePatchReuses();
  heldLeaseReplaces();
  budgetAndRegistryBalance();
  registerFailureKeepsCurrent();
  refusedAllocateKeepsCurrent();
  refusedPatchNeverReplaysStale();
  CHECK(liveArrays.load() == 0);
  std::printf("verify-stapip-arena-pair: %u checks, %u failed\n", checks, failed);
  return failed != 0;
}
