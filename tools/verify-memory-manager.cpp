// Added by TyraX: host verifier for the memory manager (inc/memory/manager.hpp,
// src/memory/manager.cpp) and Pool<T> (inc/memory/pool.hpp). Built and run by
// tools/verify-memory.py, once with TYRA_MEMORY_CHECKS=0 and once with =1.
//
// The manager runs on a fake Platform: newlib is the host's malloc behind an
// aligning wrapper that counts every call, the thread id is a variable, the
// lock records whether it is ever taken twice or left taken, print collects the
// lines and halt throws, as installThrowingFailure() makes fail() do. The first
// scenario drives Manager::instance() (Pool<T> and MemScope bind to it); the
// rest make Managers of their own.

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <new>
#include <string>
#include <utility>
#include <vector>

#include "memory/memory.hpp"
#include "memory/telemetry.hpp"
#include "verify-memory-check.hpp"

namespace Scratch = Tyra::Scratch;

namespace {

// ---- the fake platform -----------------------------------------------------

struct Fake {
  struct Raw {
    void* base;   // what malloc returned
    size_t size;  // what was asked for: rawUsable answers it
  };
  std::map<void*, Raw> blocks;  // by the address handed out
  int rawAllocs = 0, rawFrees = 0, unknownFrees = 0;
  size_t lastAlign = 0, lastSize = 0;
  size_t availableBytes = 8u << 20;
  uint32_t thread = 1;
  bool locked = false;
  int locks = 0, unlocks = 0, reentries = 0, badUnlocks = 0;
  std::vector<std::string> lines;
};

// Never destroyed, like the manager: the blocks it holds are still in use when
// the program exits (and LeakSanitizer looks after the static destructors).
Fake& fake = *new Fake();

constexpr uint32_t kLockState = 0xA5u;

void* fakeRawAlloc(size_t align, size_t size) {
  ++fake.rawAllocs;
  fake.lastAlign = align;
  fake.lastSize = size;
  if (align < alignof(std::max_align_t)) align = alignof(std::max_align_t);
  void* base = std::malloc(size + align);
  if (base == nullptr) return nullptr;
  const uintptr_t a =
      (reinterpret_cast<uintptr_t>(base) + align - 1) & ~(uintptr_t(align) - 1);
  void* p = reinterpret_cast<void*>(a);
  fake.blocks[p] = Fake::Raw{base, size};
  return p;
}

void fakeRawFree(void* p) {
  auto it = fake.blocks.find(p);
  if (it == fake.blocks.end()) {
    ++fake.unknownFrees;
    return;
  }
  ++fake.rawFrees;
  std::free(it->second.base);
  fake.blocks.erase(it);
}

size_t fakeRawUsable(void* p) {
  auto it = fake.blocks.find(p);
  return it != fake.blocks.end() ? it->second.size : 0;
}

size_t fakeAvailable() { return fake.availableBytes; }
uint32_t fakeThreadId() { return fake.thread; }

uint32_t fakeLock() {
  if (fake.locked) ++fake.reentries;
  fake.locked = true;
  ++fake.locks;
  return kLockState;
}

void fakeUnlock(uint32_t state) {
  if (state != kLockState || !fake.locked) ++fake.badUnlocks;
  fake.locked = false;
  ++fake.unlocks;
}

void fakePrint(const char* line) { fake.lines.push_back(line); }
void fakeHalt() { verify_detail::throwingHalt(); }

Platform fakePlatform() {
  Platform p;
  p.rawAlloc = &fakeRawAlloc;
  p.rawFree = &fakeRawFree;
  p.rawUsable = &fakeRawUsable;
  p.available = &fakeAvailable;
  p.threadId = &fakeThreadId;
  p.lock = &fakeLock;
  p.unlock = &fakeUnlock;
  p.print = &fakePrint;
  p.halt = &fakeHalt;
  return p;
}

bool anyLine(const char* text) {
  for (const std::string& l : fake.lines) {
    if (l.find(text) != std::string::npos) return true;
  }
  return false;
}

int countLines(const char* text) {
  int n = 0;
  for (const std::string& l : fake.lines) {
    if (l.find(text) != std::string::npos) ++n;
  }
  return n;
}

bool lockBalanced() {
  return !fake.locked && fake.locks == fake.unlocks && fake.reentries == 0 &&
         fake.badUnlocks == 0;
}

// ---- helpers over a manager ------------------------------------------------

constexpr BudgetId kGame = Budget::FirstGame;
constexpr BudgetId kMeasured = Budget::FirstGame + 1;

Manager::BudgetStats statsOf(const Manager& m, const char* name) {
  for (uint32_t i = 0; i < m.budgetCount(); ++i) {
    const Manager::BudgetStats s = m.budgetStats(i);
    if (s.name != nullptr && std::strcmp(s.name, name) == 0) return s;
  }
  Manager::BudgetStats none = {};
  return none;
}

bool inRegion(const Manager& m, const void* p, BudgetId id) {
  return m.regionOf(p) == static_cast<int>(id);
}

uintptr_t addressOf(const void* p) { return reinterpret_cast<uintptr_t>(p); }

// The line printTable writes for a budget: sizes in KB, rounded up.
std::string expectedLine(const Manager::BudgetStats& s) {
  auto kb = [](size_t b) { return std::to_string((b + 1023) / 1024); };
  return "[MEM] budget " + std::string(s.name) + " cap=" + kb(s.cap) +
         " used=" + kb(s.used) + " peak=" + kb(s.peak) +
         " exhausted=" + std::to_string(s.exhaustions) +
         " refused=" + std::to_string(s.refused);
}

std::unique_ptr<Manager> freshManager() {
  std::unique_ptr<Manager> m(new Manager());
  m->setPlatform(fakePlatform());
  return m;
}

struct Rng {
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  uint32_t below(uint32_t n) { return next() % n; }
};

// ---- the types the pools hold -----------------------------------------------

int alive = 0;

struct Foo {
  int v;
  explicit Foo(int x) : v(x) { ++alive; }
  ~Foo() { --alive; }
};

struct Node {
  int v;
  Ptr<Node> next;
  explicit Node(int x) : v(x) {}
};

struct alignas(64) Wide {
  unsigned char bytes[64];
  explicit Wide(unsigned char b) { std::memset(bytes, b, sizeof bytes); }
};

struct Big {
  char bytes[sizeof(Foo) + 8];
};

PoolId fooPool = kNoPool, haltPool = kNoPool, recyclePool = kNoPool,
       nodePool = kNoPool, widePool = kNoPool, unalignedPool = kNoPool;

// ---- 1. Manager::instance(), Pool<T> ----------------------------------------

void instanceScenario() {
  installThrowingFailure();
  fake.lines.clear();

  Manager& m = Manager::instance();
  CHECK(&m == &Manager::instance());
  CHECK(!m.started());
  m.setPlatform(fakePlatform());

  // Before start: newlib.
  void* early = m.allocate(40, 16, 0);
  CHECK(fake.rawAllocs == 1);
  std::memset(early, 7, 40);
  void* early2 = m.allocate(24, 16, 0);  // realloc'd after start
  std::memset(early2, 9, 24);
  void* early3 = m.reallocate(nullptr, 32, 0);  // realloc before start: newlib
  CHECK(fake.rawAllocs == 3 && !m.owns(early3));
  early3 = m.reallocate(early3, 64, 0);
  CHECK(fake.rawAllocs == 4 && fake.rawFrees == 1 && !m.owns(early3));
  CHECK(m.totals().newlibBytes == 40 + 24 + 64);
  CHECK(m.budgetCount() == 0);

  Registry r;
  r.budget(kGame, "game").regionBytes = 64 * 1024;
  r.budget(kGame).extensionBytes = 64 * 1024;
  r.budget(kMeasured, "measured");  // regionBytes 0
  r.budget(Budget::FrameScratch).regionBytes = 16 * 1024;
  r.budget(Budget::LoadScratch).regionBytes = 8 * 1024;
  fooPool = r.pool(PoolDesc{"foo", kGame, sizeof(Foo), alignof(Foo), 2, 2, 2,
                            Policy::Refuse});
  haltPool = r.pool(PoolDesc{"halt", kGame, sizeof(Foo), alignof(Foo), 1, 0, 0,
                             Policy::Halt});
  recyclePool = r.pool(PoolDesc{"recycle", kMeasured, sizeof(Foo), alignof(Foo),
                                2, 0, 0, Policy::RecycleOldest});
  nodePool = r.pool(PoolDesc{"node", kGame, sizeof(Node), alignof(Node), 2, 4,
                             8, Policy::Refuse});
  widePool = r.pool(PoolDesc{"wide", kGame, sizeof(Wide), alignof(Wide), 1, 2,
                             2, Policy::Refuse});
  unalignedPool = r.pool(
      PoolDesc{"unaligned", kGame, sizeof(Wide), 0, 1, 0, 0, Policy::Refuse});
  Layout expected;
  CHECK(r.fits(fake.availableBytes, expected));

  CHECK(detail::g_ptrs == nullptr);
  CHECK(m.start(r));
  CHECK(m.started());
  const int rawAllocsAfterStart = fake.rawAllocs;
  CHECK(rawAllocsAfterStart == 5);  // the block, once
  CHECK(fake.lastSize == expected.blockBytes);
  CHECK(fake.lastAlign == expected.blockAlign);
  CHECK(detail::g_ptrs != nullptr);
  CHECK(detail::g_movableLo != 0 && detail::g_movableLo < detail::g_movableHi);
  CHECK(detail::g_movableHi - detail::g_movableLo == expected.heapBytes);
  CHECK(detail::g_pinHook != nullptr);
#if TYRA_MEMORY_CHECKS
  CHECK(detail::g_idHook != nullptr);
#endif
  CHECK(m.totals().fixedBytes == expected.fixedBytes);

  // Review Focus 2: a block from before start goes back to newlib.
  m.release(early);
  CHECK(fake.rawFrees == 2);
  CHECK(m.totals().newlibBytes == 24 + 64);
  m.release(early3);
  CHECK(fake.rawFrees == 3);

  void* sys = m.allocate(1000, 16, 0);
  CHECK(m.owns(sys) && inRegion(m, sys, Budget::System));
  CHECK(statsOf(m, "System").used >= 1000);
#if TYRA_MEMORY_CHECKS
  CHECK(anyLine("[MEM] unscoped caller=0x0"));
  CHECK(m.totals().unscopedSites == 1);
#else
  CHECK(!anyLine("[MEM] unscoped"));
  CHECK(m.totals().unscopedSites == 0);
#endif

  const size_t systemUsed = statsOf(m, "System").used;
  {
    MemScope s(kGame);
    void* g = m.allocate(10240, 16, 0);
    CHECK(inRegion(m, g, kGame));
    CHECK(statsOf(m, "game").used >= 10240);
    CHECK(statsOf(m, "System").used == systemUsed);
    // Review Focus 1: an over-aligned block, realloc'd with no byte lost.
    void* a = m.allocate(5000, 128, 0);
    CHECK(addressOf(a) % 128 == 0 && inRegion(m, a, kGame));
    std::memset(a, 1, 5000);
    a = m.reallocate(a, 6000, 0);
    CHECK(allBytes(a, 5000, 1) && inRegion(m, a, kGame));
    m.release(a);
    void* d = m.allocate(3000, 64, 0);
    CHECK(addressOf(d) % 64 == 0 && inRegion(m, d, kGame));
    m.release(d);
    // Review Focus 2: a block from before start, realloc'd: copied into the
    // current budget, given back to newlib.
    const int freesBefore = fake.rawFrees;
    void* moved = m.reallocate(early2, 4000, 0);
    CHECK(inRegion(m, moved, kGame) && allBytes(moved, 24, 9));
    CHECK(fake.rawFrees == freesBefore + 1);
    CHECK(m.totals().newlibBytes == 0);
    m.release(moved);
    m.release(g);
    CHECK(statsOf(m, "game").used == 0);
    CHECK(statsOf(m, "game").peak >= 10240 + 6000);
  }
  {
    MemScope s(kMeasured);
    void* q = m.allocate(4096, 16, 0);
    CHECK(inRegion(m, q, Budget::System));
    CHECK(statsOf(m, "measured").used >= 4096);
    CHECK(statsOf(m, "measured").cap == 0);
    m.release(q);
    CHECK(statsOf(m, "measured").used == 0);
    CHECK(statsOf(m, "measured").peak >= 4096);
  }

  // Review Focus 3: realloc stays in its budget and fails loudly there.
  void* g2;
  {
    MemScope s(kGame);
    g2 = m.allocate(30000, 16, 0);
  }
  std::memset(g2, 3, 30000);
  const size_t gameUsed = statsOf(m, "game").used;
  CHECK(throwsFailure([&] { m.reallocate(g2, 70000, 0); }, "budget full"));
  CHECK(lockBalanced());
  CHECK(statsOf(m, "game").used == gameUsed && allBytes(g2, 30000, 3));
  CHECK(anyLine("[MEM] FAIL budget full name=game size=70000"));
  // With no scope open, and system's room, it still stays in game.
  void* g3 = m.reallocate(g2, 32000, 0);
  CHECK(inRegion(m, g3, kGame) && allBytes(g3, 30000, 3));
  CHECK(throwsFailure(
      [&] {
        MemScope s(kGame);
        m.allocate(60000, 16, 0x77);
      },
      "budget full"));
  CHECK(anyLine("[MEM] FAIL budget full name=game size=60000 caller=0x77"));
  CHECK(lockBalanced());
  m.release(g3);
  CHECK(statsOf(m, "game").used == 0);

  // Other threads go to system, scope or no scope, and a scope they open
  // does not touch the game thread's.
  void* g4;
  {
    MemScope s(kGame);
    g4 = m.allocate(100, 16, 0);
  }
  std::memset(g4, 6, 100);
  fake.thread = 2;
  void* other;
  {
    MemScope s(kGame);
    other = m.allocate(64, 16, 0);
    CHECK(inRegion(m, other, Budget::System));
  }
  // realloc from another thread keeps the block's budget.
  g4 = m.reallocate(g4, 5000, 0);
  CHECK(inRegion(m, g4, kGame) && allBytes(g4, 100, 6));
  CHECK(statsOf(m, "game").used >= 5000);
  fake.thread = 1;
  {
    MemScope s(kMeasured);
    fake.thread = 2;
    {
      MemScope t(kGame);
    }  // ignored: not the game thread
    fake.thread = 1;
    void* q = m.allocate(32, 16, 0);
    CHECK(inRegion(m, q, Budget::System) && statsOf(m, "measured").used >= 32);
    m.release(q);
  }
  m.release(other);
  m.release(g4);
  m.release(sys);

  // Frame start: the allocation count (allocate and reallocate, any thread),
  // the frame scratch.
  CHECK(Scratch::frame().capacity() == 16 * 1024);
  CHECK(Scratch::load().capacity() == 8 * 1024);
  m.frameStart();
  void* c1 = m.allocate(8, 16, 0);
  void* c2 = m.allocate(8, 16, 0);
  CHECK(Scratch::frame().allocate(100, 16) != nullptr);
  CHECK(Scratch::load().allocate(100, 16) != nullptr);
  m.frameStart();
  CHECK(m.totals().allocsLastFrame == 2);
  CHECK(Scratch::frame().used() == 0);
  CHECK(Scratch::load().used() != 0);  // only a load resets it
  m.loadStarted();
  CHECK(Scratch::load().used() == 0);
  c1 = m.reallocate(c1, 4000, 0);
  fake.thread = 3;
  m.release(m.allocate(8, 16, 0));  // another thread counts too
  fake.thread = 1;
  m.frameStart();
  CHECK(m.totals().allocsLastFrame == 2);
  m.frameStart();
  CHECK(m.totals().allocsLastFrame == 0);
  m.release(c1);
  m.release(c2);

  // A bucket the defragmentation moves - the pool follows it
  // through the move listener, and the Ptrs (a ring through the bucket that
  // moves, Review Focus 4) are resolved.
  {
    // The heap is empty: the extensions come in address order.
    Pool<Node> nodes(nodePool);
    std::vector<Ptr<Node>> held;
    // 2 in the first bucket, then extensions of 4: b1 2..5, b2 6..9,
    // b3 10..13.
    for (int i = 0; i < 14; ++i) held.push_back(nodes.acquire(i));
    for (int i = 0; i < 14; ++i) CHECK(held[i] && held[i]->v == i);
    const uintptr_t b3Before = addressOf(held[10].get());
    for (int i = 2; i < 10; ++i) nodes.release(held[i]);
    // The ring: 0 -> 10 -> 11 -> 12 -> 13 -> 1 -> 0.
    const int ring[] = {0, 10, 11, 12, 13, 1};
    for (int k = 0; k < 6; ++k) held[ring[k]]->next = held[ring[(k + 1) % 6]];
    const uint32_t ptrsBefore = m.totals().ptrs;
    CHECK(ptrsBefore >= 6);
    {
      // A Pin (through the hook start installed) holds b3 where it is.
      Pin<Node> pin(held[11]);
      m.frameStart();  // b1 stays as the spare, b2 goes, b3 may not move
      CHECK(m.totals().movedLastFrame == 0);
      CHECK(addressOf(held[10].get()) == b3Before);
    }
    m.frameStart();  // unpinned: b3 slides into b2's place
    CHECK(m.totals().movedLastFrame > 0);
    CHECK(addressOf(held[10].get()) < b3Before);
    CHECK(m.totals().ptrs == ptrsBefore);
    // The pool follows its bucket (the move listener reached bucketMoved).
    CHECK(m.pool(nodePool).entryAt(3, 0) == held[10].get());
    CHECK(m.pool(nodePool).entryAt(3, 3) == held[13].get());
    for (int k = 0; k < 6; ++k) CHECK(held[ring[k]]->v == ring[k]);
    Ptr<Node> walk = held[0];
    for (int k = 1; k <= 6; ++k) {
      walk = walk->next;
      CHECK(walk && walk->v == ring[k % 6]);
    }
    walk.reset();
    int seen = 0;
    nodes.forEach([&](Node& n) {
      CHECK(n.next);
      ++seen;
    });
    CHECK(seen == 6 && nodes.live() == 6);
#if TYRA_MEMORY_CHECKS
    CHECK(held[12].id().bucket == 3);
    m.frameStart();  // the sweep finds every Ptr live after the move
#endif
    for (int k = 0; k < 6; ++k) held[ring[k]]->next.reset();
    for (Ptr<Node>& p : held) nodes.release(p);
    m.frameStart();
    CHECK(m.totals().ptrs == 0);
  }

  // Pool<T>: placement-new, a Ptr, destroy on release.
  {
    Pool<Foo> foos(fooPool);
    Ptr<Foo> f = foos.acquire(5);
    CHECK(f && f->v == 5 && alive == 1 && foos.live() == 1);
    foos.release(f);
    CHECK(!f && alive == 0 && foos.live() == 0);
    foos.release(f);  // an empty Ptr: nothing
    CHECK(alive == 0);

    // Up to the cap (2 first + 2 extensions of 2), then Refuse: empty.
    std::vector<Ptr<Foo>> held;
    for (int i = 0; i < 6; ++i) held.push_back(foos.acquire(10 + i));
    CHECK(alive == 6 && foos.live() == 6);
    for (int i = 0; i < 6; ++i) CHECK(held[i] && held[i]->v == 10 + i);
    // The extensions come from the movable heap, charged to game.
    CHECK(addressOf(held[5].get()) >= detail::g_movableLo &&
          addressOf(held[5].get()) < detail::g_movableHi);
    CHECK(m.regionOf(held[5].get()) == -1 && m.owns(held[5].get()));
    CHECK(statsOf(m, "game").exhaustions == 0);
    Ptr<Foo> refused = foos.acquire(99);
    CHECK(!refused && alive == 6);
    CHECK(statsOf(m, "game").exhaustions == 1);
    int sum = 0, visited = 0;
    foos.forEach([&](Foo& x) {
      sum += x.v;
      ++visited;
    });
    CHECK(visited == 6 && sum == 10 + 11 + 12 + 13 + 14 + 15);

    // The exhaustion is a line at the next frame start, once.
    fake.lines.clear();
    m.frameStart();
    CHECK(countLines("[MEM] pool foo exhausted +1 policy=Refuse") == 1);
    m.frameStart();
    CHECK(countLines("[MEM] pool foo exhausted") == 1);
    CHECK(foos.acquire(1).get() == nullptr);
    CHECK(foos.acquire(1).get() == nullptr);
    m.frameStart();
    CHECK(countLines("[MEM] pool foo exhausted +2 policy=Refuse") == 1);
    CHECK(statsOf(m, "game").exhaustions == 3);

#if TYRA_MEMORY_CHECKS
    // The id hook: a pool entry's Ptr carries its id; anything else none.
    CHECK(held[0].id().pool == fooPool && held[0].id().bucket == 0);
    CHECK(held[5].id().pool == fooPool && held[5].id().bucket != 0);
    // Inside an entry (Foo's stride is 4): not the start of one.
    Ptr<Foo> interior(reinterpret_cast<Foo*>(
        reinterpret_cast<unsigned char*>(held[1].get()) + 2));
    CHECK(isEmptyId(interior.id()));
    Ptr<Foo> interiorExt(reinterpret_cast<Foo*>(
        reinterpret_cast<unsigned char*>(held[5].get()) + 2));
    CHECK(isEmptyId(interiorExt.id()));
    void* routed = m.allocate(sizeof(Foo), 16, 0);
    Ptr<Foo> plain(static_cast<Foo*>(routed));
    CHECK(isEmptyId(plain.id()));
    plain.reset();
    m.release(routed);
    // A copy outlives the release: dead, at get() and at the frame sweep.
    Ptr<Foo> copy = held[4];
    foos.release(held[4]);
    CHECK(throwsFailure([&] { copy.get(); }, "ptr to a dead object"));
    CHECK(throwsFailure([&] { m.frameStart(); }, "ptr to a dead object"));
    CHECK(anyLine("[MEM] dead ptr at 0x"));
    copy.reset();
    m.frameStart();  // nothing dead any more
    Ptr<Foo> firstCopy = held[0];
    foos.release(held[0]);
    CHECK(throwsFailure([&] { firstCopy.get(); }, "ptr to a dead object"));
    firstCopy.reset();
#else
    foos.release(held[4]);
    foos.release(held[0]);
#endif
    for (Ptr<Foo>& p : held) foos.release(p);
    CHECK(alive == 0 && foos.live() == 0);
    m.frameStart();
  }

  // Halt: a failure; RecycleOldest: the oldest is destroyed and reused.
  {
    Pool<Foo> halts(haltPool);
    Ptr<Foo> one = halts.acquire(1);
    CHECK(one && alive == 1);
    CHECK(throwsFailure([&] { halts.acquire(2); }, "pool exhausted"));
    CHECK(alive == 1);
    halts.release(one);

    Pool<Foo> recycled(recyclePool);
    Ptr<Foo> a = recycled.acquire(1);
    Ptr<Foo> b = recycled.acquire(2);
    Ptr<Foo> c = recycled.acquire(3);  // takes a's entry
    CHECK(alive == 2 && c.get() != b.get() && c->v == 3);
#if TYRA_MEMORY_CHECKS
    CHECK(throwsFailure([&] { a.get(); }, "ptr to a dead object"));
#endif
    a.reset();
    CHECK(statsOf(m, "measured").exhaustions == 1);
    recycled.release(b);
    recycled.release(c);
    CHECK(alive == 0);
  }

  // The pool's row: a wrong type, an unknown id.
  CHECK(throwsFailure([&] { Pool<Big> wrong(fooPool); },
                      "pool type does not match its row"));
  CHECK(throwsFailure([&] { Pool<Wide> wrong(unalignedPool); },
                      "pool type does not match its row"));
  CHECK(anyLine("[MEM] FAIL pool type does not match its row name=unaligned"));
  {
    Pool<Foo> fooInWide(fooPool);
  }  // alignof(Foo) <= 16: no row demand needed
  CHECK(throwsFailure([&] { Pool<Foo> nowhere(77); },
                      "pool is not in the registry"));
  CHECK(throwsFailure([&] { m.pool(77); }, "pool is not in the registry"));
  CHECK(&m.pool(fooPool) != &m.pool(nodePool));
  CHECK(std::strcmp(m.pool(nodePool).name(), "node") == 0);

  // An over-aligned type: every entry aligned, the first bucket and the
  // extensions (which are pinned).
  {
    Pool<Wide> wides(widePool);
    std::vector<Ptr<Wide>> w;
    for (int i = 0; i < 5; ++i) w.push_back(wides.acquire((unsigned char)i));
    for (int i = 0; i < 5; ++i) {
      CHECK(w[i] && addressOf(w[i].get()) % 64 == 0);
      CHECK(allBytes(w[i]->bytes, 64, (unsigned char)i));
    }
    for (Ptr<Wide>& p : w) wides.release(p);
  }

  // The stats and the table.
  CHECK(m.budgetCount() == 10);  // the 8 engine rows, game, measured
  const Manager::BudgetStats game = statsOf(m, "game");
  CHECK(game.id == kGame && game.kind == Kind::Logic && game.cap == 64 * 1024);
  CHECK(game.peak >= game.used && game.peak >= 30000);
  const Manager::BudgetStats system = statsOf(m, "System");
  CHECK(system.id == Budget::System && system.kind == Kind::System);
  CHECK(system.cap == (expected.systemBytes & ~size_t(15)));
  CHECK(statsOf(m, "FrameScratch").cap == 16 * 1024);
  CHECK(statsOf(m, "FrameScratch").kind == Kind::Scratch);
  CHECK(statsOf(m, "Renderer").cap == 0);
  const Manager::Totals t = m.totals();
  CHECK(t.systemFree > 0 && t.systemFree <= system.cap);
  CHECK(t.heapFree <= expected.heapBytes && t.heapLargest <= t.heapFree);
  fake.lines.clear();
  m.printTable();
  CHECK(anyLine("[MEM] budget game kind=Logic region=64 heap=64"));
  CHECK(anyLine("[MEM] pool node budget=game"));
  CHECK(anyLine("[MEM] total fixed="));
  // foo refused 3 times, halt once: both are game's pools.
  CHECK(game.exhaustions == 4);
  CHECK(anyLine(expectedLine(game).c_str()));
  CHECK(anyLine("[MEM] budget game cap=64 used="));
  const Manager::BudgetStats measured = statsOf(m, "measured");
  CHECK(measured.used == 0 && measured.exhaustions == 1);
  CHECK(anyLine(expectedLine(measured).c_str()));
  CHECK(anyLine("[MEM] budget measured cap=0 used=0 peak="));
  CHECK(anyLine("[MEM] budget Renderer cap=0 used=0 peak=0 exhausted=0"));
  CHECK(anyLine("[MEM] newlib boot=0 free="));
  CHECK(anyLine("[MEM] heap cap=64 free="));

  // Starting twice is a failure.
  CHECK(throwsFailure([&] { m.start(r); }, "manager started twice"));

  // Nothing above went to newlib after start; the lock was always paired.
  CHECK(fake.rawAllocs == rawAllocsAfterStart);
  CHECK(fake.unknownFrees == 0);
  CHECK(lockBalanced());

  // A registry that does not fit: the table, then the failure - and the
  // started manager's globals are left alone.
  const PtrRegistry* installed = detail::g_ptrs;
  Registry big;
  big.budget(kGame, "game").regionBytes = 2 * 1024 * 1024;
  fake.availableBytes = 1024;
  fake.lines.clear();
  std::unique_ptr<Manager> fresh = freshManager();
  CHECK(throwsFailure([&] { fresh->start(big); }, "registry does not fit"));
  CHECK(anyLine("[MEM] total"));
  CHECK(anyLine("[MEM] budget game kind=Logic region=2048"));
  CHECK(anyLine("registry does not fit: fixed"));
  CHECK(!fresh->started() && detail::g_ptrs == installed);
  CHECK(fake.rawAllocs == rawAllocsAfterStart);
  fake.availableBytes = 8u << 20;
}

// ---- 2. start: what it refuses ---------------------------------------------

void startFailures() {
  installThrowingFailure();

  // System's region is sized by settings.systemBytes only.
  {
    Registry r;
    r.budget(Budget::System).regionBytes = 4096;
    std::unique_ptr<Manager> m = freshManager();
    fake.lines.clear();
    CHECK(throwsFailure([&] { m->start(r); }, "settings.systemBytes"));
    CHECK(anyLine("[MEM] total") && !m->started());
  }
  // A system region below the smallest block; a region rounded down.
  {
    Registry r;
    r.settings.systemBytes = 16;
    std::unique_ptr<Manager> m = freshManager();
    fake.lines.clear();
    CHECK(throwsFailure([&] { m->start(r); }, "region too small"));
    CHECK(anyLine("[MEM] total") && !m->started());
  }
  {
    Registry r;
    r.budget(kGame, "tiny").regionBytes = 20;  // 16 after rounding
    std::unique_ptr<Manager> m = freshManager();
    CHECK(throwsFailure([&] { m->start(r); }, "region too small"));
    CHECK(anyLine("name=tiny"));
  }
  {
    Registry r;
    r.budget(kGame, "odd").regionBytes = 1000;
    r.settings.systemBytes = 100 * 1024 + 8;
    std::unique_ptr<Manager> m = freshManager();
    CHECK(m->start(r));
    CHECK(statsOf(*m, "odd").cap == 992);
    CHECK(statsOf(*m, "System").cap == 100 * 1024);
    void* p = m->allocate(900, 16, 0);
    CHECK(inRegion(*m, p, Budget::System));
    m->pushScope(kGame);
    void* q = m->allocate(900, 16, 0);
    CHECK(inRegion(*m, q, kGame));
    CHECK(throwsFailure([&] { m->allocate(900, 16, 0); }, "budget full"));
    m->release(q);
    m->popScope();
    m->release(p);
  }
  // More extensions than the movable heap has rows for.
  {
    Registry r;
    for (int i = 0; i < 9; ++i) {
      r.pool(
          PoolDesc{"many", Budget::System, 16, 16, 1, 1, 63, Policy::Refuse});
    }
    std::unique_ptr<Manager> m = freshManager();
    fake.lines.clear();
    CHECK(throwsFailure([&] { m->start(r); }, "movable heap"));
    CHECK(anyLine("[MEM] total") && !m->started());
  }
  {
    Registry r;  // a pool PoolCore would refuse, found before anything starts
    r.pool(PoolDesc{"wide", Budget::System, 16, 16, 1, 1, 64, Policy::Refuse});
    std::unique_ptr<Manager> m = freshManager();
    CHECK(throwsFailure([&] { m->start(r); }, "too many extensions"));
    CHECK(!m->started() && detail::g_ptrs == nullptr);
  }
  // The block is allocated at the layout's alignment and size.
  {
    Registry r;
    r.pool(
        PoolDesc{"aligned", Budget::System, 128, 128, 3, 0, 0, Policy::Refuse});
    Layout l;
    CHECK(r.fits(fake.availableBytes, l) && l.blockAlign == 128);
    std::unique_ptr<Manager> m = freshManager();
    CHECK(m->start(r));
    CHECK(fake.lastAlign == 128 && fake.lastSize == l.blockBytes);
    void* e = m->pool(0).acquire();
    CHECK(e != nullptr && addressOf(e) % 128 == 0 && m->owns(e));
    CHECK(m->regionOf(e) == -1);
    m->pool(0).release(e);
  }
  // No platform: start refuses, allocate answers null.
  {
    std::unique_ptr<Manager> m(new Manager());
    CHECK(m->allocate(16, 16, 0) == nullptr);
    m->release(nullptr);
    Registry r;
    CHECK(throwsFailure([&] { m->start(r); }, "platform"));
  }
  CHECK(detail::g_ptrs == nullptr && lockBalanced());
}

// ---- 3. routing, scopes, the unscoped callers ------------------------------

void routing() {
  installThrowingFailure();
  Registry r;
  r.budget(kGame, "game").regionBytes = 32 * 1024;
  r.budget(kMeasured, "measured");
  r.budget(Budget::FrameScratch).regionBytes = 1024;
  r.pool(PoolDesc{"entries", kGame, 32, 16, 4, 0, 0, Policy::Refuse});
  std::unique_ptr<Manager> m = freshManager();
  CHECK(m->start(r));

  // owns / regionOf.
  void* sys = m->allocate(100, 16, 0);
  CHECK(m->owns(sys) && m->regionOf(sys) == Budget::System);
  CHECK(!m->owns(&r) && m->regionOf(&r) == -1);
  CHECK(m->regionOf(nullptr) == -1 && !m->owns(nullptr));
  void* frameBlock = Scratch::frame().allocate(16, 16);
  CHECK(m->regionOf(frameBlock) == Budget::FrameScratch);
  void* entry = m->pool(0).acquire();
  CHECK(m->owns(entry) && m->regionOf(entry) == -1);

  // Scopes nest; the innermost wins; popping goes back out.
  m->pushScope(kGame);
  m->pushScope(kMeasured);
  void* inner = m->allocate(64, 16, 0);
  m->popScope();
  void* outer = m->allocate(64, 16, 0);
  m->popScope();
  void* none = m->allocate(64, 16, 0);
  CHECK(inRegion(*m, inner, Budget::System) &&
        statsOf(*m, "measured").used >= 64);
  CHECK(inRegion(*m, outer, kGame) && inRegion(*m, none, Budget::System));
  m->pushScope(Budget::System);  // explicit: system, and not "unscoped"
  void* explicitSys = m->allocate(64, 16, 0xE5);
  m->popScope();
  CHECK(inRegion(*m, explicitSys, Budget::System));
  for (void* p : {inner, outer, none, explicitSys}) m->release(p);

  // The depth and the ids a scope accepts.
  for (uint32_t i = 0; i < Manager::kMaxScopeDepth; ++i) m->pushScope(kGame);
  CHECK(throwsFailure([&] { m->pushScope(kGame); }, "budget scope too deep"));
  for (uint32_t i = 0; i < Manager::kMaxScopeDepth; ++i) m->popScope();
  CHECK(throwsFailure([&] { m->popScope(); }, "budget scope underflow"));
  CHECK(throwsFailure([&] { m->pushScope(40); }, "unknown budget"));
  CHECK(
      throwsFailure([&] { m->pushScope(Budget::Max + 3); }, "unknown budget"));
  CHECK(throwsFailure([&] { m->pushScope(Budget::FrameScratch); },
                      "scratch budget"));
  CHECK(throwsFailure([&] { m->pushScope(Budget::LoadScratch); },
                      "scratch budget"));
  fake.thread = 9;  // another thread: neither push nor pop touch the stack
  m->pushScope(kGame);
  m->popScope();
  m->popScope();
  fake.thread = 1;
  CHECK(inRegion(*m, sys, Budget::System));

  // Unscoped callers: remembered once each on the game thread, with checks.
  fake.lines.clear();
  void* u1 = m->allocate(16, 16, 0x1234);
  void* u2 = m->allocate(16, 16, 0x1234);
  void* u3 = m->allocate(16, 16, 0xABCDEF);
  fake.thread = 4;
  void* u4 = m->allocate(16, 16, 0x5555);  // another thread: not unscoped
  fake.thread = 1;
  m->pushScope(kGame);
  void* u5 = m->allocate(16, 16, 0x6666);  // scoped
  m->popScope();
#if TYRA_MEMORY_CHECKS
  CHECK(countLines("[MEM] unscoped caller=0x1234") == 1);
  CHECK(countLines("[MEM] unscoped caller=0xabcdef") == 1);
  CHECK(!anyLine("0x5555") && !anyLine("0x6666") && !anyLine("0xe5"));
  const uint32_t sitesBefore = m->totals().unscopedSites;
  CHECK(sitesBefore >= 3);
  // The set holds 256; past that, one line says so and nothing else.
  std::vector<void*> many;
  for (uintptr_t c = 0; c < 400; ++c) {
    many.push_back(m->allocate(16, 16, 0x100000 + c * 4));
  }
  CHECK(m->totals().unscopedSites == Manager::kMaxUnscopedSites);
  CHECK(countLines("[MEM] unscoped caller=") ==
        static_cast<int>(Manager::kMaxUnscopedSites - sitesBefore + 2));
  CHECK(countLines("[MEM] unscoped callers: the set is full") == 1);
  for (void* p : many) m->release(p);
#else
  CHECK(!anyLine("[MEM] unscoped"));
  CHECK(m->totals().unscopedSites == 0);
#endif
  for (void* p : {u1, u2, u3, u4, u5}) m->release(p);

  // Alignments: below 16 is 16; not a power of two is null, no failure.
  void* small = m->allocate(1, 1, 0);
  CHECK(small != nullptr && addressOf(small) % 16 == 0);
  CHECK(m->allocate(16, 48, 0) == nullptr);
  void* zero = m->allocate(0, 16, 0);  // malloc(0): a real, unique block
  CHECK(zero != nullptr && zero != small);
  m->release(small);
  m->release(zero);

  // release: null, a block outside (back to newlib), memory of the manager
  // that is not a routed block, a block released twice.
  const int frees = fake.rawFrees;
  m->release(nullptr);
  void* outside = fakeRawAlloc(16, 48);
  m->release(outside);
  CHECK(fake.rawFrees == frees + 1);
  const Manager::BudgetStats sysBefore = statsOf(*m, "System");
  void* twice = m->allocate(200, 16, 0);
  m->release(twice);
#if TYRA_MEMORY_CHECKS
  CHECK(throwsFailure([&] { m->release(entry); }, "not a routed block"));
  CHECK(throwsFailure([&] { m->release(frameBlock); }, "not a routed block"));
  CHECK(throwsFailure([&] { m->release(twice); }, "not live"));
  CHECK(throwsFailure([&] { m->reallocate(entry, 64, 0); },
                      "not a routed block"));
#else
  m->release(entry);  // ignored
  CHECK(m->pool(0).live() == 1);
  CHECK(m->reallocate(entry, 64, 0) == nullptr);
#endif
  CHECK(statsOf(*m, "System").used == sysBefore.used);
  CHECK(lockBalanced());

  // reallocate: null, 0, a shrink in place, a grow.
  void* p = m->reallocate(nullptr, 100, 0);
  CHECK(inRegion(*m, p, Budget::System));
  std::memset(p, 4, 100);
  CHECK(m->reallocate(p, 50, 0) == p && allBytes(p, 100, 4));
  void* grown = m->reallocate(p, 5000, 0);
  CHECK(grown != p && allBytes(grown, 100, 4));
  CHECK(m->reallocate(grown, 0, 0) == nullptr);  // frees, like newlib
  CHECK(statsOf(*m, "System").used == sysBefore.used);

  m->pool(0).release(entry);
  m->release(sys);
  CHECK(lockBalanced());
}

// ---- 4. the accounting of a shared region -----------------------------------

// Blocks of every budget that shares system's region, over-aligned ones among
// them (whose gap can join the block in front), allocated, realloc'd and
// released in a random order: every budget ends where it started, to the
// byte, and its peak is never below what it holds.
void sharedAccounting() {
  installThrowingFailure();
  Registry r;
  r.budget(kMeasured, "measured");
  r.budget(kMeasured + 1, "second");
  std::unique_ptr<Manager> m = freshManager();
  CHECK(m->start(r));
  const BudgetId ids[] = {Budget::System, kMeasured, kMeasured + 1};
  const char* names[] = {"System", "measured", "second"};
  size_t start[3];
  for (int i = 0; i < 3; ++i) start[i] = statsOf(*m, names[i]).used;

  struct Live {
    void* p;
    size_t bytes;
    unsigned char fill;
  };
  std::vector<Live> live;
  Rng rng(7);
  bool ok = true;
  for (int step = 0; step < 6000; ++step) {
    const uint32_t what = rng.below(10);
    if (what < 5 || live.empty()) {
      const int who = static_cast<int>(rng.below(3));
      static const size_t aligns[] = {16, 16, 32, 64, 128};
      const size_t align = aligns[rng.below(5)];
      const size_t bytes = 1 + rng.below(3000);
      if (ids[who] != Budget::System) m->pushScope(ids[who]);
      void* p = m->allocate(bytes, align, 0);
      if (ids[who] != Budget::System) m->popScope();
      ok = ok && p != nullptr && addressOf(p) % align == 0 &&
           m->regionOf(p) == Budget::System;
      const unsigned char fill = static_cast<unsigned char>(step);
      std::memset(p, fill, bytes);
      live.push_back(Live{p, bytes, fill});
    } else if (what < 7) {
      Live& l = live[rng.below(static_cast<uint32_t>(live.size()))];
      const size_t bytes = 1 + rng.below(4000);
      void* q = m->reallocate(l.p, bytes, 0);
      ok = ok && q != nullptr && allBytes(q, std::min(bytes, l.bytes), l.fill);
      l.p = q;
      if (bytes > l.bytes)
        std::memset(static_cast<unsigned char*>(q) + l.bytes, l.fill,
                    bytes - l.bytes);
      l.bytes = bytes;
    } else {
      const size_t i = rng.below(static_cast<uint32_t>(live.size()));
      ok = ok && allBytes(live[i].p, live[i].bytes, live[i].fill);
      m->release(live[i].p);
      live[i] = live.back();
      live.pop_back();
    }
    for (int i = 0; i < 3; ++i) {
      const Manager::BudgetStats s = statsOf(*m, names[i]);
      ok = ok && s.peak >= s.used;
    }
  }
  CHECK(ok);
  for (const Live& l : live) m->release(l.p);
  for (int i = 0; i < 3; ++i) {
    CHECK(statsOf(*m, names[i]).used == start[i]);
    CHECK(statsOf(*m, names[i]).peak > start[i]);
  }
  CHECK(lockBalanced());
}

// ---- 5. full budgets: halt or refuse ----------------------------------------

constexpr BudgetId kRender = Budget::FirstGame + 2;

// A budget scope on a manager of the scenario's own (MemScope opens one on
// Manager::instance()).
struct Scope {
  Manager& m;
  Scope(Manager& manager, BudgetId id) : m(manager) { m.pushScope(id); }
  ~Scope() { m.popScope(); }
};
constexpr BudgetId kAsset = Budget::FirstGame + 3;

// A budget's region, filled to the last block that fits; the blocks are
// returned for release.
std::vector<void*> fill(Manager& m, BudgetId id, size_t block) {
  std::vector<void*> blocks;
  m.pushScope(id);
  for (;;) {
    void* p = m.allocate(block, 16, 0x500);
    if (p == nullptr) break;
    blocks.push_back(p);
  }
  m.popScope();
  return blocks;
}

void refusals() {
  installThrowingFailure();
  fake.lines.clear();
  Registry r;
  r.budget(kGame, "game").regionBytes = 16 * 1024;  // Logic: halts
  r.budget(kRender, "props", Kind::Render).regionBytes = 16 * 1024;
  r.budget(kAsset, "sounds", Kind::Asset).regionBytes = 16 * 1024;
  r.budget(kAsset).onFull = OnFull::Refuse;  // a game may say otherwise
  r.budget(Budget::Renderer).regionBytes = 16 * 1024;
  r.budget(Budget::Level).regionBytes = 16 * 1024;  // Asset: halts
  r.settings.refusalReportFrames = 3;
  std::unique_ptr<Manager> m = freshManager();
  CHECK(m->start(r));

  // Refuse vs halt, by kind: a render budget (and an asset budget the game
  // made refuse) answer null and count it; a logic budget halts, and so does
  // an asset budget as the engine registers it (Level).
  std::vector<void*> props = fill(*m, kRender, 1000);
  CHECK(!props.empty() && lockBalanced());
  CHECK(statsOf(*m, "props").refused == 1);
  CHECK(anyLine("[MEM] refused name=props size=1000 caller=0x500 frame=0"));
  std::vector<void*> sounds = fill(*m, kAsset, 1000);
  CHECK(!sounds.empty() && statsOf(*m, "sounds").refused == 1);
  std::vector<void*> renderer = fill(*m, Budget::Renderer, 1000);
  CHECK(!renderer.empty() && statsOf(*m, "Renderer").refused == 1);
  CHECK(throwsFailure(
      [&] {
        Scope s(*m, kGame);
        for (;;) m->allocate(1000, 16, 0x600);
      },
      "budget full"));
  CHECK(anyLine("[MEM] FAIL budget full name=game size=1000 caller=0x600"));
  CHECK(statsOf(*m, "game").refused == 0);
  CHECK(throwsFailure(
      [&] {
        Scope s(*m, Budget::Level);
        for (;;) m->allocate(1000, 16, 0x601);
      },
      "budget full"));
  CHECK(statsOf(*m, "Level").refused == 0);
  CHECK(lockBalanced());

  // The nothrow new (allocate) answers null; the throwing new
  // (allocateOrFail) cannot, and fails - that call site is a bug to convert.
  {
    Scope s(*m, kRender);
    CHECK(m->allocate(4000, 16, 0x510) == nullptr);
    CHECK(m->allocate(4000, 64, 0x510) == nullptr);
    CHECK(throwsFailure([&] { m->allocateOrFail(4000, 16, 0x520); },
                        "throwing new in a refusing budget"));
  }
  CHECK(anyLine("[MEM] FAIL throwing new in a refusing budget name=props "
                "size=4000 caller=0x520"));
  CHECK(statsOf(*m, "props").refused == 4);  // 1 + 2 nulls + the throwing one
  CHECK(lockBalanced());
  // With room, allocateOrFail is allocate (here System's, unscoped).
  m->release(m->allocateOrFail(0, 16, 0));
  void* gameBlock;
  {
    Scope s(*m, kRender);
    m->release(props.back());
    props.pop_back();
    gameBlock = m->allocateOrFail(500, 16, 0);
    CHECK(inRegion(*m, gameBlock, kRender));
  }

  // A refused realloc leaves the block as it was and answers null.
  std::memset(gameBlock, 0x5A, 500);
  CHECK(m->reallocate(gameBlock, 12000, 0x530) == nullptr);
  CHECK(allBytes(gameBlock, 500, 0x5A) && inRegion(*m, gameBlock, kRender));
  CHECK(statsOf(*m, "props").refused == 5);
  CHECK(anyLine("[MEM] refused name=props size=12000 caller=0x530"));

  // The line: once per call site per refusalReportFrames frames; another site
  // has its own; the count goes on regardless.
  const int before = countLines("caller=0x510");
  CHECK(before == 1);
  {
    Scope s(*m, kRender);
    for (int i = 0; i < 5; ++i) CHECK(m->allocate(9000, 16, 0x510) == nullptr);
    CHECK(countLines("caller=0x510") == 1);
    m->frameStart();
    m->frameStart();
    CHECK(m->allocate(9000, 16, 0x510) == nullptr);
    CHECK(countLines("caller=0x510") == 1);  // 2 frames < 3
    m->frameStart();
    CHECK(m->frame() == 3);
    CHECK(m->allocate(9000, 16, 0x510) == nullptr);
    CHECK(countLines("caller=0x510") == 2);
    CHECK(anyLine("[MEM] refused name=props size=9000 caller=0x510 frame=3"));
  }
  CHECK(statsOf(*m, "props").refused == 12);

  // The table and the stats carry the counts.
  fake.lines.clear();
  m->printTable();
  CHECK(anyLine("[MEM] budget props kind=Render region=16 heap=0 full=Refuse"));
  CHECK(anyLine("[MEM] budget sounds kind=Asset region=16 heap=0 full=Refuse"));
  CHECK(anyLine("[MEM] budget Level kind=Asset region=16 heap=0 full=Halt"));
  CHECK(anyLine("[MEM] budget game kind=Logic region=16 heap=0 full=Halt"));
  CHECK(anyLine(expectedLine(statsOf(*m, "props")).c_str()));
  CHECK(anyLine(" refused=12"));
  CHECK(anyLine(expectedLine(statsOf(*m, "game")).c_str()));

  // The many-call-sites case: past kMaxReportKeys sites, newcomers still get
  // their line.
  fake.lines.clear();
  {
    Scope s(*m, kRender);
    for (uintptr_t site = 0; site < Manager::kMaxReportKeys + 10; ++site) {
      CHECK(m->allocate(9000, 16, 0x9000 + site * 8) == nullptr);
    }
  }
  CHECK(countLines("[MEM] refused name=props size=9000") ==
        static_cast<int>(Manager::kMaxReportKeys + 10));

  // noteFallback: a line per key per refusalReportFrames frames.
  fake.lines.clear();
  int assetA = 0, assetB = 0;
  m->noteFallback("baked arena", "bag-1092", 2, &assetA);
  m->noteFallback("baked arena", "bag-1092", 2, &assetA);
  m->noteFallback("skin output", "soldier", 1, &assetB);
  CHECK(countLines("[MEM] fallback") == 2);
  CHECK(anyLine("[MEM] fallback baked arena asset=bag-1092 lod=2 frame=3"));
  CHECK(anyLine("[MEM] fallback skin output asset=soldier lod=1 frame=3"));
  for (int i = 0; i < 3; ++i) m->frameStart();
  m->noteFallback("baked arena", nullptr, -1, &assetA);
  CHECK(anyLine("[MEM] fallback baked arena asset=? lod=-1 frame=6"));
  CHECK(lockBalanced());

  // ...and at most kMaxFallbackLinesPerWindow lines a window, every key
  // together: the rest are dropped and counted, and the window's end says how
  // many (frames 9-11 here: refusalReportFrames is 3).
  for (int i = 0; i < 3; ++i) m->frameStart();  // a fresh window at frame 9
  fake.lines.clear();
  static int many[Manager::kMaxFallbackLinesPerWindow + 5];
  for (int& k : many) m->noteFallback("bbox cache", "bag-36", -1, &k);
  CHECK(countLines("[MEM] fallback bbox cache") ==
        static_cast<int>(Manager::kMaxFallbackLinesPerWindow));
  m->frameStart();
  m->frameStart();
  CHECK(countLines("suppressed") == 0);  // the window is still open
  m->frameStart();
  CHECK(anyLine("[MEM] fallback lines suppressed=5 frames=9-11"));
  m->noteFallback("bbox cache", "bag-36", -1, &many[0]);  // a new window
  CHECK(countLines("[MEM] fallback bbox cache") ==
        static_cast<int>(Manager::kMaxFallbackLinesPerWindow) + 1);
  CHECK(lockBalanced());

  for (void* p : props) m->release(p);
  for (void* p : sounds) m->release(p);
  for (void* p : renderer) m->release(p);
  m->release(gameBlock);
  CHECK(statsOf(*m, "props").used == 0 && statsOf(*m, "sounds").used == 0);
}

// ---- 6. measure mode: the headroom System keeps ----------------------------

void headroom() {
  installThrowingFailure();
  fake.lines.clear();
  Registry r;
  r.budget(kRender, "props", Kind::Render);    // measure mode, refuses
  r.budget(kMeasured, "measured");             // measure mode, halts
  r.budget(kAsset, "own", Kind::Render).regionBytes = 64 * 1024;  // own region
  r.settings.systemBytes = 256 * 1024;
  r.settings.refuseHeadroomBytes = 64 * 1024;
  std::unique_ptr<Manager> m = freshManager();
  CHECK(m->start(r));
  const size_t systemCap = statsOf(*m, "System").cap;
  CHECK(systemCap == 256 * 1024);

  // A refusing budget served from System's region stops at the headroom...
  std::vector<void*> props = fill(*m, kRender, 4096);
  CHECK(statsOf(*m, "props").refused == 1);
  const size_t freeAfter = m->totals().systemFree;
  CHECK(freeAfter >= 64 * 1024 && freeAfter < 64 * 1024 + 4096 + 64);
  CHECK(anyLine("[MEM] refused name=props size=4096"));
  // ...a request bigger than all that is left is refused too, not halted on...
  {
    Scope s(*m, kRender);
    CHECK(m->allocate(1u << 20, 16, 0) == nullptr);
  }
  // ...and the system - and a halting budget in measure mode - still get the
  // headroom it kept.
  void* sys = m->allocate(32 * 1024, 16, 0);
  CHECK(sys != nullptr && inRegion(*m, sys, Budget::System));
  void* measured;
  {
    Scope s(*m, kMeasured);
    measured = m->allocate(16 * 1024, 16, 0);
  }
  CHECK(measured != nullptr);
  // A refusing budget with a region of its own is not held to System's
  // headroom: its region is its own.
  {
    Scope s(*m, kAsset);
    void* own = m->allocate(48 * 1024, 16, 0);
    CHECK(own != nullptr && inRegion(*m, own, kAsset));
    m->release(own);
  }
  // A refused realloc in measure mode keeps its block.
  void* grown = props.back();
  std::memset(grown, 0x33, 4096);
  CHECK(m->reallocate(grown, 200 * 1024, 0x700) == nullptr);
  CHECK(allBytes(grown, 4096, 0x33));
  // Room again: it is served again.
  m->release(sys);
  m->release(measured);
  for (int i = 0; i < 12; ++i) {
    m->release(props.back());
    props.pop_back();
  }
  {
    Scope s(*m, kRender);
    void* again = m->allocate(4096, 16, 0);
    CHECK(again != nullptr && inRegion(*m, again, Budget::System));
    m->release(again);
  }
  for (void* p : props) m->release(p);
  CHECK(statsOf(*m, "props").used == 0);
  CHECK(lockBalanced());

  // A headroom of 0: a refusing budget in measure mode takes System's last
  // block, then refuses rather than halts.
  Registry r0;
  r0.budget(kRender, "props", Kind::Render);
  r0.settings.systemBytes = 64 * 1024;
  r0.settings.refuseHeadroomBytes = 0;
  std::unique_ptr<Manager> m0 = freshManager();
  CHECK(m0->start(r0));
  std::vector<void*> all = fill(*m0, kRender, 1024);
  CHECK(m0->totals().systemFree < 1024 + 64);
  CHECK(statsOf(*m0, "props").refused == 1);
  CHECK(throwsFailure([&] { m0->allocate(4096, 16, 0); }, "budget full"));
  for (void* p : all) m0->release(p);
}

// ---- 7. fragmentation, the defrag ring, the telemetry block ----------------

constexpr BudgetId kLong = Budget::FirstGame + 4;

struct Bulk {
  unsigned char bytes[1000];
};

// The fields of a region row and of regionStats agree.
bool rowIs(const TelemetryRegionRow& row, const Manager::RegionStats& s) {
  return row.id == s.id && row.cells == s.cells && row.capacity == s.capacity &&
         row.freeBytes == s.freeBytes && row.largestFree == s.largestFree &&
         row.freeBlocks == s.freeBlocks &&
         row.largestRequest == s.largestRequest &&
         std::memcmp(row.map, s.map, sizeof row.map) == 0;
}

void telemetry() {
  installThrowingFailure();
  fake.lines.clear();
  Registry r;
  r.budget(kGame, "game").regionBytes = 64 * 1024;  // a region of its own
  r.budget(kGame).extensionBytes = 256 * 1024;
  r.budget(kMeasured, "measured");                     // System's region
  r.budget(kRender, "props", Kind::Render).regionBytes = 32 * 1024;  // refuses
  r.budget(kLong, "averylongbudgetname");
  r.budget(Budget::FrameScratch).regionBytes = 4 * 1024;  // not a heap: no row
  r.settings.systemBytes = 512 * 1024;
  r.settings.defragBytesPerFrame = 8 * 1024;
  const PoolId bulk = r.pool(PoolDesc{"bulk", kGame, sizeof(Bulk), alignof(Bulk),
                                      1, 16, 8, Policy::Refuse});
  std::unique_ptr<Manager> m = freshManager();

  // Before start: no rows, and the block is left alone.
  static TelemetryBlock block;
  std::memset(&block, 0, sizeof block);
  CHECK(m->regionCount() == 0);
  writeTelemetry(block, *m, 1);
  CHECK(block.seq == 0 && block.version == 0);
  CHECK(m->start(r));

  // The rows: System, the budgets' own regions by id, the movable heap.
  CHECK(m->regionCount() == 4);
  const Manager::RegionStats sys = m->regionStats(0);
  CHECK(sys.id == Budget::System && sys.capacity == 512 * 1024);
  CHECK(sys.freeBytes == sys.capacity && sys.freeBlocks == 1);
  CHECK(sys.largestFree == sys.capacity - RegionHeap::kHeader);
  CHECK(sys.cells == 64 && allBytes(sys.map, sizeof sys.map, 0));
  CHECK(m->regionStats(1).id == kGame && m->regionStats(1).capacity == 64 * 1024);
  CHECK(m->regionStats(2).id == kRender && m->regionStats(2).capacity == 32 * 1024);
  const Manager::RegionStats heap = m->regionStats(3);
  CHECK(heap.id == Manager::kMovableHeapRegion && heap.capacity > 0);
  CHECK(heap.freeBytes == m->totals().heapFree && heap.freeBlocks == 1);
  const Manager::RegionStats none = m->regionStats(4);
  CHECK(none.id == 0 && none.capacity == 0 && none.cells == 0);
  CHECK(statsOf(*m, "props").onFull == OnFull::Refuse);
  CHECK(statsOf(*m, "game").onFull == OnFull::Halt);

  // Fragment System's region: eight blocks of 10 KB, every other one given
  // back (the last joins the free rest).
  std::vector<void*> blocks;
  for (int i = 0; i < 8; ++i) blocks.push_back(m->allocate(10 * 1024, 16, 0));
  for (int i = 1; i < 8; i += 2) m->release(blocks[i]);
  const size_t block10 = 10 * 1024 + RegionHeap::kHeader;  // 10256 bytes
  const Manager::RegionStats frag = m->regionStats(0);
  CHECK(frag.freeBlocks == 4);
  CHECK(frag.freeBytes == 512 * 1024 - 4 * block10);
  CHECK(frag.largestFree == 512 * 1024 - 7 * block10 - RegionHeap::kHeader);
  CHECK(frag.map[0] == 255);  // cells of 8 KB: block 0 covers cell 0
  CHECK(frag.map[1] == (block10 - 8192) * 255 / 8192);
  CHECK(frag.map[8] > 0 && frag.map[9] == 0 && frag.map[63] == 0);
  CHECK(frag.largestRequest == 10 * 1024 && m->largestRequest() == 10 * 1024);

  // The largest request, per region, served or refused.
  void* gameBlock;
  {
    Scope s(*m, kGame);
    gameBlock = m->allocate(20000, 16, 0);
  }
  CHECK(gameBlock != nullptr && m->regionStats(1).largestRequest == 20000);
  CHECK(m->largestRequest() == 20000);
  {
    Scope s(*m, kRender);
    CHECK(m->allocate(40 * 1024, 16, 0x7E) == nullptr);  // refused
  }
  CHECK(m->regionStats(2).largestRequest == 40 * 1024);
  CHECK(m->largestRequest() == 40 * 1024);
  void* measured;
  {
    Scope s(*m, kMeasured);  // served from System's region: its row
    measured = m->allocate(12000, 16, 0);
  }
  CHECK(m->regionStats(0).largestRequest == 12000);
  blocks[0] = m->reallocate(blocks[0], 30000, 0);  // a realloc that grows
  CHECK(blocks[0] != nullptr && m->regionStats(0).largestRequest == 30000);
  CHECK(m->largestRequest() == 40 * 1024);
  m->resetLargestRequests();
  CHECK(m->largestRequest() == 0);
  for (uint32_t i = 0; i < m->regionCount(); ++i) {
    CHECK(m->regionStats(i).largestRequest == 0);
  }

  // Pool extensions in the movable heap: each is bigger than the 8 KB step
  // budget, so every move is an oversize one, alone in its frame. The first
  // bucket fills first, then extension 1, 2, ...: entries 1..32 are the first
  // two extensions. (The pool is used raw - Pool<T> binds to instance() - and
  // its entries are not given back: the moves leave their addresses stale.)
  PoolCore& pool = m->pool(bulk);
  std::vector<void*> entries;
  for (int i = 0; i < 1 + 16 * 4; ++i) {
    entries.push_back(pool.acquire());
    CHECK(entries.back() != nullptr);
  }
  CHECK(m->regionStats(3).largestRequest > 8 * 1024);
  for (int i = 1; i <= 32; ++i) pool.release(entries[i]);  // two extensions
  const MovableHeap::Health before = m->heapHealth();
  CHECK(before.buckets == 4 && before.pinned == 0);
  // Four buckets bigger than the budget, but none has to move: no oversize work.
  CHECK(before.oversize == 0 && before.toMove == 0 && before.framesToCompact == 0);
  CHECK(before.debt == 0);  // empty, but not given back yet: no hole

  // Every frame start writes its step into the ring: frame f at f % 64.
  std::vector<Manager::DefragFrame> history(1);  // [f] = frame f's step
  std::vector<uint32_t> estimate(1);  // [f] = heapHealth's frames after frame f
  int movedFrames = 0;
  for (int f = 0; f < 70; ++f) {
    m->frameStart();
    const Manager::DefragFrame d =
        m->defragRing()[m->frame() % Manager::kDefragRingFrames];
    history.push_back(d);
    estimate.push_back(m->heapHealth().framesToCompact);
    CHECK(d.bytes == m->totals().movedLastFrame);
    CHECK((d.bytes == 0) == (d.buckets == 0));
    CHECK(d.oversize == d.buckets);  // every bucket is bigger than the budget
    CHECK(d.buckets <= 1);
    if (d.bytes != 0) ++movedFrames;
  }
  CHECK(movedFrames >= 1);
  CHECK(m->frame() == history.size() - 1);
  for (uint32_t f = m->frame() - Manager::kDefragRingFrames + 1; f <= m->frame();
       ++f) {
    const Manager::DefragFrame& d = m->defragRing()[f % Manager::kDefragRingFrames];
    CHECK(d.bytes == history[f].bytes && d.buckets == history[f].buckets);
  }
  // From the first frame start on (which gave extension 2 back) nothing but the
  // steps changes the heap: after every frame the estimate is exactly the
  // frames that still moved something.
  for (size_t f = 1; f < history.size(); ++f) {
    uint32_t left = 0;
    for (size_t g = f + 1; g < history.size(); ++g) left += history[g].bytes != 0;
    CHECK(estimate[f] == left);
  }
  CHECK(estimate[1] >= 1);
  const MovableHeap::Health after = m->heapHealth();
  CHECK(after.debt == 0 && after.blocked == 0);  // paid off, nothing fixed
  CHECK(after.framesToCompact == 0 && after.toMove == 0 && after.oversize == 0);
  CHECK(m->regionStats(3).freeBlocks == 1);

  // The block: everything above, under the seqlock; then a new request window.
  {
    Scope s(*m, kRender);
    void* prop = m->allocate(3000, 16, 0);
    CHECK(prop != nullptr);
    m->release(prop);
  }
  const size_t largest = m->largestRequest();
  CHECK(largest == 3000);
  std::vector<Manager::RegionStats> rows;
  for (uint32_t i = 0; i < m->regionCount(); ++i) rows.push_back(m->regionStats(i));
  const MovableHeap::Health health = m->heapHealth();
  const Manager::Totals totals = m->totals();
  writeTelemetry(block, *m, 7);
  CHECK(block.seq == 2);  // odd while writing, even after
  CHECK(block.frame == 7 && block.count == m->budgetCount());
  CHECK(block.version == kTelemetryVersion && block.blockBytes == sizeof block);
  CHECK(block.defragBytesPerFrame == 8 * 1024);
  CHECK(block.largestRequest == largest && m->largestRequest() == 0);
  CHECK(block.ringHead == m->frame() && block.regionCount == 4);
  CHECK(block.systemFree == totals.systemFree && block.heapFree == totals.heapFree);
  CHECK(block.heapLargest == totals.heapLargest &&
        block.allocsLastFrame == totals.allocsLastFrame && block.ptrs == totals.ptrs);
  CHECK(block.heapDebt == health.debt && block.heapBlocked == health.blocked);
  CHECK(block.heapPinned == health.pinned && block.heapOversize == health.oversize);
  CHECK(block.heapOversizeBytes == health.oversizeBytes);
  CHECK(block.heapBuckets == health.buckets &&
        block.heapFramesToCompact == health.framesToCompact);
  for (uint32_t i = 0; i < block.count; ++i) {
    const Manager::BudgetStats s = m->budgetStats(i);
    const TelemetryBudgetRow& row = block.budgets[i];
    CHECK(std::strncmp(row.name, s.name, sizeof row.name) == 0);
    CHECK(row.id == s.id && row.kind == uint8_t(s.kind));
    CHECK(row.onFull == uint8_t(s.onFull));
    CHECK(row.cap == s.cap && row.used == s.used && row.peak == s.peak);
    CHECK(row.exhaustions == s.exhaustions && row.refused == s.refused);
  }
  const TelemetryBudgetRow* longRow = nullptr;
  for (uint32_t i = 0; i < block.count; ++i) {
    if (block.budgets[i].id == kLong) longRow = &block.budgets[i];
  }
  CHECK(longRow != nullptr &&
        std::memcmp(longRow->name, "averylongbud", 12) == 0);  // no terminator
  CHECK(std::strncmp(block.budgets[0].name, "System", 12) == 0 &&
        block.budgets[0].name[6] == '\0');
  for (uint32_t i = 0; i < Manager::kDefragRingFrames; ++i) {
    CHECK(block.ring[i].bytes == m->defragRing()[i].bytes);
    CHECK(block.ring[i].buckets == m->defragRing()[i].buckets);
    CHECK(block.ring[i].oversize == m->defragRing()[i].oversize);
  }
  for (uint32_t i = 0; i < block.regionCount; ++i) {
    CHECK(rowIs(block.regions[i], rows[i]));  // read before the window restarted
  }
  CHECK(block.regions[2].largestRequest == 3000);
  CHECK(block.regions[3].id == 0xFFFF);
  writeTelemetry(block, *m, 17);
  CHECK(block.seq == 4 && block.frame == 17 && block.largestRequest == 0);
  CHECK(block.regions[2].largestRequest == 0);

  for (void* p : {blocks[0], blocks[2], blocks[4], blocks[6], gameBlock, measured}) {
    m->release(p);
  }
  CHECK(m->regionStats(0).freeBlocks == 1 && m->regionStats(1).freeBlocks == 1);
  CHECK(allBytes(m->regionStats(0).map, 64, 0));
  CHECK(lockBalanced());
}

}  // namespace

int main() {
  instanceScenario();
  startFailures();
  routing();
  sharedAccounting();
  refusals();
  headroom();
  telemetry();
  return finish("manager");
}
