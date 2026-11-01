// Added by TyraX: host verifier for the movable heap and its incremental
// defragmentation (inc/memory/movable_heap.hpp, src/memory/movable_heap.cpp).
// Built and run by tools/verify-memory.py, once with TYRA_MEMORY_CHECKS=0 and
// once with =1.
//
// A MovableHeap over a static arena, with a PtrRegistry installed as the
// globals the Ptrs use and, in some scenarios, PoolCores on top whose
// bucketMoved is the heap's move listener - the pieces as the manager wires
// them. Buckets carry a pattern that is checked after every move.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <new>
#include <utility>
#include <vector>

#include "memory/movable_heap.hpp"
#include "memory/pool.hpp"
#include "memory/ptr.hpp"
#include "memory/registry.hpp"
#include "verify-memory-check.hpp"

namespace {

constexpr size_t kArenaBytes = 1u << 20;
alignas(64) unsigned char g_arena[kArenaBytes];

constexpr BudgetId kBudgetA = Budget::FirstGame;
constexpr BudgetId kBudgetB = Budget::FirstGame + 1;
constexpr BudgetId kBudgetC = Budget::FirstGame + 2;

constexpr size_t kHeader = RegionHeap::kHeader;

size_t roundUp16(size_t n) { return (n + 15) & ~size_t(15); }

// The block a request of `bytes` takes from a big free block: header, payload
// rounded to 16, at least the smallest block.
size_t blockFor(size_t bytes) {
  const size_t block = kHeader + roundUp16(bytes);
  return block < 2 * kHeader ? 2 * kHeader : block;
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
  bool chance(uint32_t percent) { return below(100) < percent; }
};

// ---- the heap under test ---------------------------------------------------

struct Caps {
  uint32_t v[Budget::Max];
  Caps() { std::memset(v, 0, sizeof v); }
  Caps& set(BudgetId b, uint32_t cap) {
    v[b] = cap;
    return *this;
  }
};

Caps roomy() {
  Caps c;
  c.set(kBudgetA, kArenaBytes).set(kBudgetB, kArenaBytes);
  c.set(kBudgetC, kArenaBytes);
  return c;
}

// The arena as a MovableHeap with the Ptr registry installed. Declare it
// before anything that owns a Ptr: the Ptrs must go first.
class Stage {
 public:
  explicit Stage(const Caps& caps = roomy(), size_t bytes = kArenaBytes,
                 bool withPtrs = true, bool hook = true,
                 uint32_t ptrCapacity = 4096)
      : storage_((PtrTable::bytesFor(ptrCapacity) + 7) / 8 + 1) {
    std::memset(g_arena, 0xA5, sizeof g_arena);
    heap.init(g_arena, bytes, caps.v, withPtrs ? &registry : nullptr);
    const uintptr_t lo = reinterpret_cast<uintptr_t>(g_arena);
    registry.init(storage_.data(), ptrCapacity, lo, lo + bytes,
                  &MovableHeap::tokenAtCallback, &heap);
    if (withPtrs) {
      detail::g_ptrs = &registry;
      detail::g_movableLo = lo;
      detail::g_movableHi = lo + bytes;
    }
    if (hook) heap.installPinHook();
  }
  ~Stage() {
    detail::g_ptrs = nullptr;
    detail::g_movableLo = detail::g_movableHi = 0;
  }
  Stage(const Stage&) = delete;
  Stage& operator=(const Stage&) = delete;

  PtrTable& table() { return registry.table(); }

  MovableHeap heap;
  PtrRegistry registry;

 private:
  std::vector<uintptr_t> storage_;
};

// Every registered Ptr's row names the bucket its raw pointer is in now.
bool rowsConsistent(Stage& st) {
  bool ok = true;
  st.table().forEach([&](void* ptrAddress, uint32_t token) {
    uintptr_t raw;
    std::memcpy(&raw, ptrAddress, sizeof raw);
    if (st.heap.tokenAt(reinterpret_cast<const void*>(raw)) != token) {
      ok = false;
    }
  });
  return ok;
}

// ---- buckets with a pattern ------------------------------------------------

unsigned char patternByte(uint32_t token, size_t i) {
  return static_cast<unsigned char>(token * 131u + i * 7u + 1u);
}

void fillPattern(void* p, size_t from, size_t to, uint32_t token) {
  unsigned char* b = static_cast<unsigned char*>(p);
  for (size_t i = from; i < to; ++i) b[i] = patternByte(token, i);
}

bool patternOk(const void* p, size_t from, size_t to, uint32_t token) {
  const unsigned char* b = static_cast<const unsigned char*>(p);
  for (size_t i = from; i < to; ++i) {
    if (b[i] != patternByte(token, i)) return false;
  }
  return true;
}

struct Live {
  unsigned char* base;
  size_t bytes;
  size_t align;
  bool pinned;
  BudgetId budget;
  size_t from;  // the pattern covers [from, to)
  size_t to;
};

// Buckets the test asked the heap for, where the move listener last said they
// are, and what to check about them.
class Tracker {
 public:
  explicit Tracker(Stage& s) : st(s) {
    st.heap.setMoveListener(&Tracker::onMove, this);
  }
  ~Tracker() { st.heap.setMoveListener(nullptr, nullptr); }
  Tracker(const Tracker&) = delete;
  Tracker& operator=(const Tracker&) = delete;

  // A bucket with a fresh token; 0 when the heap refused.
  uint32_t alloc(size_t bytes, size_t align = 16, bool pinned = false,
                 BudgetId budget = kBudgetA, size_t from = 0,
                 size_t to = SIZE_MAX) {
    const uint32_t token = nextToken++;
    void* p = st.heap.allocateBucket(bytes, align, pinned, token, budget);
    if (p == nullptr) return 0;
    to = std::min(to, bytes);
    fillPattern(p, from, to, token);
    live[token] = Live{
        static_cast<unsigned char*>(p), bytes, align, pinned, budget, from, to};
    return token;
  }

  void release(uint32_t token) {
    st.heap.releaseBucket(live.at(token).base);
    live.erase(token);
  }

  unsigned char* base(uint32_t token) const { return live.at(token).base; }

  // Whether every bucket is where the heap says, with its bytes.
  bool intact() const {
    bool ok = st.heap.validate() && !sawGarbage && !sawUnknown;
    for (const auto& kv : live) {
      const Live& l = kv.second;
      ok = ok && st.heap.tokenAt(l.base) == kv.first;
      ok = ok && st.heap.tokenAt(l.base + l.bytes - 1) == kv.first;
      ok = ok && reinterpret_cast<uintptr_t>(l.base) % l.align == 0;
      ok = ok && patternOk(l.base, l.from, l.to, kv.first);
    }
    return ok && st.heap.bucketCount() == live.size();
  }

  Stage& st;
  std::map<uint32_t, Live> live;
  std::vector<uint32_t> order;  // the listener's calls, in order
  bool sawGarbage = false;      // a bucket's bytes were not at its new place
  bool sawUnknown = false;      // the listener named a bucket nobody made
  uint32_t nextToken = 1;

 private:
  static void onMove(uint32_t token, void* newBase, void* ctx) {
    Tracker* t = static_cast<Tracker*>(ctx);
    t->order.push_back(token);
    auto it = t->live.find(token);
    if (it == t->live.end()) {
      t->sawUnknown = true;
      return;
    }
    const Live& l = it->second;
    if (!patternOk(newBase, l.from, l.to, token)) t->sawGarbage = true;
    it->second.base = static_cast<unsigned char*>(newBase);
  }
};

// ---- pools on the movable heap ---------------------------------------------

// 24 bytes (with the checks; a pool strides 24 either way): its own index, a
// check on it, and a Ptr to another entry.
struct Entry {
  uint32_t index;
  uint32_t salt;
  Ptr<Entry> next;
  explicit Entry(uint32_t i) : index(i), salt(saltOf(i)) {}
  static uint32_t saltOf(uint32_t i) { return i * 2654435761u + 7u; }
};
static_assert(sizeof(Entry) <= 24, "an Entry fits its pool's stride");

// Stage + registry + pools whose extensions come from the stage's heap and
// whose buckets the heap's move listener tells about their moves.
class World {
 public:
  struct PoolBox {
    PoolDesc desc;
    PoolId id = 0;
    PoolCore core;
    std::vector<unsigned char> storage;
    unsigned char* first = nullptr;
  };

  World() { stage.heap.setMoveListener(&route, this); }
  ~World() {
#if TYRA_MEMORY_CHECKS
    detail::g_idHook = nullptr;
#endif
    stage.heap.setMoveListener(nullptr, nullptr);
  }

  PoolBox& addPool(const PoolDesc& d, Kind kind) {
    reg.budget(d.budget, "probe", kind);
    std::unique_ptr<PoolBox> box(new PoolBox());
    box->desc = d;
    box->id = reg.pool(d);
    const size_t unit = std::max<uint32_t>(d.align, 16);
    box->storage.assign(reg.poolFixedBytes(box->id) + unit, 0xCD);
    box->first = reinterpret_cast<unsigned char*>(
        (reinterpret_cast<uintptr_t>(box->storage.data()) + unit - 1) / unit *
        unit);
    box->core.init(box->id, box->desc, reg, box->first, &stage.heap);
    pools.push_back(std::move(box));
#if TYRA_MEMORY_CHECKS
    detail::g_idHook = &idHook_;
#endif
    return *pools.back();
  }

  Stage stage;
  Registry reg;
  std::vector<std::unique_ptr<PoolBox>> pools;
  int foreignMoves = 0;  // moves of buckets that are no pool's

 private:
  static void route(uint32_t token, void* newBase, void* ctx) {
    World* w = static_cast<World*>(ctx);
    const uint32_t pool = (token >> 16) - 1;  // wraps for a token below 65536
    if (pool < w->pools.size()) {
      w->pools[pool]->core.bucketMoved(static_cast<uint16_t>(token & 0xFFFF),
                                       newBase);
    } else {
      ++w->foreignMoves;
    }
  }

#if TYRA_MEMORY_CHECKS
  static bool hookLive(const void* raw, const Id& id, void* ctx) {
    World* w = static_cast<World*>(ctx);
    return id.pool < w->pools.size() && w->pools[id.pool]->core.valid(id, raw);
  }
  static Id hookId(const void* raw, void* ctx) {
    World* w = static_cast<World*>(ctx);
    for (auto& box : w->pools) {
      for (uint32_t b = 0; b < box->core.bucketCount(); ++b) {
        for (uint32_t i = 0; i < box->core.countIn(uint16_t(b)); ++i) {
          if (box->core.entryAt(uint16_t(b), i) == raw) {
            return box->core.idOf(raw);
          }
        }
      }
    }
    return kEmptyId;
  }
  detail::IdHook idHook_ = {&hookLive, &hookId, this};
#endif
};

Entry* makeEntry(World::PoolBox& p, uint32_t index) {
  void* raw = p.core.acquire();
  return raw != nullptr ? new (raw) Entry(index) : nullptr;
}

void destroyEntry(World::PoolBox& p, Entry* e) {
  e->~Entry();
  p.core.release(e);
}

std::vector<Entry*> liveEntries(const World::PoolBox& p) {
  std::vector<Entry*> out;
  for (uint32_t b = 0; b < p.core.bucketCount(); ++b) {
    for (uint32_t i = 0; i < p.core.countIn(uint16_t(b)); ++i) {
      if (p.core.isLive(uint16_t(b), i)) {
        out.push_back(static_cast<Entry*>(p.core.entryAt(uint16_t(b), i)));
      }
    }
  }
  return out;
}

Entry* findEntry(const World::PoolBox& p, uint32_t index) {
  for (Entry* e : liveEntries(p)) {
    if (e->index == index) return e;
  }
  return nullptr;
}

// Every live entry still holds its own index (and its check).
bool allEntriesHoldTheirIndex(const World::PoolBox& p) {
  std::vector<uint32_t> seen;
  for (Entry* e : liveEntries(p)) {
    if (e->salt != Entry::saltOf(e->index)) return false;
    seen.push_back(e->index);
  }
  std::sort(seen.begin(), seen.end());
  return std::adjacent_find(seen.begin(), seen.end()) == seen.end();
}

// The pool's buckets are where the heap says: the token of each bucket's
// first entry is the bucket's.
bool bucketsMatchTokens(const World& w, const World::PoolBox& p) {
  for (uint32_t b = 1; b < p.core.bucketCount(); ++b) {
    const void* at = p.core.entryAt(uint16_t(b), 0);
    if (at == nullptr) continue;  // a gap
    if (w.stage.heap.tokenAt(at) != bucketToken(p.id, uint16_t(b))) {
      return false;
    }
  }
  return true;
}

// ---- the brief -------------------------------------------------------------

void testBrief() {
  World w;
  World::PoolBox& pool = w.addPool(
      {"entry", kBudgetA, 24, 8, 4, 4, 8, Policy::Refuse}, Kind::Logic);
  MovableHeap& heap = w.stage.heap;

  // 36 entries: the first bucket's 4 and 8 extensions of 4.
  Entry* e[36];
  for (uint32_t i = 0; i < 36; ++i) {
    e[i] = makeEntry(pool, i);
    CHECK(e[i] != nullptr);
  }
  CHECK(pool.core.live() == 36 && pool.core.capacity() == 36);
  CHECK(pool.core.bucketCount() == 9 && heap.bucketCount() == 8);
  CHECK(heap.usedBy(kBudgetA) == 8 * blockFor(w.reg.bucketBytes(pool.id, 4)));

  // A ring through all of them, and Ptrs held outside to every 3rd.
  for (uint32_t i = 0; i < 36; ++i) e[i]->next = Ptr<Entry>(e[(i + 1) % 36]);
  std::vector<Ptr<Entry>> held;
  held.reserve(12);
  for (uint32_t i = 0; i < 36; i += 3) held.emplace_back(e[i]);

  // Release every entry of extensions 1, 3 and 5 (entries 4n .. 4n+3).
  std::vector<uint32_t> survivors;
  for (uint32_t i = 0; i < 36; ++i) {
    const uint32_t ext = i < 4 ? 0 : (i - 4) / 4 + 1;
    if (ext == 1 || ext == 3 || ext == 5) {
      if (i % 3 == 0) held[i / 3].reset();
      destroyEntry(pool, e[i]);
      e[i] = nullptr;
    } else {
      survivors.push_back(i);
    }
  }
  for (size_t s = 0; s < survivors.size(); ++s) {
    e[survivors[s]]->next =
        Ptr<Entry>(e[survivors[(s + 1) % survivors.size()]]);
  }
  pool.core.releaseEmptyExtensions();
  CHECK(heap.bucketCount() == 6);  // 8 - extensions 3 and 5 (1 is the spare)
  CHECK(heap.validate());

  const uint32_t rowsBefore = w.stage.table().size();
  const size_t usedBefore = heap.usedBy(kBudgetA);
  const size_t freeBefore = heap.freeBytes();
  const size_t largestBefore = heap.largestFree();
  size_t movedTotal = 0;
  for (int f = 0; f < 20; ++f) movedTotal += heap.step(64 * 1024);

  CHECK(heap.largestFree() > largestBefore);
  CHECK(movedTotal > 0 && heap.movedLastStep() == 0);
  CHECK(heap.usedBy(kBudgetA) == usedBefore);  // moving costs nothing
  CHECK(heap.freeBytes() == freeBefore);
  CHECK(heap.largestFree() + kHeader == heap.freeBytes());  // one free block
  CHECK(w.stage.table().size() == rowsBefore);
  CHECK(rowsConsistent(w.stage));
  CHECK(heap.validate());
  CHECK(allEntriesHoldTheirIndex(pool));
  CHECK(bucketsMatchTokens(w, pool));
  CHECK(w.foreignMoves == 0);

  // Ptrs outside, to entries in buckets that moved.
  bool heldRight = true;
  for (size_t j = 0; j < held.size(); ++j) {
    const uint32_t index = uint32_t(3 * j);
    const uint32_t ext = index < 4 ? 0 : (index - 4) / 4 + 1;
    if (ext == 1 || ext == 3 || ext == 5) continue;  // reset above
    heldRight = heldRight && held[j].get() == findEntry(pool, index);
  }
  CHECK(heldRight);

  // Ptrs inside buckets that moved: the ring still visits every survivor.
  bool ring = true;
  Entry* at = findEntry(pool, survivors[0]);
  for (size_t s = 0; s < survivors.size(); ++s) {
    ring = ring && at != nullptr && at->index == survivors[s];
    at = at != nullptr ? at->next.get() : nullptr;
  }
  CHECK(ring && at == findEntry(pool, survivors[0]));

  // The pool goes on working on the moved buckets.
  Entry* extra = makeEntry(pool, 100);
  CHECK(extra != nullptr && extra->index == 100);
  destroyEntry(pool, extra);
  for (Entry* x : liveEntries(pool)) x->next.reset();
  held.clear();
  CHECK(w.stage.table().size() == 0);
}

// ---- the per-step budget ---------------------------------------------------

void testStepBudget() {
  Stage st;
  Tracker t(st);
  MovableHeap& h = st.heap;
  const size_t big = 40 * 1024;

  // Three 40 KB buckets behind a hole: one moves a step.
  const uint32_t x = t.alloc(4000);
  const uint32_t a = t.alloc(big), b = t.alloc(big), c = t.alloc(big);
  CHECK(x && a && b && c);
  unsigned char* const xAt = t.base(x);
  unsigned char* const bAt = t.base(b);
  unsigned char* const cAt = t.base(c);
  t.release(x);

  CHECK(h.step(0) == 0 && h.movedLastStep() == 0);  // no budget, no move
  CHECK(t.order.empty());

  CHECK(h.step(64 * 1024) == blockFor(big));
  CHECK(h.movedLastStep() == blockFor(big));
  CHECK(t.order.size() == 1 && t.order[0] == a);
  CHECK(t.base(a) == xAt);  // it landed where the hole began
  CHECK(t.base(b) == bAt && t.base(c) == cAt);
  CHECK(t.intact());

  t.order.clear();
  CHECK(h.step(64 * 1024) == blockFor(big));
  CHECK(t.order.size() == 1 && t.order[0] == b);
  CHECK(t.base(b) == xAt + blockFor(big));
  CHECK(h.step(64 * 1024) == blockFor(big));
  CHECK(t.order.size() == 2 && t.order[1] == c);
  CHECK(t.base(c) == xAt + 2 * blockFor(big));
  CHECK(h.step(64 * 1024) == 0 && h.movedLastStep() == 0);
  CHECK(h.largestFree() + kHeader == h.freeBytes());
  CHECK(t.intact());
}

void testStepBudgetEdges() {
  const size_t big = 40 * 1024;

  {  // a budget of exactly two buckets moves two
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(4000);
    const uint32_t a = t.alloc(big), b = t.alloc(big), c = t.alloc(big);
    unsigned char* const cAt = t.base(c);
    t.release(x);
    CHECK(st.heap.step(2 * blockFor(big)) == 2 * blockFor(big));
    CHECK(t.order.size() == 2 && t.order[0] == a && t.order[1] == b);
    CHECK(t.base(c) == cAt);
    CHECK(t.intact());
  }
  {  // one byte short of two: one moves
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(4000);
    t.alloc(big);
    t.alloc(big);
    t.release(x);
    CHECK(st.heap.step(2 * blockFor(big) - 1) == blockFor(big));
    CHECK(t.intact());
  }
  {  // a bucket bigger than the whole budget moves alone, in a step of its own
    Stage st;
    Tracker t(st);
    const size_t huge = 100 * 1024;
    const uint32_t x = t.alloc(4000);
    const uint32_t a = t.alloc(huge), y = t.alloc(1000);
    unsigned char* const yAt = t.base(y);
    t.release(x);
    const size_t moved = st.heap.step(64 * 1024);
    CHECK(moved >= huge && moved == blockFor(huge));
    CHECK(t.order.size() == 1 && t.order[0] == a);
    CHECK(t.base(y) == yAt);  // the next one waits for the next step
    CHECK(st.heap.step(64 * 1024) == blockFor(1000));
    CHECK(t.base(y) == yAt - blockFor(4000));
    CHECK(t.intact());
  }
  {  // ... and it waits for a step of its own: smaller buckets go first
    Stage st;
    Tracker t(st);
    const size_t huge = 100 * 1024;
    const uint32_t x = t.alloc(4000);
    const uint32_t s1 = t.alloc(20 * 1024), a = t.alloc(huge),
                   s2 = t.alloc(1000);
    unsigned char* const aAt = t.base(a);
    t.release(x);
    CHECK(st.heap.step(64 * 1024) == blockFor(20 * 1024));
    CHECK(t.order.size() == 1 && t.order[0] == s1);
    CHECK(t.base(a) == aAt);
    CHECK(st.heap.step(64 * 1024) == blockFor(huge));  // alone now
    CHECK(t.order.size() == 2 && t.order[1] == a);
    CHECK(st.heap.step(64 * 1024) == blockFor(1000));
    CHECK(t.order.size() == 3 && t.order[2] == s2);
    CHECK(st.heap.step(64 * 1024) == 0);
    CHECK(t.intact());
  }
  {  // a budget smaller than any bucket still makes progress, a bucket a step
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(4000);
    const uint32_t a = t.alloc(3000), b = t.alloc(3000);
    t.release(x);
    CHECK(st.heap.step(1) == blockFor(3000));
    CHECK(t.order.size() == 1 && t.order[0] == a);
    CHECK(st.heap.step(1) == blockFor(3000));
    CHECK(t.order.size() == 2 && t.order[1] == b);
    CHECK(st.heap.step(1) == 0);
    CHECK(t.intact());
  }
  {  // holes between buckets: each slide merges the hole it leaves with the
     // next
    Stage st;
    Tracker t(st);
    const uint32_t h1 = t.alloc(500), b1 = t.alloc(700);
    const uint32_t h2 = t.alloc(900), b2 = t.alloc(1100);
    unsigned char* const first = t.base(h1);
    t.release(h1);
    t.release(h2);
    CHECK(st.heap.step(1 << 20) == blockFor(700) + blockFor(1100));
    CHECK(t.base(b1) == first);
    CHECK(t.base(b2) == first + blockFor(700));
    CHECK(t.intact());
  }
  {  // nothing to do: no hole, no move
    Stage st;
    Tracker t(st);
    t.alloc(1000);
    t.alloc(2000);
    CHECK(st.heap.step(1 << 20) == 0);
    CHECK(t.order.empty());
  }
  {  // an empty heap
    Stage st;
    CHECK(st.heap.step(1 << 20) == 0 && st.heap.validate());
  }
}

void testMoveLimit() {
  Stage st;
  Tracker t(st);
  const uint32_t n = MovableHeap::kMaxMovesPerStep + 8;
  std::vector<uint32_t> holes, kept;
  for (uint32_t i = 0; i < n; ++i) {
    holes.push_back(t.alloc(100));
    kept.push_back(t.alloc(100));
  }
  // A Ptr into the last bucket, which only the second step reaches.
  Ptr<uint32_t> last(reinterpret_cast<uint32_t*>(t.base(kept.back()) + 8));
  unsigned char* const lastAt = t.base(kept.back());
  for (uint32_t h : holes) t.release(h);

  CHECK(st.heap.step(1 << 20) == MovableHeap::kMaxMovesPerStep * blockFor(100));
  CHECK(t.order.size() == MovableHeap::kMaxMovesPerStep);
  CHECK(reinterpret_cast<unsigned char*>(last.get()) == lastAt + 8);
  CHECK(rowsConsistent(st) && t.intact());

  CHECK(st.heap.step(1 << 20) == 8 * blockFor(100));
  CHECK(t.order.size() == n);
  CHECK(reinterpret_cast<unsigned char*>(last.get()) ==
        t.base(kept.back()) + 8);
  CHECK(last.get() != reinterpret_cast<uint32_t*>(lastAt + 8));
  CHECK(st.heap.step(1 << 20) == 0);
  CHECK(st.heap.largestFree() + kHeader == st.heap.freeBytes());
  CHECK(rowsConsistent(st) && t.intact());

  // The listener ran in address order.
  bool ascending = true;
  for (size_t i = 0; i < kept.size(); ++i) {
    ascending = ascending && t.order[i] == kept[i];
  }
  CHECK(ascending);
}

// ---- what never moves ------------------------------------------------------

struct Probe {
  uint32_t v[4];
};

void testPin() {
  Stage st;
  Tracker t(st);
  MovableHeap& h = st.heap;

  const uint32_t x = t.alloc(2000);
  const uint32_t a = t.alloc(3000), b = t.alloc(3000), c = t.alloc(3000);
  unsigned char* const xAt = t.base(x);
  unsigned char* const bAt = t.base(b);
  unsigned char* const cAt = t.base(c);
  t.release(x);

  Ptr<Probe> inB(reinterpret_cast<Probe*>(bAt + 64));
  CHECK(st.table().size() == 1);
  {
    Pin<Probe> hold(inB);
    CHECK(h.step(1 << 20) == blockFor(3000));  // only a, in front of the hole
    CHECK(t.base(a) == xAt);
    CHECK(t.base(b) == bAt && t.base(c) == cAt);  // b pinned, c behind it
    CHECK(reinterpret_cast<unsigned char*>(inB.get()) == bAt + 64);
    {
      Pin<Probe> again(inB);  // pins nest
      CHECK(h.step(1 << 20) == 0);
    }
    CHECK(h.step(1 << 20) == 0);  // one pin left
    CHECK(t.base(b) == bAt);
  }
  CHECK(t.intact());

  // Let go: b moves, and so does c behind it, and the Ptr follows.
  CHECK(h.step(1 << 20) == 2 * blockFor(3000));
  CHECK(t.base(b) == xAt + blockFor(3000));
  CHECK(reinterpret_cast<unsigned char*>(inB.get()) == t.base(b) + 64);
  CHECK(rowsConsistent(st) && t.intact());
}

void testPinEdges() {
  {  // a Pin on memory that is in no bucket does nothing
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(2000);
    const uint32_t a = t.alloc(3000);
    unsigned char* const xAt = t.base(x);
    t.release(x);
    Ptr<Probe> inHole(reinterpret_cast<Probe*>(xAt));
    Ptr<Probe> empty;
    {
      Pin<Probe> p1(inHole), p2(empty);
      CHECK(st.heap.step(1 << 20) == blockFor(3000));
    }
    CHECK(t.base(a) == xAt);
    CHECK(t.intact());
  }
  {  // a pin keeps its bucket for any address inside it, first byte to last
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(2000);
    const uint32_t a = t.alloc(3000);
    unsigned char* const aAt = t.base(a);
    t.release(x);
    st.heap.pin(aAt);
    st.heap.pin(aAt + 2999);
    CHECK(st.heap.step(1 << 20) == 0);
    st.heap.unpin(aAt + 1500);
    CHECK(st.heap.step(1 << 20) == 0);
    st.heap.unpin(aAt + 7);
    CHECK(st.heap.step(1 << 20) == blockFor(3000));
    st.heap.unpin(aAt + 7);  // more unpins than pins: ignored
    CHECK(t.intact());
  }
  {  // pins are per bucket
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(2000);
    const uint32_t a = t.alloc(3000), b = t.alloc(3000);
    t.release(x);
    st.heap.pin(t.base(b));  // b only: a is free to move
    CHECK(st.heap.step(1 << 20) == blockFor(3000));
    CHECK(t.order.size() == 1 && t.order[0] == a);
    st.heap.unpin(t.base(b));
    CHECK(st.heap.step(1 << 20) == blockFor(3000));
    CHECK(t.intact());
  }
  {  // a pinned bucket that is released takes its pins with it
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(2000);
    const uint32_t a = t.alloc(3000);
    unsigned char* const aAt = t.base(a);
    st.heap.pin(aAt + 4);
    st.heap.pin(aAt + 4);
    t.release(a);
    CHECK(st.heap.validate());
    // Another bucket with the same token is not held by the stale pins.
    void* again = st.heap.allocateBucket(3000, 16, false, a, kBudgetA);
    CHECK(again == aAt);
    st.heap.unpin(aAt + 4);  // the old Pin lets go: nothing to undo
    t.release(x);
    CHECK(st.heap.step(1 << 20) == blockFor(3000));
    CHECK(st.heap.validate());
  }
  {  // a pin on every live bucket at once
    Stage st;
    Tracker t(st);
    std::vector<uint32_t> all;
    for (uint32_t i = 0; i < MovableHeap::kMaxBuckets; ++i) {
      all.push_back(t.alloc(16));
      CHECK(all.back() != 0);
    }
    for (uint32_t tok : all) st.heap.pin(t.base(tok));
    CHECK(st.heap.validate());
    for (uint32_t tok : all)
      st.heap.pin(t.base(tok) + 8);  // nested: no new row
    for (uint32_t tok : all) st.heap.unpin(t.base(tok) + 8);
    for (uint32_t tok : all) st.heap.unpin(t.base(tok));
    CHECK(st.heap.validate() && t.intact());
  }
}

void testNeverMoves() {
  {  // allocated pinned: never moves, though a hole is right in front of it
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(2000);
    const uint32_t p = t.alloc(3000, 16, true), b = t.alloc(3000);
    const uint32_t y = t.alloc(100);
    unsigned char* const pAt = t.base(p);
    unsigned char* const bAt = t.base(b);
    t.release(x);
    for (int i = 0; i < 4; ++i) CHECK(st.heap.step(1 << 20) == 0);
    CHECK(t.base(p) == pAt && t.base(b) == bAt && t.order.empty());
    CHECK(t.intact());

    // The bucket behind it moves once there is a hole in front of that.
    t.release(b);
    CHECK(st.heap.step(1 << 20) == blockFor(100));
    CHECK(t.base(y) == bAt && t.base(p) == pAt);
    CHECK(t.intact());
  }
  {  // aligned above 16: not slidable, pinned or not
    Stage st;
    Tracker t(st);
    const uint32_t x = t.alloc(2000);
    const uint32_t o = t.alloc(3000, 128), b = t.alloc(3000);
    CHECK(o && b);
    unsigned char* const oAt = t.base(o);
    unsigned char* const bAt = t.base(b);
    t.release(x);
    for (int i = 0; i < 4; ++i) CHECK(st.heap.step(1 << 20) == 0);
    CHECK(t.base(o) == oAt && t.base(b) == bAt);
    CHECK(reinterpret_cast<uintptr_t>(oAt) % 128 == 0);
    t.release(o);  // the hole it leaves lets b go
    CHECK(st.heap.step(1 << 20) == blockFor(3000));
    CHECK(t.base(b) < bAt);
    CHECK(t.intact());
  }
  {  // the pools: a Render budget's extensions and an over-aligned type's
    World w;
    World::PoolBox& logic = w.addPool(
        {"logic", kBudgetA, 24, 8, 1, 1, 4, Policy::Refuse}, Kind::Logic);
    World::PoolBox& render = w.addPool(
        {"render", kBudgetB, 24, 8, 1, 1, 4, Policy::Refuse}, Kind::Render);
    World::PoolBox& wide = w.addPool(
        {"wide", kBudgetC, 64, 64, 1, 1, 4, Policy::Refuse}, Kind::Logic);
    MovableHeap& heap = w.stage.heap;

    // A filler at the bottom, then L1 R1 W1 R2 W2 in that order.
    void* filler = heap.allocateBucket(500, 16, false, 0x100, kBudgetA);
    CHECK(filler != nullptr);
    logic.core.acquire();
    logic.core.acquire();
    render.core.acquire();
    render.core.acquire();
    wide.core.acquire();
    wide.core.acquire();
    render.core.acquire();
    wide.core.acquire();
    CHECK(heap.bucketCount() == 6);

    void* const l1 = logic.core.entryAt(1, 0);
    void* const r1 = render.core.entryAt(1, 0);
    void* const r2 = render.core.entryAt(2, 0);
    void* const w1 = wide.core.entryAt(1, 0);
    void* const w2 = wide.core.entryAt(2, 0);
    CHECK(reinterpret_cast<uintptr_t>(w1) % 64 == 0);
    CHECK(reinterpret_cast<uintptr_t>(w2) % 64 == 0);

    heap.releaseBucket(filler);  // a hole in front of L1
    const size_t l1Block = blockFor(w.reg.bucketBytes(logic.id, 1));
    CHECK(heap.step(1 << 20) == l1Block);  // L1 slides; R1 stays behind it
    CHECK(logic.core.entryAt(1, 0) != l1);
    CHECK(logic.core.entryAt(1, 0) == filler);  // where the filler was
    CHECK(render.core.entryAt(1, 0) == r1 && render.core.entryAt(2, 0) == r2);
    CHECK(wide.core.entryAt(1, 0) == w1 && wide.core.entryAt(2, 0) == w2);
    for (int i = 0; i < 4; ++i) CHECK(heap.step(1 << 20) == 0);
    CHECK(render.core.entryAt(1, 0) == r1 && wide.core.entryAt(2, 0) == w2);
    CHECK(heap.validate() && bucketsMatchTokens(w, logic));
    CHECK(bucketsMatchTokens(w, render) && bucketsMatchTokens(w, wide));
  }
}

// A bucket that cannot move, with a hole right in front of it, does not end the
// walk: the buckets behind it still move, even when it is bigger than what is
// left of the step's budget.
void testUnmovableDoesNotStopTheWalk() {
  for (int kind = 0; kind < 3; ++kind) {  // pinned, held by a Pin, over-aligned
    Stage st;
    Tracker t(st);
    const size_t huge = 70 * 1024;
    const uint32_t x = t.alloc(2000), s = t.alloc(1024);
    const uint32_t u = t.alloc(huge, kind == 2 ? 128 : 16, kind == 0);
    const uint32_t m = t.alloc(huge);  // movable, but nothing is in front of it
    const uint32_t y = t.alloc(2000), b = t.alloc(3000);
    CHECK(x && s && u && m && y && b);
    unsigned char* const uAt = t.base(u);
    unsigned char* const mAt = t.base(m);
    if (kind == 1) st.heap.pin(uAt + 100);
    t.release(x);
    t.release(y);

    // s slides into x's place; u has a hole in front of it now and stays, and m
    // has none; b slides into y's. Neither u nor m, both bigger than what is
    // left of the budget, ends the walk.
    CHECK(st.heap.step(64 * 1024) == blockFor(1024) + blockFor(3000));
    CHECK(t.order.size() == 2 && t.order[0] == s && t.order[1] == b);
    CHECK(t.base(u) == uAt && t.base(m) == mAt);
    CHECK(t.intact());
    if (kind == 1) st.heap.unpin(uAt + 100);
  }
}

// A Ptr that lives in the last bytes of a bucket moves with it.
void testPtrAtTheEnd() {
  Stage st;
  Tracker t(st);
  constexpr size_t kBytes = 112;
  constexpr size_t kTail = kBytes - 16;  // a Ptr fits in the last 16 bytes
  const uint32_t x = t.alloc(2000);
  const uint32_t a = t.alloc(kBytes, 16, false, kBudgetA, 0, kTail);
  const uint32_t b = t.alloc(kBytes, 16, false, kBudgetA, 0, kTail);
  unsigned char* const aAt = t.base(a);
  unsigned char* const bAt = t.base(b);
  new (aAt + kTail) Ptr<uint32_t>(reinterpret_cast<uint32_t*>(bAt + 104));
  new (bAt + kTail) Ptr<uint32_t>(reinterpret_cast<uint32_t*>(aAt + 108));
  CHECK(st.table().size() == 2);
  t.release(x);

  CHECK(st.heap.step(1 << 20) == 2 * blockFor(kBytes));
  Ptr<uint32_t>* inA = reinterpret_cast<Ptr<uint32_t>*>(t.base(a) + kTail);
  Ptr<uint32_t>* inB = reinterpret_cast<Ptr<uint32_t>*>(t.base(b) + kTail);
  CHECK(t.base(a) < aAt && t.base(b) < bAt);
  CHECK(inA->get() == reinterpret_cast<uint32_t*>(t.base(b) + 104));
  CHECK(inB->get() == reinterpret_cast<uint32_t*>(t.base(a) + 108));
  CHECK(st.table().size() == 2 && rowsConsistent(st) && t.intact());
  inA->~Ptr();
  inB->~Ptr();
  CHECK(st.table().size() == 0);
}

// ---- extension caps --------------------------------------------------------

void testCaps() {
  {
    Caps caps;
    caps.set(kBudgetA, 4096).set(kBudgetB, 2048);  // C has none
    Stage st(caps);
    MovableHeap& h = st.heap;
    Tracker t(st);

    // The brief's refusal: past the cap, by a request that cannot fit.
    CHECK(h.allocateBucket(4096 + 16, 16, false, 900, kBudgetA) == nullptr);
    CHECK(h.allocateBucket(4096, 16, false, 900, kBudgetA) ==
          nullptr);  // +header
    CHECK(h.usedBy(kBudgetA) == 0 && h.bucketCount() == 0);

    // Four blocks of 1024 fill the cap to the byte; a fifth is refused.
    std::vector<uint32_t> four;
    for (int i = 0; i < 4; ++i) four.push_back(t.alloc(1024 - kHeader));
    CHECK(four[0] && four[1] && four[2] && four[3]);
    CHECK(h.usedBy(kBudgetA) == 4096);
    CHECK(t.alloc(1) == 0);
    CHECK(t.alloc(16) == 0);
    CHECK(h.usedBy(kBudgetA) == 4096 && h.bucketCount() == 4);

    // Another budget is counted on its own.
    CHECK(t.alloc(1024 - kHeader, 16, false, kBudgetB) != 0);
    CHECK(t.alloc(1024 - kHeader, 16, false, kBudgetB) != 0);
    CHECK(t.alloc(1, 16, false, kBudgetB) == 0);
    CHECK(h.usedBy(kBudgetA) == 4096 && h.usedBy(kBudgetB) == 2048);

    // A budget with no cap takes nothing, whatever the heap has room for.
    CHECK(h.usedBy(kBudgetC) == 0 && t.alloc(16, 16, false, kBudgetC) == 0);
    CHECK(h.allocateBucket(16, 16, false, 901, Budget::System) == nullptr);

    // A release gives the room back, exactly.
    t.release(four[1]);
    CHECK(h.usedBy(kBudgetA) == 3072);
    CHECK(t.alloc(1024 - kHeader + 1) == 0);  // one byte over the block
    CHECK(t.alloc(1024 - kHeader) != 0);
    CHECK(h.usedBy(kBudgetA) == 4096);
    CHECK(t.intact());
  }

  {  // The cap counts the blocks that are really there (headers included), so
     // small requests cost their blocks.
    Caps small;
    small.set(kBudgetA, 128);
    Stage st(small);
    Tracker t(st);
    CHECK(t.alloc(1) != 0 && st.heap.usedBy(kBudgetA) == 32);
    CHECK(t.alloc(1) != 0 && t.alloc(1) != 0 && t.alloc(1) != 0);
    CHECK(st.heap.usedBy(kBudgetA) == 128 && t.alloc(1) == 0);
  }

  {  // The heap itself running out is a refusal too, and changes nothing.
    Stage st(roomy(), 4096);
    Tracker t(st);
    uint32_t got = 0;
    while (t.alloc(1000) != 0) ++got;
    CHECK(got == 4);
    const size_t freeNow = st.heap.freeBytes();
    CHECK(t.alloc(1000) == 0 && st.heap.freeBytes() == freeNow);
    CHECK(t.alloc(64, 128) == 0);
    CHECK(t.intact());
  }

  // No caps at all: nothing may take a bucket.
  MovableHeap bare;
  bare.init(g_arena, kArenaBytes, nullptr, nullptr);
  CHECK(bare.allocateBucket(16, 16, false, 1, kBudgetA) == nullptr);
  Caps one;
  one.set(kBudgetA, 64);
  bare.init(g_arena, kArenaBytes, one.v, nullptr);
  CHECK(bare.allocateBucket(16, 16, false, 1, kBudgetA) != nullptr);
  bare.init(g_arena, kArenaBytes, nullptr, nullptr);  // forgets it
  CHECK(bare.bucketCount() == 0 && bare.usedBy(kBudgetA) == 0);
  CHECK(bare.freeBytes() == kArenaBytes);
  CHECK(bare.validate());
}

void testMisuse() {
  {  // The table of buckets fills up, in every build.
    Stage st;
    MovableHeap& h = st.heap;
    Tracker t(st);
    CHECK(t.alloc(1000) != 0);
    std::vector<uint32_t> tiny;
    for (uint32_t i = 1; i < MovableHeap::kMaxBuckets; ++i) {
      tiny.push_back(t.alloc(16));
    }
    CHECK(h.bucketCount() == MovableHeap::kMaxBuckets);
    CHECK(throwsFailure([&] { t.alloc(16); }, "bucket table full"));
    CHECK(h.bucketCount() == MovableHeap::kMaxBuckets && h.validate());
    t.release(tiny.back());
    CHECK(t.alloc(16) != 0 && h.validate());
    CHECK(h.usedBy(Budget::Max) == 0 && h.usedBy(0xFFFF) == 0);
  }

  Stage st2;
  MovableHeap& g = st2.heap;
#if TYRA_MEMORY_CHECKS
  CHECK(throwsFailure([&] { g.allocateBucket(16, 16, false, 5, Budget::Max); },
                      "budget out of range"));
  CHECK(throwsFailure([&] { g.allocateBucket(16, 16, false, 0, kBudgetA); },
                      "without a token"));
  CHECK(g.allocateBucket(16, 16, false, 5, kBudgetA) != nullptr);
  CHECK(throwsFailure([&] { g.allocateBucket(16, 16, false, 5, kBudgetA); },
                      "already in use"));
  CHECK(throwsFailure([&] { g.releaseBucket(g_arena + 1024); },
                      "invalid release"));
  CHECK(g.bucketCount() == 1 && g.validate());
#else
  CHECK(g.allocateBucket(16, 16, false, 5, Budget::Max) == nullptr);
  CHECK(g.allocateBucket(16, 16, false, 0, kBudgetA) == nullptr);
  g.releaseBucket(g_arena + 1024);  // not a bucket: ignored
  CHECK(g.bucketCount() == 0 && g.validate());
#endif
  g.releaseBucket(nullptr);  // ignored everywhere
  CHECK(g.validate());
}

// An over-aligned bucket whose gap is one header long leaves the bytes with the
// bucket in front of it: those bytes are that bucket's budget's.
void testAbsorbedGap() {
  Stage st;
  Tracker t(st);
  MovableHeap& h = st.heap;
  const uint32_t a = t.alloc(16, 16, false, kBudgetA);
  CHECK(h.usedBy(kBudgetA) == 32);
  const uint32_t b = t.alloc(100, 64, false, kBudgetB);
  CHECK(b != 0 && t.base(b) == g_arena + 64);  // 16 bytes up from where it fit
  CHECK(h.usedBy(kBudgetA) == 48);             // a took them in
  CHECK(h.usedBy(kBudgetB) == blockFor(100));
  CHECK(h.freeBytes() == kArenaBytes - 48 - blockFor(100));
  CHECK(h.tokenAt(t.base(a) + 24) == a);  // its payload is longer now
  CHECK(t.intact());
  t.release(b);
  CHECK(h.usedBy(kBudgetB) == 0 && h.usedBy(kBudgetA) == 48);
  t.release(a);
  CHECK(h.usedBy(kBudgetA) == 0 && h.freeBytes() == kArenaBytes);
  CHECK(h.validate());
}

// ---- tokenAt ---------------------------------------------------------------

void testTokenAt() {
  Stage st;
  Tracker t(st);
  MovableHeap& h = st.heap;

  std::vector<uint32_t> toks;
  for (uint32_t i = 0; i < 300; ++i) toks.push_back(t.alloc(16 + (i % 7) * 48));
  bool exact = true;
  for (uint32_t tok : toks) {
    unsigned char* base = t.base(tok);
    const size_t bytes = t.live.at(tok).bytes;
    exact = exact && h.tokenAt(base) == tok;
    exact = exact && h.tokenAt(base + bytes / 2) == tok;
    exact = exact && h.tokenAt(base + bytes - 1) == tok;
    exact = exact && h.tokenAt(base + bytes) == 0;  // the next header
    exact = exact && h.tokenAt(base - 1) == 0;      // its own header
    exact = exact && h.tokenAt(base - kHeader) == 0;
  }
  CHECK(exact);

  // Outside the heap, null, and a bucket that has been given back.
  int local = 0;
  CHECK(h.tokenAt(nullptr) == 0 && h.tokenAt(&local) == 0);
  CHECK(h.tokenAt(reinterpret_cast<const void*>(
            reinterpret_cast<uintptr_t>(g_arena) - 1)) == 0);
  CHECK(h.tokenAt(g_arena + kArenaBytes) == 0);
  CHECK(h.tokenAt(g_arena + kArenaBytes - 1) == 0);  // free space
  for (size_t i = 0; i < toks.size(); i += 3) {
    unsigned char* base = t.base(toks[i]);
    t.release(toks[i]);
    CHECK(h.tokenAt(base) == 0 && h.tokenAt(base + 8) == 0);
  }
  bool others = true;
  for (size_t i = 0; i < toks.size(); ++i) {
    if (i % 3 != 0) others = others && h.tokenAt(t.base(toks[i])) == toks[i];
  }
  CHECK(others && t.intact());

  // A heap that was never given a range knows nothing.
  MovableHeap blank;
  CHECK(blank.tokenAt(g_arena + 64) == 0 && blank.tokenAt(nullptr) == 0);
}

// ---- releasing a bucket ----------------------------------------------------

void testReleaseDropsPtrs() {
  Stage st;
  Tracker t(st);
  MovableHeap& h = st.heap;

  const uint32_t x = t.alloc(2000);
  const uint32_t a = t.alloc(1000);
  unsigned char* const aAt = t.base(a);
  Ptr<uint32_t> stale(reinterpret_cast<uint32_t*>(aAt + 8));
  Ptr<uint32_t> kept(reinterpret_cast<uint32_t*>(t.base(x) + 8));
  CHECK(st.table().size() == 2);

  t.release(a);  // dropBucket(a's token): the stale Ptr is forgotten
  CHECK(st.table().size() == 1);

  // A new bucket with the same token, at the same place, is moved: the stale
  // Ptr is not patched for it.
  void* again = h.allocateBucket(1000, 16, false, a, kBudgetA);
  CHECK(again == aAt);
  t.release(x);  // and x's Ptr goes with it
  CHECK(st.table().size() == 0);
  CHECK(h.step(1 << 20) == blockFor(1000));
  CHECK(h.tokenAt(aAt - blockFor(2000)) == a);
  CHECK(stale.get() == reinterpret_cast<uint32_t*>(aAt + 8));  // left alone
  stale.reset();
  kept.reset();
  CHECK(st.table().size() == 0);
  h.releaseBucket(aAt - blockFor(2000));
  CHECK(h.bucketCount() == 0 && h.validate());
  CHECK(h.freeBytes() == kArenaBytes);
}

void testNoPtrRegistry() {
  Stage st(roomy(), kArenaBytes, /*withPtrs=*/false);
  Tracker t(st);
  const uint32_t x = t.alloc(2000);
  const uint32_t a = t.alloc(3000), b = t.alloc(3000);
  t.release(x);
  CHECK(st.heap.step(1 << 20) == 2 * blockFor(3000));
  CHECK(t.order.size() == 2 && t.order[0] == a && t.order[1] == b);
  CHECK(t.intact());
  t.release(a);
  t.release(b);
  CHECK(st.heap.freeBytes() == kArenaBytes && st.heap.validate());
}

void testNoListener() {
  Stage st;
  Tracker t(st);
  const uint32_t x = t.alloc(2000);
  const uint32_t a = t.alloc(3000);
  unsigned char* const aAt = t.base(a);
  t.release(x);
  st.heap.setMoveListener(nullptr, nullptr);
  CHECK(st.heap.step(1 << 20) == blockFor(3000));
  CHECK(st.heap.tokenAt(aAt + 2999) == 0);  // the old end is free again
  CHECK(st.heap.tokenAt(aAt - blockFor(2000)) == a);
  CHECK(st.heap.validate());
}

void testPinHook() {
  CHECK(detail::g_pinHook == nullptr);
  {
    MovableHeap one;
    one.init(g_arena, kArenaBytes, roomy().v, nullptr);
    one.installPinHook();
    CHECK(detail::g_pinHook != nullptr);
    {
      MovableHeap two;  // never installed: its end leaves the hook alone
    }
    CHECK(detail::g_pinHook != nullptr);
  }
  CHECK(detail::g_pinHook == nullptr);  // the heap that set it took it back

  // Without the hook a Pin does nothing, and the bucket moves.
  Stage st(roomy(), kArenaBytes, true, /*hook=*/false);
  Tracker t(st);
  const uint32_t x = t.alloc(2000);
  const uint32_t a = t.alloc(3000);
  t.release(x);
  Ptr<Probe> inA(reinterpret_cast<Probe*>(t.base(a)));
  {
    Pin<Probe> hold(inA);
    CHECK(st.heap.step(1 << 20) == blockFor(3000));
  }
  CHECK(t.intact());
}

// ---- random runs: heap, pins, Ptrs, caps -----------------------------------

constexpr size_t kSlot = 16;  // room for a Ptr at the start of a bigger bucket
constexpr size_t kSlotFrom = 64;

struct Target {
  uint32_t token = 0;  // 0: points at nothing
  size_t offset = 0;
};

class Stress {
 public:
  Stress(uint32_t seed, bool plain)
      : st(caps(), kStressBytes), t(st), rng(seed), plain_(plain) {}

  void run(int rounds) {
    for (int r = 0; r < rounds; ++r) {
      const uint32_t op = rng.below(100);
      if (op < 28) {
        allocOne();
      } else if (op < 46) {
        releaseOne();
      } else if (op < 54 && !plain_) {
        pinOne();
      } else if (op < 62 && !plain_) {
        unpinOne();
      } else if (op < 72) {
        addOutside();
      } else if (op < 77) {
        dropOutside();
      } else if (op < 87) {
        retargetResident();
      } else {
        stepOnce();
      }
      if (r % 8 == 0) checkAll();
    }
    checkAll();
    drainAndCheck();
  }

 private:
  static constexpr size_t kStressBytes = 256 * 1024;

  static Caps caps() {
    Caps c;
    c.set(kBudgetA, 120 * 1024).set(kBudgetB, 60 * 1024);
    c.set(kBudgetC, 30 * 1024);
    return c;
  }

  Ptr<uint32_t>* slotOf(uint32_t token) {
    return reinterpret_cast<Ptr<uint32_t>*>(t.base(token));
  }

  bool hasSlot(uint32_t token) const {
    return t.live.at(token).bytes >= kSlotFrom;
  }

  uint32_t anyLive() {
    if (t.live.empty()) return 0;
    auto it = t.live.begin();
    std::advance(it, rng.below(uint32_t(t.live.size())));
    return it->first;
  }

  uint32_t pick(std::vector<uint32_t> v) {
    return v[rng.below(uint32_t(v.size()))];
  }

  // A random 4-aligned address inside a bucket.
  size_t offsetIn(uint32_t token) {
    const size_t words = std::max<size_t>(1, t.live.at(token).bytes / 4);
    return rng.below(uint32_t(words)) * 4;
  }

  void allocOne() {
    const bool large = rng.chance(5);
    const size_t bytes = large ? 2000 + rng.below(6000) : 1 + rng.below(520);
    const size_t align = plain_ ? 16 : pick({16, 16, 16, 64, 128});
    const bool pinned = !plain_ && rng.chance(12);
    const BudgetId budget = pick({kBudgetA, kBudgetA, kBudgetB, kBudgetC});
    const bool slot = bytes >= kSlotFrom;
    const uint32_t tok =
        t.alloc(bytes, align, pinned, budget, slot ? kSlot : 0);
    if (tok == 0) return;
    if (slot) new (t.base(tok)) Ptr<uint32_t>();
    resident[tok] = Target();
  }

  // Every Ptr that points into `token` lets go, then the bucket goes.
  void releaseOne() {
    for (int tries = 0; tries < 8; ++tries) {
      const uint32_t tok = anyLive();
      if (tok == 0) return;
      if (pinCount[tok] != 0) continue;
      for (size_t i = 0; i < outside.size(); ++i) {
        if (outsideAt[i].token == tok) {
          outside[i]->reset();
          outsideAt[i] = Target();
        }
      }
      for (auto& kv : resident) {
        if (kv.first != tok && kv.second.token == tok) {
          slotOf(kv.first)->reset();
          kv.second = Target();
        }
      }
      if (hasSlot(tok)) slotOf(tok)->~Ptr();
      resident.erase(tok);
      t.release(tok);
      return;
    }
  }

  void pinOne() {
    const uint32_t tok = anyLive();
    if (tok == 0) return;
    st.heap.pin(t.base(tok) + rng.below(uint32_t(t.live.at(tok).bytes)));
    ++pinCount[tok];
  }

  void unpinOne() {
    for (auto& kv : pinCount) {
      if (kv.second == 0) continue;
      if (rng.chance(40)) continue;
      st.heap.unpin(t.base(kv.first) +
                    rng.below(uint32_t(t.live.at(kv.first).bytes)));
      --kv.second;
      return;
    }
  }

  void addOutside() {
    const uint32_t tok = anyLive();
    if (tok == 0 || outside.size() >= 200) return;
    const size_t off = offsetIn(tok);
    outside.emplace_back(
        new Ptr<uint32_t>(reinterpret_cast<uint32_t*>(t.base(tok) + off)));
    outsideAt.push_back(Target{tok, off});
  }

  void dropOutside() {
    if (outside.empty()) return;
    const size_t i = rng.below(uint32_t(outside.size()));
    outside.erase(outside.begin() + long(i));
    outsideAt.erase(outsideAt.begin() + long(i));
  }

  void retargetResident() {
    const uint32_t r = anyLive();
    if (r == 0 || !hasSlot(r)) return;
    const uint32_t g = anyLive();
    const size_t off = offsetIn(g);
    *slotOf(r) = Ptr<uint32_t>(reinterpret_cast<uint32_t*>(t.base(g) + off));
    resident[r] = Target{g, off};
  }

  // Whether a bucket can move at all.
  bool immovable(uint32_t tok) {
    const Live& l = t.live.at(tok);
    return l.pinned || l.align > 16 || pinCount[tok] != 0;
  }

  void stepOnce() {
    static const size_t budgets[] = {0, 1, 200, 4096, 65536, 1 << 20};
    const size_t budget = budgets[rng.below(6)];
    std::map<uint32_t, unsigned char*> before;
    for (const auto& kv : t.live) before[kv.first] = kv.second.base;
    t.order.clear();

    const size_t moved = st.heap.step(budget);

    CHECK(moved == st.heap.movedLastStep());
    CHECK(t.order.size() <= MovableHeap::kMaxMovesPerStep);
    CHECK(budget != 0 || moved == 0);
    CHECK(moved <= budget || t.order.size() == 1);
    for (const auto& kv : t.live) {
      const uint32_t tok = kv.first;
      if (kv.second.base == before[tok]) continue;
      CHECK(!immovable(tok));
      CHECK(kv.second.base < before[tok]);  // moves only slide down
    }
    CHECK(rowsConsistent(st));
    if (plain_ && rng.chance(10)) drainAndCheck();
  }

  // Steps until nothing moves; with nothing pinned the heap ends compact.
  void drainAndCheck() {
    for (int i = 0; i < 400; ++i) {
      if (st.heap.step(64 * 1024) == 0) break;
    }
    CHECK(st.heap.step(64 * 1024) == 0);
    if (plain_) {
      CHECK(st.heap.freeBytes() == 0 ||
            st.heap.largestFree() + kHeader == st.heap.freeBytes());
    }
    checkAll();
  }

  void checkPtr(const Ptr<uint32_t>& p, const Target& at) {
    if (at.token == 0) {
      CHECK(!p);
      return;
    }
    CHECK(p.get() ==
          reinterpret_cast<uint32_t*>(t.live.at(at.token).base + at.offset));
  }

  void checkAll() {
    CHECK(t.intact());
    CHECK(rowsConsistent(st));
    uint32_t pointers = 0;
    for (size_t i = 0; i < outside.size(); ++i) {
      checkPtr(*outside[i], outsideAt[i]);
      pointers += outsideAt[i].token != 0 ? 1 : 0;
    }
    for (const auto& kv : resident) {
      if (!hasSlot(kv.first)) continue;
      checkPtr(*slotOf(kv.first), kv.second);
      pointers += kv.second.token != 0 ? 1 : 0;
    }
    CHECK(st.table().size() == pointers);
    size_t total = 0;
    for (BudgetId b : {kBudgetA, kBudgetB, kBudgetC}) {
      total += st.heap.usedBy(b);
    }
    CHECK(total == kStressBytes - st.heap.freeBytes());
  }

 public:
  Stage st;
  Tracker t;
  Rng rng;
  bool plain_;
  std::map<uint32_t, Target> resident;  // the Ptr at the start of a bucket
  std::map<uint32_t, uint32_t> pinCount;
  std::vector<std::unique_ptr<Ptr<uint32_t>>> outside;
  std::vector<Target> outsideAt;

  ~Stress() {
    // The Ptrs go while the table is still installed.
    outside.clear();
    for (auto& kv : resident) {
      if (hasSlot(kv.first)) slotOf(kv.first)->~Ptr();
    }
    resident.clear();
  }
};

void testStress() {
  for (uint32_t seed = 1; seed <= 6; ++seed) {
    Stress plain(seed, true);
    plain.run(3000);
  }
  for (uint32_t seed = 11; seed <= 16; ++seed) {
    Stress mixed(seed, false);
    mixed.run(3000);
  }
}

// ---- random run: pools, entries linked by Ptrs, defragmentation ------------

void testPoolRun() {
  World w;
  World::PoolBox* pools[2] = {
      &w.addPool({"left", kBudgetA, 24, 8, 4, 4, 8, Policy::Refuse},
                 Kind::Logic),
      &w.addPool({"right", kBudgetB, 24, 8, 2, 5, 6, Policy::Refuse},
                 Kind::Logic)};
  MovableHeap& heap = w.stage.heap;
  Rng rng(7);

  struct Model {
    uint32_t pool;
    uint32_t next;  // serial of the entry its Ptr names; 0: none
  };
  std::map<uint32_t, Model> model;  // serial -> what it should be
  struct Holder {
    std::unique_ptr<Ptr<Entry>> ptr;
    uint32_t serial;
  };
  std::vector<Holder> holders;
  uint32_t serial = 0;

  auto entryOf = [&](uint32_t s) -> Entry* {
    return findEntry(*pools[model.at(s).pool], s);
  };
  auto pickSerial = [&]() -> uint32_t {
    if (model.empty()) return 0;
    auto it = model.begin();
    std::advance(it, rng.below(uint32_t(model.size())));
    return it->first;
  };

  auto check = [&]() {
    CHECK(heap.validate());
    CHECK(rowsConsistent(w.stage));
    size_t live = 0;
    bool right = true;
    for (World::PoolBox* p : pools) {
      live += p->core.live();
      right =
          right && allEntriesHoldTheirIndex(*p) && bucketsMatchTokens(w, *p);
      for (Entry* e : liveEntries(*p)) {
        auto it = model.find(e->index);
        if (it == model.end()) {
          right = false;
          continue;
        }
        right = right && (it->second.next == 0
                              ? !e->next
                              : e->next.get() == entryOf(it->second.next));
      }
    }
    CHECK(live == model.size() && right);
    bool held = true;
    for (const Holder& h : holders) {
      held = held && h.ptr->get() == entryOf(h.serial);
    }
    CHECK(held);
  };

  for (int round = 0; round < 1500; ++round) {
    const uint32_t op = rng.below(100);
    if (op < 38) {  // acquire
      const uint32_t pi = rng.below(2);
      Entry* e = makeEntry(*pools[pi], serial + 1);
      if (e != nullptr) {
        ++serial;
        model[serial] = Model{pi, 0};
        const uint32_t target = pickSerial();
        if (target != 0 && rng.chance(80)) {
          e->next = Ptr<Entry>(entryOf(target));
          model[serial].next = target;
        }
      }
    } else if (op < 66) {  // release, after everything that points at it
      const uint32_t s = pickSerial();
      if (s == 0) continue;
      for (auto& kv : model) {
        if (kv.second.next == s) {
          entryOf(kv.first)->next.reset();
          kv.second.next = 0;
        }
      }
      for (size_t i = holders.size(); i-- > 0;) {
        if (holders[i].serial == s) holders.erase(holders.begin() + long(i));
      }
      destroyEntry(*pools[model.at(s).pool], entryOf(s));
      model.erase(s);
    } else if (op < 72) {  // a Ptr held outside
      const uint32_t s = pickSerial();
      if (s != 0 && holders.size() < 60) {
        holders.push_back(
            Holder{std::unique_ptr<Ptr<Entry>>(new Ptr<Entry>(entryOf(s))), s});
      }
    } else if (op < 78) {  // empty extensions go back
      pools[rng.below(2)]->core.releaseEmptyExtensions();
    } else {  // a frame start
      static const size_t budgets[] = {200, 4096, 65536};
      heap.step(budgets[rng.below(3)]);
    }
    if (round % 5 == 0) check();
  }
  check();

  // Settle, and let everything go.
  for (int i = 0; i < 200 && heap.step(64 * 1024) != 0; ++i) {
  }
  check();
  holders.clear();
  for (auto& kv : model) entryOf(kv.first)->next.reset();
  while (!model.empty()) {
    const uint32_t s = model.begin()->first;
    destroyEntry(*pools[model.at(s).pool], entryOf(s));
    model.erase(s);
  }
  CHECK(w.stage.table().size() == 0);
  for (World::PoolBox* p : pools) p->core.releaseEmptyExtensions();
  CHECK(heap.validate());
}

}  // namespace

int main() {
  installThrowingFailure();
  try {
    testBrief();
    testStepBudget();
    testStepBudgetEdges();
    testMoveLimit();
    testPin();
    testPinEdges();
    testNeverMoves();
    testUnmovableDoesNotStopTheWalk();
    testPtrAtTheEnd();
    testCaps();
    testMisuse();
    testAbsorbedGap();
    testTokenAt();
    testReleaseDropsPtrs();
    testNoPtrRegistry();
    testNoListener();
    testPinHook();
    testStress();
    testPoolRun();
  } catch (const ThrownFailure& thrown) {
    ++verify_detail::checks;
    ++verify_detail::failures;
    std::fprintf(stderr, "unexpected failure: %s name=%s size=%zu\n",
                 thrown.f.what, thrown.f.name != nullptr ? thrown.f.name : "-",
                 thrown.f.size);
  }
  return finish("defrag");
}
