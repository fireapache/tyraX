// Added by TyraX: host verifier for Ptr<T> and its table (inc/memory/ptr.hpp,
// src/memory/ptr_registry.cpp). Built and run by tools/verify-memory.py, once
// with TYRA_MEMORY_CHECKS=0 and once with =1.
//
// A fake movable heap - a static arena cut into 32 KB slots, buckets that the
// tests move by hand with memmove - stands in for the real one.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <map>
#include <new>
#include <type_traits>
#include <utility>
#include <vector>

#include "memory/pool.hpp"
#include "memory/ptr.hpp"
#include "memory/registry.hpp"
#include "verify-memory-check.hpp"

namespace {

// ---- the fake movable heap -------------------------------------------------

constexpr size_t kSlotBytes = 32768;
constexpr unsigned kSlots = 8;

alignas(64) unsigned char g_arena[kSlots * kSlotBytes];

unsigned char* slot(unsigned i) { return g_arena + size_t(i) * kSlotBytes; }

// Storage for a table of `capacity` entries, aligned for a pointer.
std::vector<uintptr_t> storageFor(uint32_t capacity) {
  return std::vector<uintptr_t>(
      (PtrTable::bytesFor(capacity) + sizeof(uintptr_t) - 1) /
          sizeof(uintptr_t) +
      1);
}

// A bucket as tokenAt sees it.
struct Zone {
  uintptr_t base;
  size_t bytes;
  uint32_t token;
};

// The Ptr registry over the arena, installed as the globals the Ptrs use for
// as long as it lives.
class World {
 public:
  explicit World(uint32_t capacity = 1024) : storage_(storageFor(capacity)) {
    const uintptr_t lo = reinterpret_cast<uintptr_t>(g_arena);
    registry.init(storage_.data(), capacity, lo, lo + sizeof g_arena,
                  &World::tokenThunk, this);
    install();
  }
  ~World() { uninstall(); }
  World(const World&) = delete;
  World& operator=(const World&) = delete;

  void install() {
    detail::g_ptrs = &registry;
    detail::g_movableLo = registry.lo();
    detail::g_movableHi = registry.hi();
  }
  static void uninstall() {
    detail::g_ptrs = nullptr;
    detail::g_movableLo = detail::g_movableHi = 0;
  }

  PtrTable& table() { return registry.table(); }

  PtrRegistry registry;
  std::vector<Zone> zones;

 private:
  static uint32_t tokenThunk(const void* p, void* ctx) {
    const World* w = static_cast<const World*>(ctx);
    const uintptr_t a = reinterpret_cast<uintptr_t>(p);
    for (const Zone& z : w->zones) {
      if (a - z.base < z.bytes) return z.token;
    }
    return 0;
  }

  std::vector<uintptr_t> storage_;
};

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

// Everything the table holds, as {address -> token}.
std::map<uintptr_t, uint32_t> contents(const PtrTable& t) {
  std::map<uintptr_t, uint32_t> m;
  t.forEach([&](void* address, uint32_t token) {
    m[reinterpret_cast<uintptr_t>(address)] = token;
  });
  return m;
}

size_t withToken(const PtrTable& t, uint32_t token) {
  size_t n = 0;
  t.forEach([&](void*, uint32_t k) { n += k == token ? 1 : 0; });
  return n;
}

size_t entriesIn(const PtrTable& t, const void* base, size_t bytes) {
  size_t n = 0;
  const uintptr_t b = reinterpret_cast<uintptr_t>(base);
  t.forEach([&](void* address, uint32_t) {
    if (reinterpret_cast<uintptr_t>(address) - b < bytes) ++n;
  });
  return n;
}

bool anyEntryIn(const PtrTable& t, const void* base, size_t bytes) {
  return entriesIn(t, base, bytes) != 0;
}

// What the scenarios build.
struct Node {
  int value;
  Ptr<Node> next;
  Ptr<Node> other;
  explicit Node(int v) : value(v) {}
};

Node* nodeAt(void* base, size_t i) {
  return reinterpret_cast<Node*>(static_cast<unsigned char*>(base) +
                                 i * sizeof(Node));
}

// ---- the brief
// ---------------------------------------------------------------

int* intIn(unsigned slotIndex, size_t i) {
  return reinterpret_cast<int*>(slot(slotIndex)) + i;
}

void testBrief() {
  World w;
  w.zones = {{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes, 1},
             {reinterpret_cast<uintptr_t>(slot(1)), kSlotBytes, 2}};
  PtrTable& table = w.table();
  int outside = 0;

  {
    Ptr<int> p(&outside);
    CHECK(table.size() == 0);
  }
  {
    Ptr<int> p(intIn(0, 0));
    CHECK(table.size() == 1);
    Ptr<int> c = p;
    CHECK(table.size() == 2);
    Ptr<int> m = std::move(c);
    CHECK(table.size() == 2);
    CHECK(!c);  // a move empties the source
    CHECK(m.get() == intIn(0, 0));
    m.reset();
    CHECK(table.size() == 1);
    CHECK(!m);
  }
  CHECK(table.size() == 0);
}

// The whole life of a Ptr, one operation at a time, against the table.
void testLifecycle() {
  World w;
  w.zones = {{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes, 1},
             {reinterpret_cast<uintptr_t>(slot(1)), kSlotBytes, 2}};
  PtrTable& table = w.table();
  int outside = 0;
  int* in1 = intIn(0, 4);
  int* in2 = intIn(1, 4);

  // The token is the bucket the target is in.
  {
    Ptr<int> a(in1), b(in2);
    CHECK(table.size() == 2);
    CHECK(contents(table)[reinterpret_cast<uintptr_t>(&a)] == 1);
    CHECK(contents(table)[reinterpret_cast<uintptr_t>(&b)] == 2);
  }
  CHECK(table.size() == 0);

  // Null and the default are not registered.
  {
    Ptr<int> d;
    Ptr<int> n(nullptr);
    Ptr<int> n2 = nullptr;
    Ptr<int> fromNull(static_cast<int*>(nullptr));
    CHECK(!d && !n && !n2 && !fromNull);
    CHECK(table.size() == 0);
  }

  // Copy assignment: outside -> in, in -> outside, in -> in (token replaced),
  // in -> null.
  {
    Ptr<int> src(in1), dst, other(&outside), toB2(in2);
    dst = src;
    CHECK(table.size() == 3 && dst.get() == in1);
    dst = other;  // leaves the movable heap
    CHECK(table.size() == 2 && dst.get() == &outside);
    dst = src;
    CHECK(table.size() == 3);
    dst = toB2;  // another bucket: same Ptr, new token
    CHECK(table.size() == 3);
    CHECK(contents(table)[reinterpret_cast<uintptr_t>(&dst)] == 2);
    dst = Ptr<int>();
    CHECK(table.size() == 2 && !dst);
    dst = src;
    CHECK(table.size() == 3);
    dst = nullptr;
    CHECK(table.size() == 2 && !dst);
    Ptr<int>& dstAlias = dst;
    Ptr<int>& srcAlias = src;
    dst = dstAlias;  // self-assignment changes nothing
    CHECK(table.size() == 2);
    src = srcAlias;
    CHECK(table.size() == 2 && src.get() == in1);
  }
  CHECK(table.size() == 0);

  // Move assignment.
  {
    Ptr<int> a(in1), b(in2);
    a = std::move(b);
    CHECK(table.size() == 1 && a.get() == in2 && !b);
    CHECK(contents(table)[reinterpret_cast<uintptr_t>(&a)] == 2);
    Ptr<int> o(&outside);
    a = std::move(o);  // an outside target replaces an inside one
    CHECK(table.size() == 0 && a.get() == &outside && !o);
    Ptr<int> c(in1);
    Ptr<int>& cAlias = c;
    c = std::move(cAlias);  // self-move leaves it alone
    CHECK(table.size() == 1 && c.get() == in1);
    Ptr<int> d;
    d = std::move(c);
    CHECK(table.size() == 1 && d.get() == in1 && !c);
  }
  CHECK(table.size() == 0);

  // In a container that grows (the elements are moved).
  {
    std::vector<Ptr<int>> v;
    for (int i = 0; i < 40; ++i) v.push_back(Ptr<int>(intIn(i % 2, i)));
    CHECK(table.size() == 40);
    std::vector<Ptr<int>> copy = v;
    CHECK(table.size() == 80);
    v.erase(v.begin(), v.begin() + 10);
    CHECK(table.size() == 70);
    copy.clear();
    CHECK(table.size() == 30);
  }
  CHECK(table.size() == 0);

  // A Ptr to a place in the movable heap that is in no bucket is registered
  // with token 0: its own address may still move.
  {
    Ptr<int> free(intIn(5, 0));
    CHECK(table.size() == 1);
    CHECK(contents(table)[reinterpret_cast<uintptr_t>(&free)] == 0);
  }
  CHECK(table.size() == 0);

  // Const and void targets compile; a Ptr is an ordinary standard-layout
  // object whose first word is the raw pointer.
  {
    Ptr<const int> c(in1);
    Ptr<void> v(in1);
    CHECK(c.get() == in1 && v.get() == in1);
    CHECK(*c == *in1);
    uintptr_t first;
    std::memcpy(&first, static_cast<const void*>(&c), sizeof first);
    CHECK(first == reinterpret_cast<uintptr_t>(in1));
    static_assert(std::is_standard_layout<Ptr<Node>>::value, "");
    static_assert(sizeof(Ptr<Node>) == sizeof(Ptr<char>), "");
  }
  CHECK(table.size() == 0);

  // A Ptr behaves as a pointer.
  {
    Node n(5);
    Ptr<Node> p(&n);
    CHECK(p->value == 5 && (*p).value == 5 && bool(p));
    p->value = 6;
    CHECK(n.value == 6);
  }
}

// ---- resolve ---------------------------------------------------------------

struct Ring {
  static constexpr int kN = 8;
  Node* node[kN];
};

// A ring of Nodes at the start of `base`, node i linked to node i+1.
Ring buildRing(void* base, int firstValue) {
  Ring r;
  for (int i = 0; i < Ring::kN; ++i) {
    r.node[i] = new (nodeAt(base, i)) Node(firstValue + i);
  }
  for (int i = 0; i < Ring::kN; ++i) {
    r.node[i]->next = Ptr<Node>(r.node[(i + 1) % Ring::kN]);
  }
  return r;
}

Ring ringAt(void* base) {
  Ring r;
  for (int i = 0; i < Ring::kN; ++i) r.node[i] = nodeAt(base, i);
  return r;
}

bool ringIsIntact(const Ring& r, int firstValue, int exceptIndex = -1,
                  int exceptValue = 0) {
  const Node* cur = r.node[0];
  for (int i = 0; i < Ring::kN; ++i) {
    if (cur != r.node[i]) return false;
    const int expect = i == exceptIndex ? exceptValue : firstValue + i;
    if (cur->value != expect) return false;
    cur = cur->next.get();
  }
  return cur == r.node[0];
}

void destroyRing(const Ring& r) {
  for (int i = 0; i < Ring::kN; ++i) r.node[i]->~Node();
}

// Review Focus 4: Ptrs that live in a bucket and point into it, a Ptr outside
// that points into it, and a Ptr inside it that points into another bucket.
void testResolveBrief() {
  World w;
  const uintptr_t b1 = reinterpret_cast<uintptr_t>(slot(0));
  const uintptr_t b2 = reinterpret_cast<uintptr_t>(slot(1));
  const uintptr_t b1moved = reinterpret_cast<uintptr_t>(slot(2));
  w.zones = {{b1, kSlotBytes, 1}, {b2, kSlotBytes, 2}};
  PtrTable& table = w.table();
  std::memset(g_arena, 0xCD, sizeof g_arena);

  Ring ring = buildRing(slot(0), 100);
  Node* t = new (nodeAt(slot(1), 40)) Node(9);
  t->next = Ptr<Node>(ring.node[5]);   // bucket 2 -> bucket 1
  t->other = Ptr<Node>(t);             // bucket 2 -> bucket 2
  ring.node[0]->other = Ptr<Node>(t);  // bucket 1 -> bucket 2
  ring.node[3]->value = 7;
  Ptr<Node> outsidePtr(ring.node[3]);
  Ptr<int> interior(&ring.node[2]->value);  // into the middle of a node
  CHECK(table.size() == 8 + 1 + 1 + 1 + 1 + 1);
  CHECK(withToken(table, 1) == 8 + 1 + 1 + 1);
  CHECK(withToken(table, 2) == 2);

  std::memmove(slot(2), slot(0), kSlotBytes);
  const Move mv{b1, kSlotBytes, b1moved, 1};
  table.resolve(&mv, 1);
  w.zones = {{b1moved, kSlotBytes, 1}, {b2, kSlotBytes, 2}};

  Ring moved = ringAt(slot(2));
  CHECK(ringIsIntact(moved, 100, 3, 7));
  CHECK(outsidePtr.get() == moved.node[3] && outsidePtr->value == 7);
  CHECK(moved.node[0]->other.get() == t && moved.node[0]->other->value == 9);
  CHECK(t->next.get() == moved.node[5]);
  CHECK(t->other.get() == t);  // untouched: neither it nor its target moved
  CHECK(interior.get() == &moved.node[2]->value && *interior == 102);
  CHECK(table.size() == 13);
  CHECK(!anyEntryIn(table, slot(0),
                    kSlotBytes));  // nothing left at the old place
  CHECK(withToken(table, 1) == 11 && withToken(table, 2) == 2);

  // The moved Ptrs are found by their new addresses when they go away.
  destroyRing(moved);
  t->~Node();
  outsidePtr.reset();
  interior.reset();
  CHECK(table.size() == 0);  // no stale keys left
}

// A bucket slides down over part of its own old place, then up again.
void slideTest(size_t oldOffset, size_t newOffset) {
  World w;
  unsigned char* from = g_arena + oldOffset;
  unsigned char* to = g_arena + newOffset;
  w.zones = {{reinterpret_cast<uintptr_t>(from), kSlotBytes, 1}};
  PtrTable& table = w.table();
  std::memset(g_arena, 0xCD, sizeof g_arena);

  Ring ring = buildRing(from, 50);
  Ptr<Node> a(ring.node[2]), b(ring.node[7]);
  ring.node[1]->other = Ptr<Node>(ring.node[6]);
  CHECK(table.size() == 8 + 2 + 1);

  std::memmove(to, from, kSlotBytes);
  const Move mv{reinterpret_cast<uintptr_t>(from), kSlotBytes,
                reinterpret_cast<uintptr_t>(to), 1};
  table.resolve(&mv, 1);
  w.zones = {{reinterpret_cast<uintptr_t>(to), kSlotBytes, 1}};

  Ring now = ringAt(to);
  CHECK(ringIsIntact(now, 50));
  CHECK(a.get() == now.node[2] && b.get() == now.node[7]);
  CHECK(now.node[1]->other.get() == now.node[6]);
  CHECK(table.size() == 11);
  CHECK(entriesIn(table, to, kSlotBytes) == 9);  // the ring's 8 and one more

  destroyRing(now);
  a.reset();
  b.reset();
  CHECK(table.size() == 0);
}

void testResolveOverlap() {
  slideTest(16384, 4096);    // down, 20 KB of the old place reused
  slideTest(4096, 16384);    // up, the same
  slideTest(kSlotBytes, 0);  // down by a whole bucket: touching, no overlap
  slideTest(8, 0);           // by 8 bytes
}

// Two buckets exchange places in ONE resolve: bucket 2's Ptrs land in the
// range that bucket 1's old Ptrs occupied. A pass that re-keyed in place would
// move them twice.
void testResolveSwap() {
  World w;
  const uintptr_t s0 = reinterpret_cast<uintptr_t>(slot(0));
  const uintptr_t s1 = reinterpret_cast<uintptr_t>(slot(1));
  const uintptr_t s2 = reinterpret_cast<uintptr_t>(slot(2));
  w.zones = {{s0, kSlotBytes, 1}, {s1, kSlotBytes, 2}};
  PtrTable& table = w.table();
  std::memset(g_arena, 0xCD, sizeof g_arena);

  Ring r1 = buildRing(slot(0), 10);
  Ring r2 = buildRing(slot(1), 20);
  for (int i = 0; i < Ring::kN; ++i) {
    r1.node[i]->other = Ptr<Node>(r2.node[(i + 3) % Ring::kN]);
    r2.node[i]->other = Ptr<Node>(r1.node[(i + 5) % Ring::kN]);
  }
  CHECK(table.size() == 4 * Ring::kN);

  std::memcpy(slot(2), slot(0), kSlotBytes);  // bucket 1 -> slot 2
  std::memcpy(slot(0), slot(1),
              kSlotBytes);  // bucket 2 -> slot 0 (over bucket 1's old place)
  const Move moves[2] = {{s0, kSlotBytes, s2, 1}, {s1, kSlotBytes, s0, 2}};
  table.resolve(moves, 2);
  w.zones = {{s2, kSlotBytes, 1}, {s0, kSlotBytes, 2}};

  Ring n1 = ringAt(slot(2));
  Ring n2 = ringAt(slot(0));
  CHECK(ringIsIntact(n1, 10) && ringIsIntact(n2, 20));
  for (int i = 0; i < Ring::kN; ++i) {
    CHECK(n1.node[i]->other.get() == n2.node[(i + 3) % Ring::kN]);
    CHECK(n2.node[i]->other.get() == n1.node[(i + 5) % Ring::kN]);
  }
  CHECK(table.size() == 4 * Ring::kN);
  CHECK(!anyEntryIn(table, slot(1), kSlotBytes));

  destroyRing(n1);
  destroyRing(n2);
  CHECK(table.size() == 0);
}

// resolve with nothing to do.
void testResolveNoOps() {
  World w;
  w.zones = {{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes, 1}};
  PtrTable& table = w.table();
  const Move mv{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes,
                reinterpret_cast<uintptr_t>(slot(2)), 1};
  table.resolve(&mv, 1);  // an empty table
  table.resolve(nullptr, 0);
  CHECK(table.size() == 0);

  int* target = intIn(3, 0);
  Ptr<int> p(target);
  const auto before = contents(table);
  table.resolve(nullptr, 0);  // no moves
  CHECK(contents(table) == before && p.get() == target);
  const Move elsewhere{reinterpret_cast<uintptr_t>(slot(6)), 64,
                       reinterpret_cast<uintptr_t>(slot(7)), 9};
  table.resolve(&elsewhere, 1);  // moves that concern nobody
  CHECK(contents(table) == before && p.get() == target);

  // A move with token 0 never patches the token-0 Ptrs (they point into no
  // bucket); a Ptr whose OWN address is in the range still follows it.
  Ptr<int> free(intIn(5, 0));
  int* freeTarget = intIn(5, 0);
  const Move zero{reinterpret_cast<uintptr_t>(slot(6)), 64,
                  reinterpret_cast<uintptr_t>(slot(7)), 0};
  table.resolve(&zero, 1);
  CHECK(free.get() == freeTarget);
  free.reset();

  // A Ptr that lives in a moved bucket but points at a place that is in no
  // bucket (token 0): its entry follows it, its target stays.
  new (slot(6) + 16) Ptr<int>(freeTarget);
  CHECK(table.size() == 2);  // p and it
  std::memcpy(slot(7), slot(6), kSlotBytes);
  const Move bucket{reinterpret_cast<uintptr_t>(slot(6)), kSlotBytes,
                    reinterpret_cast<uintptr_t>(slot(7)), 6};
  table.resolve(&bucket, 1);
  Ptr<int>* now = reinterpret_cast<Ptr<int>*>(slot(7) + 16);
  CHECK(now->get() == freeTarget);
  CHECK(contents(table).count(reinterpret_cast<uintptr_t>(now)) == 1);
  CHECK(!anyEntryIn(table, slot(6), kSlotBytes));
  now->~Ptr();
  CHECK(table.size() == 1);
}

// Many rounds of random links and random moves, compared with the links that
// were made.
struct Cell {
  int id;
  Ptr<Cell> link;
};

void testRandomMoves(uint32_t seed) {
  constexpr int kBuckets = 4, kCellsPer = 48, kCells = kBuckets * kCellsPer;
  World w(1024);
  PtrTable& table = w.table();
  Rng rng(seed);

  for (int round = 0; round < 40; ++round) {
    std::memset(g_arena, 0xCD, sizeof g_arena);
    unsigned pos[kBuckets];  // the slot each bucket is in
    bool used[kSlots] = {};
    for (int b = 0; b < kBuckets; ++b) {
      pos[b] = static_cast<unsigned>(b);
      used[b] = true;
    }
    auto setZones = [&] {
      w.zones.clear();
      for (int b = 0; b < kBuckets; ++b) {
        w.zones.push_back({reinterpret_cast<uintptr_t>(slot(pos[b])),
                           kSlotBytes, static_cast<uint32_t>(b + 1)});
      }
    };
    auto cellAt = [&](int id) {
      const int b = id / kCellsPer;
      return reinterpret_cast<Cell*>(slot(pos[b]) +
                                     size_t(id % kCellsPer) * sizeof(Cell));
    };
    setZones();

    for (int id = 0; id < kCells; ++id) new (cellAt(id)) Cell{id, {}};
    std::vector<int> target(kCells, -1);
    int links = 0;
    for (int id = 0; id < kCells; ++id) {
      if (rng.below(4) == 0) continue;
      target[id] = int(rng.below(kCells));
      cellAt(id)->link = Ptr<Cell>(cellAt(target[id]));
      ++links;
    }
    std::vector<Ptr<Cell>> outside;
    std::vector<int> outsideTarget;
    outside.reserve(64);
    for (int i = 0; i < 64; ++i) {
      outsideTarget.push_back(int(rng.below(kCells)));
      outside.push_back(Ptr<Cell>(cellAt(outsideTarget.back())));
    }
    CHECK(table.size() == uint32_t(links + 64));

    for (int step = 0; step < 3; ++step) {
      std::vector<Move> moves;
      std::vector<unsigned> vacated;
      bool took[kSlots];
      std::copy(used, used + kSlots, took);
      for (int b = 0; b < kBuckets; ++b) {
        if (rng.below(2) == 0) continue;
        unsigned to = rng.below(kSlots);
        for (unsigned k = 0; k < kSlots && took[to]; ++k)
          to = (to + 1) % kSlots;
        if (took[to]) continue;
        took[to] = true;
        std::memcpy(slot(to), slot(pos[b]), kSlotBytes);
        moves.push_back({reinterpret_cast<uintptr_t>(slot(pos[b])), kSlotBytes,
                         reinterpret_cast<uintptr_t>(slot(to)),
                         static_cast<uint32_t>(b + 1)});
        vacated.push_back(pos[b]);
        used[to] = true;
        pos[b] = to;
      }
      for (unsigned v : vacated) {
        used[v] = false;
        std::memset(slot(v), 0xEE,
                    kSlotBytes);  // reads of the old place now show
      }
      table.resolve(moves.data(), static_cast<uint32_t>(moves.size()));
      setZones();

      bool linksOk = true, outsideOk = true;
      for (int id = 0; id < kCells; ++id) {
        const Cell* c = cellAt(id);
        if (c->id != id) linksOk = false;
        if (target[id] < 0 ? bool(c->link)
                           : c->link.get() != cellAt(target[id]))
          linksOk = false;
      }
      for (int i = 0; i < 64; ++i) {
        if (outside[i].get() != cellAt(outsideTarget[i])) outsideOk = false;
      }
      CHECK(linksOk);
      CHECK(outsideOk);
      CHECK(table.size() == uint32_t(links + 64));
      for (unsigned v : vacated) {
        CHECK(!anyEntryIn(table, slot(v), kSlotBytes));
      }
    }

    for (int id = 0; id < kCells; ++id) cellAt(id)->~Cell();
    outside.clear();
    CHECK(table.size() == 0);
  }
}

// ---- dropBucket
// --------------------------------------------------------------

void testDropBucket() {
  World w;
  w.zones = {{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes, 1},
             {reinterpret_cast<uintptr_t>(slot(1)), kSlotBytes, 2}};
  PtrTable& table = w.table();

  std::vector<Ptr<int>> a, b;
  a.reserve(30);
  b.reserve(30);
  for (int i = 0; i < 30; ++i) a.push_back(Ptr<int>(intIn(0, i)));
  for (int i = 0; i < 30; ++i) b.push_back(Ptr<int>(intIn(1, i)));
  Ptr<int> free(intIn(5, 0));  // token 0
  CHECK(table.size() == 61);

  table.dropBucket(0);  // token 0 means "none": ignored
  CHECK(table.size() == 61);
  table.dropBucket(2);
  CHECK(table.size() == 31);
  CHECK(withToken(table, 2) == 0);  // Review Focus 4: no entry with the token
  CHECK(withToken(table, 1) == 30);
  table.dropBucket(2);  // again: nothing
  table.dropBucket(77);
  CHECK(table.size() == 31);

  // Every Ptr that is left is still registered where it lives: a dropped one
  // is gone from the table, and going away is harmless.
  for (const Ptr<int>& p : a) {
    CHECK(contents(table).count(reinterpret_cast<uintptr_t>(&p)) == 1);
  }
  b.clear();
  CHECK(table.size() == 31);
  a.clear();
  free.reset();
  CHECK(table.size() == 0);
}

// ---- the table by itself, against a model
// ------------------------------------

void expectSame(const PtrTable& t, const std::map<uintptr_t, uint32_t>& model,
                const char* what) {
  const bool same = contents(t) == model && t.size() == model.size();
  if (!same) {
    std::fprintf(stderr, "table differs from its model after %s\n", what);
  }
  CHECK(same);
}

// Every key the model has can be found (a lookup that cannot find it would add
// a duplicate and the size would go up), and a key it does not have is absent.
void expectReachable(PtrTable& t, const std::map<uintptr_t, uint32_t>& model) {
  bool ok = true;
  for (const auto& kv : model) {
    const uint32_t before = t.size();
    if (!t.insert(reinterpret_cast<void*>(kv.first), kv.second)) ok = false;
    if (t.size() != before) ok = false;
  }
  CHECK(ok);
  const uint32_t before = t.size();
  t.erase(reinterpret_cast<void*>(uintptr_t(8) * 1000003u));
  CHECK(t.size() == before);
}

void tableModel(uint32_t capacity, uint32_t seed, int ops) {
  std::vector<uintptr_t> store = storageFor(capacity);
  PtrTable t;
  t.init(store.data(), capacity);
  CHECK(t.capacity() == capacity && t.size() == 0);
  std::map<uintptr_t, uint32_t> model;
  Rng rng(seed);
  const uint32_t keySpace = capacity * 2 + 3;
  auto key = [&] { return uintptr_t(8) * (1 + rng.below(keySpace)); };
  auto token = [&] { return rng.below(5); };

  for (int i = 0; i < ops; ++i) {
    const uint32_t op = rng.below(12);
    const char* what = "?";
    if (op < 5) {
      what = "insert";
      const uintptr_t k = key();
      const uint32_t tok = token();
      const bool room = model.count(k) != 0 || model.size() < capacity;
      CHECK(t.insert(reinterpret_cast<void*>(k), tok) == room);
      if (room) model[k] = tok;
    } else if (op < 8) {
      what = "erase";
      const uintptr_t k = key();
      t.erase(reinterpret_cast<void*>(k));
      model.erase(k);
    } else if (op < 10) {
      what = "rekey";
      const uintptr_t from = key(), to = key();
      t.rekey(reinterpret_cast<void*>(from), reinterpret_cast<void*>(to));
      auto it = model.find(from);
      if (it != model.end() && from != to) {
        const uint32_t tok = it->second;
        model.erase(it);
        model[to] = tok;
      }
    } else if (op == 10) {
      what = "dropBucket";
      const uint32_t tok = token();
      t.dropBucket(tok);
      if (tok != 0) {
        for (auto it = model.begin(); it != model.end();) {
          it = it->second == tok ? model.erase(it) : std::next(it);
        }
      }
    } else {
      what = "reachability";
      expectReachable(t, model);
    }
    expectSame(t, model, what);
  }
  expectReachable(t, model);
}

void testTableModel() {
  const uint32_t capacities[] = {1, 2, 4, 8, 16, 64, 256, 1024};
  for (uint32_t c : capacities) {
    for (uint32_t seed = 1; seed <= 6; ++seed)
      tableModel(c, seed * 31 + c, 1500);
  }
}

// A table filled to the last slot and emptied in several orders; every key
// stays reachable all the way.
void testTableFullAndUnwind() {
  for (uint32_t capacity : {2u, 16u, 256u, 4096u}) {
    for (int order = 0; order < 3; ++order) {
      std::vector<uintptr_t> store = storageFor(capacity);
      PtrTable t;
      t.init(store.data(), capacity);
      std::map<uintptr_t, uint32_t> model;
      std::vector<uintptr_t> keys;
      for (uint32_t i = 0; i < capacity; ++i) {
        const uintptr_t k = 0x1000 + uintptr_t(i) * 16;  // a regular stride
        CHECK(t.insert(reinterpret_cast<void*>(k), 1 + i % 3));
        model[k] = 1 + i % 3;
        keys.push_back(k);
      }
      CHECK(t.size() == capacity);
      CHECK(
          !t.insert(reinterpret_cast<void*>(uintptr_t(0x9000000)), 1));  // full
      CHECK(t.insert(reinterpret_cast<void*>(keys[0]),
                     9));  // known key: still ok
      model[keys[0]] = 9;
      expectSame(t, model, "fill");
      if (capacity <= 256) expectReachable(t, model);

      Rng rng(capacity + order);
      if (order == 1) std::reverse(keys.begin(), keys.end());
      if (order == 2) {
        for (size_t i = keys.size(); i > 1; --i) {
          std::swap(keys[i - 1], keys[rng.below(uint32_t(i))]);
        }
      }
      bool ok = true;
      size_t n = 0;
      for (uintptr_t k : keys) {
        t.erase(reinterpret_cast<void*>(k));
        model.erase(k);
        // After every erase the rest is still found (sampled for the big one).
        if (capacity <= 256 || ++n % 64 == 0) {
          const auto got = contents(t);
          if (got != model || t.size() != model.size()) ok = false;
          for (const auto& kv : model) {
            const uint32_t before = t.size();
            t.insert(reinterpret_cast<void*>(kv.first), kv.second);
            if (t.size() != before) ok = false;
          }
        }
      }
      CHECK(ok);
      CHECK(t.size() == 0);
    }
  }
}

void testTableRekeyAndReplace() {
  std::vector<uintptr_t> store = storageFor(4);
  PtrTable t;
  t.init(store.data(), 4);
  auto K = [](uintptr_t n) { return reinterpret_cast<void*>(n * 8); };

  CHECK(t.insert(K(1), 7));
  CHECK(t.insert(K(1), 8));  // the same Ptr again: the token is replaced
  CHECK(t.size() == 1 && contents(t)[8] == 8);
  CHECK(!t.insert(nullptr, 1));
  t.rekey(K(9), K(2));  // not registered: ignored
  CHECK(t.size() == 1 && contents(t).count(16) == 0);
  t.rekey(K(1), K(1));  // onto itself: unchanged
  CHECK(t.size() == 1 && contents(t)[8] == 8);
  t.rekey(K(1), K(2));
  CHECK(t.size() == 1 && contents(t).count(8) == 0 && contents(t)[16] == 8);

  // A full table still re-keys and still replaces tokens.
  CHECK(t.insert(K(3), 1) && t.insert(K(4), 1) && t.insert(K(5), 1));
  CHECK(t.size() == 4 && !t.insert(K(6), 1));
  t.rekey(K(3), K(6));
  CHECK(t.size() == 4 && contents(t).count(48) == 1 &&
        contents(t).count(24) == 0);
  CHECK(t.insert(K(6), 2));
  // Re-keying onto a key that is registered merges the two.
  t.rekey(K(4), K(5));
  CHECK(t.size() == 3 && contents(t)[40] == 1);
  t.erase(K(77));  // absent: ignored
  CHECK(t.size() == 3);
}

void testTableInit() {
  CHECK(PtrTable::bytesFor(0) == 0);
  CHECK(PtrTable::bytesFor(4096) == 2 * 4096 * 2 * sizeof(uintptr_t));
  CHECK(PtrTable::bytesFor(1) == 2 * 2 * sizeof(uintptr_t));
  CHECK(PtrTable::bytesFor(8192) == 2 * PtrTable::bytesFor(4096));

  std::vector<uintptr_t> store = storageFor(8);
  PtrTable t;
  CHECK(throwsFailure([&] { t.init(store.data(), 6); }, "power of two"));
  CHECK(t.capacity() == 0 && !t.insert(reinterpret_cast<void*>(8), 1));
  CHECK(throwsFailure([&] { t.init(nullptr, 8); }, "storage"));
  CHECK(throwsFailure(
      [&] { t.init(reinterpret_cast<char*>(store.data()) + 1, 8); },
      "storage"));

  t.init(nullptr, 0);  // a table that holds nothing
  CHECK(t.capacity() == 0 && t.size() == 0);
  CHECK(!t.insert(reinterpret_cast<void*>(8), 1));
  t.erase(reinterpret_cast<void*>(8));
  t.rekey(reinterpret_cast<void*>(8), reinterpret_cast<void*>(16));
  t.dropBucket(1);
  t.resolve(nullptr, 0);

  // init forgets what the table held.
  t.init(store.data(), 8);
  CHECK(t.insert(reinterpret_cast<void*>(8), 1));
  t.init(store.data(), 8);
  CHECK(t.size() == 0 && contents(t).empty());

  PtrRegistry reg;
  CHECK(throwsFailure(
      [&] { reg.init(store.data(), 8, 200, 100, nullptr, nullptr); },
      "reversed"));
  reg.init(store.data(), 8, 100, 200, nullptr, nullptr);
  CHECK(reg.lo() == 100 && reg.hi() == 200);
  CHECK(reg.contains(reinterpret_cast<void*>(100)) &&
        reg.contains(reinterpret_cast<void*>(199)) &&
        !reg.contains(reinterpret_cast<void*>(200)) &&
        !reg.contains(reinterpret_cast<void*>(99)));
  CHECK(reg.tokenAt(reinterpret_cast<void*>(150)) == 0);  // no tokenAt given
}

// ---- failures
// ------------------------------------------------------------------

void testFull() {
  World w(4);
  w.zones = {{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes, 1}};
  PtrTable& table = w.table();
  CHECK(table.capacity() == 4);

  std::vector<Ptr<int>> held;
  held.reserve(8);
  for (int i = 0; i < 4; ++i) held.push_back(Ptr<int>(intIn(0, i)));
  CHECK(table.size() == 4);

  capturedFailureLines().clear();
  CHECK(throwsFailure([&] { Ptr<int> fifth(intIn(0, 4)); }, "ptr table full"));
  CHECK(!capturedFailureLines().empty() &&
        capturedFailureLines().back().find("[MEM] FAIL ptr table full") == 0);
  CHECK(table.size() == 4);  // the fifth was not made, nothing else changed
  CHECK(throwsFailure([&] { Ptr<int> copy = held[0]; }, "ptr table full"));
  CHECK(table.size() == 4);

  // What needs no new slot works on a full table.
  Ptr<int> a = std::move(held[0]);  // a move
  CHECK(table.size() == 4 && !held[0]);
  held[1] = held[2];  // in -> in: the token is replaced
  CHECK(table.size() == 4);
  a = held[3];
  CHECK(table.size() == 4);
  Ptr<int> b = std::move(a);
  CHECK(table.size() == 4 && !a && b.get() == held[3].get());

  // What does need one fails and leaves the Ptr as it was.
  int outside = 0;
  Ptr<int> o(&outside);
  CHECK(throwsFailure([&] { o = b; }, "ptr table full"));
  CHECK(o.get() == &outside && table.size() == 4);

  // Room comes back when a Ptr is let go.
  held[3].reset();
  CHECK(table.size() == 3);
  o = b;
  CHECK(table.size() == 4 && o.get() == b.get());

  held.clear();
  b.reset();
  o.reset();
  CHECK(table.size() == 0);
}

void testNoRegistry() {
  // The range is set but no table is installed: nothing is registered, and
  // nothing breaks.
  World w;
  w.zones = {{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes, 1}};
  detail::g_ptrs = nullptr;
  {
    Ptr<int> p(intIn(0, 0));
    Ptr<int> q = p;
    q = Ptr<int>(intIn(0, 1));
    p.reset();
    Pin<int> pin(q);
  }
  CHECK(w.table().size() == 0);
  w.install();

  // detail::attach itself ignores a target that is outside the registry's
  // range.
  int outside = 0;
  Ptr<int> holder;
  detail::attach(&holder, &outside);
  CHECK(w.table().size() == 0);
  detail::detach(&holder);  // not registered: ignored
  detail::moved(&holder, &outside);
  CHECK(w.table().size() == 0);

  // With no manager at all (the default), the range is empty and no Ptr is
  // registered.
  World::uninstall();
  {
    Ptr<int> p(intIn(0, 0));
    Ptr<int> q = p;
    CHECK(q.get() == intIn(0, 0));
  }
  CHECK(w.table().size() == 0);
  w.install();
}

// ---- pins
// --------------------------------------------------------------------

struct PinLog {
  std::vector<const void*> pinned;
  std::vector<const void*> unpinned;
};

void fakePin(const void* raw, void* ctx) {
  static_cast<PinLog*>(ctx)->pinned.push_back(raw);
}

void fakeUnpin(const void* raw, void* ctx) {
  static_cast<PinLog*>(ctx)->unpinned.push_back(raw);
}

void testPin() {
  World w;
  w.zones = {{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes, 1}};
  int outside = 0;
  Ptr<int> inside(intIn(0, 3)), out(&outside), empty;

  // No hook: pin and unpin do nothing.
  CHECK(detail::g_pinHook == nullptr);
  {
    Pin<int> pin(inside);
  }
  detail::pin(nullptr);
  detail::unpin(nullptr);

  PinLog log;
  detail::PinHook hook = {&fakePin, &fakeUnpin, &log};
  detail::g_pinHook = &hook;
  {
    Pin<int> a(inside);
    CHECK(log.pinned.size() == 1 && log.pinned[0] == intIn(0, 3));
    CHECK(log.unpinned.empty());
    {
      Pin<int> b(inside);  // pins nest
      CHECK(log.pinned.size() == 2);
    }
    CHECK(log.unpinned.size() == 1 && log.unpinned[0] == intIn(0, 3));
    Pin<int> none(empty);   // nothing to pin
    Pin<int> notHeap(out);  // not in the movable heap
    CHECK(log.pinned.size() == 2);
  }
  CHECK(log.unpinned.size() == 2);
  CHECK(log.pinned.size() == 2);

  // A Pin keeps what it pinned even if the Ptr is reset meanwhile.
  {
    Ptr<int> p(intIn(0, 7));
    Pin<int> pin(p);
    p.reset();
  }
  CHECK(log.unpinned.size() == 3 && log.unpinned[2] == intIn(0, 7));

  // A hook with a missing function is a no-op for that one.
  detail::PinHook half = {nullptr, &fakeUnpin, &log};
  detail::g_pinHook = &half;
  {
    Pin<int> pin(inside);
  }
  CHECK(log.pinned.size() == 3 && log.unpinned.size() == 4);
  detail::g_pinHook = nullptr;
}

// ---- ids (TYRA_MEMORY_CHECKS)
// ----------------------------------------------------

#if TYRA_MEMORY_CHECKS

bool inArena(uintptr_t a) {
  return a >= reinterpret_cast<uintptr_t>(g_arena) &&
         a < reinterpret_cast<uintptr_t>(g_arena) + sizeof g_arena;
}

struct IdLog {
  std::vector<const void*> dead;
  int idCalls = 0;
  int liveCalls = 0;
  Id lastChecked = kEmptyId;
  bool everything = false;  // report every entry dead
};

Id fakeIdOf(const void* raw, void* ctx) {
  IdLog* log = static_cast<IdLog*>(ctx);
  ++log->idCalls;
  if (!inArena(reinterpret_cast<uintptr_t>(raw))) return kEmptyId;
  return Id{3, 1, 2, 5};
}

bool fakeCheckLive(const void* raw, const Id& id, void* ctx) {
  IdLog* log = static_cast<IdLog*>(ctx);
  ++log->liveCalls;
  log->lastChecked = id;
  if (log->everything) return false;
  return std::find(log->dead.begin(), log->dead.end(), raw) == log->dead.end();
}

bool sameId(const Id& a, const Id& b) {
  return a.pool == b.pool && a.bucket == b.bucket && a.index == b.index &&
         a.generation == b.generation;
}

void testIds() {
  World w;
  w.zones = {{reinterpret_cast<uintptr_t>(slot(0)), kSlotBytes, 1}};
  int outside = 0;
  int* inside = intIn(0, 2);

  // The empty id.
  CHECK(kEmptyId.pool == 0xFFFF && isEmptyId(kEmptyId));
  CHECK(!isEmptyId(Id{0, 0, 0, 0}));

  // No hook: the Ptr has no id, nothing is checked, nothing breaks.
  CHECK(detail::g_idHook == nullptr);
  CHECK(detail::checkLive(inside, Id{1, 2, 3, 4}));
  CHECK(isEmptyId(detail::idOf(inside)));
  {
    Ptr<int> p(inside), d;
    CHECK(isEmptyId(p.id()) && isEmptyId(d.id()));
    CHECK(p.get() == inside);
  }

  IdLog log;
  detail::IdHook hook = {&fakeCheckLive, &fakeIdOf, &log};
  detail::g_idHook = &hook;

  Ptr<int> p(inside);
  CHECK(log.idCalls == 1 && sameId(p.id(), Id{3, 1, 2, 5}));
  CHECK(sameId(detail::idOf(inside), Id{3, 1, 2, 5}));
  Ptr<int> o(&outside);
  CHECK(isEmptyId(o.id()));
  Ptr<int> n;
  Ptr<int> nn(static_cast<int*>(nullptr));
  CHECK(isEmptyId(n.id()) && isEmptyId(nn.id()));

  // Copies and moves carry the id; the source of a move loses it.
  Ptr<int> c = p;
  CHECK(sameId(c.id(), Id{3, 1, 2, 5}));
  Ptr<int> m = std::move(c);
  CHECK(sameId(m.id(), Id{3, 1, 2, 5}) && isEmptyId(c.id()));
  Ptr<int> a;
  a = m;
  CHECK(sameId(a.id(), Id{3, 1, 2, 5}));
  Ptr<int> b;
  b = std::move(a);
  CHECK(sameId(b.id(), Id{3, 1, 2, 5}) && isEmptyId(a.id()));
  b.reset();
  CHECK(isEmptyId(b.id()));
  const int idCalls = log.idCalls;
  Ptr<int> copy = p;  // a copy does not ask again
  copy = p;
  CHECK(log.idCalls == idCalls);

  // get() checks a Ptr that has an id.
  log.liveCalls = 0;
  CHECK(p.get() == inside);
  CHECK(log.liveCalls == 1 && sameId(log.lastChecked, Id{3, 1, 2, 5}));
  CHECK(p.operator->() == inside && &*p == inside);
  CHECK(log.liveCalls == 3);
  CHECK(bool(p));  // not a liveness check
  CHECK(log.liveCalls == 3);

  // ...but not a Ptr that has none, or none at all.
  log.everything = true;
  log.liveCalls = 0;
  CHECK(o.get() == &outside);
  CHECK(n.get() == nullptr);
  CHECK(log.liveCalls == 0);

  // A dead one fails, through all three accessors.
  CHECK(throwsFailure([&] { (void)p.get(); }, "ptr to a dead object"));
  CHECK(throwsFailure([&] { (void)p.operator->(); }, "ptr to a dead object"));
  CHECK(throwsFailure([&] { (void)*p; }, "ptr to a dead object"));
  CHECK(capturedFailureLines().back().find(
            "[MEM] FAIL ptr to a dead object name=ptr size=0 caller=0x") == 0);
  log.everything = false;
  log.dead.push_back(inside);
  CHECK(throwsFailure([&] { (void)p.get(); }, "ptr to a dead object"));
  log.dead.clear();
  CHECK(p.get() == inside);

  // Layout: a Ptr is larger than a raw pointer with the checks on.
  CHECK(sizeof(Ptr<int>) > sizeof(int*));

  // The registry is the same with ids: a moved Ptr keeps its id and the
  // table still resolves it.
  m.reset();
  p.reset();
  copy.reset();
  detail::g_idHook = nullptr;
  CHECK(w.table().size() == 0);
}

#endif  // TYRA_MEMORY_CHECKS

}  // namespace

int main() {
  installThrowingFailure();
  testBrief();
  testLifecycle();
  testResolveBrief();
  testResolveOverlap();
  testResolveSwap();
  testResolveNoOps();
  for (uint32_t seed = 1; seed <= 5; ++seed) testRandomMoves(seed);
  testDropBucket();
  testTableInit();
  testTableRekeyAndReplace();
  testTableModel();
  testTableFullAndUnwind();
  testFull();
  testNoRegistry();
  testPin();
#if TYRA_MEMORY_CHECKS
  testIds();
#endif
  return finish("ptr");
}
