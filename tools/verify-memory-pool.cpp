// Added by TyraX: host verifier for the pools of linked buckets
// (inc/memory/pool.hpp, src/memory/pool_core.cpp). Built and run by
// tools/verify-memory.py, once with TYRA_MEMORY_CHECKS=0 and once with =1.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "memory/pool.hpp"
#include "memory/registry.hpp"
#include "verify-memory-check.hpp"

namespace {

// ---- a bucket source that counts what it is asked --------------------------

// Hands out host memory (filled with junk, so a pool that relies on zeroed
// memory is found), remembers every block, and gives the junk back with
// delete[] when a bucket is released - so a pool that touches a released or
// moved bucket is caught by the sanitizers.
class FakeSource : public BucketSource {
 public:
  int calls = 0;
  int released = 0;
  bool refuse = false;
  size_t lastBytes = 0;
  size_t lastAlign = 0;
  bool lastPinned = false;
  uint32_t lastOwner = 0;
  BudgetId lastBudget = 0;
  void* lastReleased = nullptr;

  ~FakeSource() override {
    for (auto& kv : blocks_) delete[] kv.second;
  }

  void* rawAlloc(size_t bytes, size_t align) {
    const size_t a = std::max<size_t>(align, 16);
    unsigned char* raw = new unsigned char[bytes + a];
    unsigned char* p = reinterpret_cast<unsigned char*>(
        (reinterpret_cast<uintptr_t>(raw) + a - 1) / a * a);
    std::memset(p, 0xA5, bytes);
    blocks_[p] = raw;
    return p;
  }

  void rawFree(void* p) {
    auto it = blocks_.find(p);
    CHECK(it != blocks_.end());
    if (it == blocks_.end()) return;
    delete[] it->second;
    blocks_.erase(it);
  }

  size_t blocks() const { return blocks_.size(); }

  void* allocateBucket(size_t bytes, size_t align, bool pinned, uint32_t owner,
                       BudgetId budget) override {
    ++calls;
    lastBytes = bytes;
    lastAlign = align;
    lastPinned = pinned;
    lastOwner = owner;
    lastBudget = budget;
    return refuse ? nullptr : rawAlloc(bytes, align);
  }

  void releaseBucket(void* base) override {
    ++released;
    lastReleased = base;
    rawFree(base);
  }

 private:
  std::map<void*, unsigned char*> blocks_;
};

// A registry with one pool, its first bucket in host memory (junk-filled) and a
// PoolCore initialised on it.
struct Rig {
  Registry reg;
  PoolDesc desc;
  PoolId id;
  FakeSource src;
  std::vector<unsigned char> storage;
  unsigned char* first;
  PoolCore pool;

  explicit Rig(const PoolDesc& d, Kind kind = Kind::Logic,
               bool withSource = true)
      : desc(d) {
    reg.budget(d.budget, "probe", kind);
    id = reg.pool(d);
    storage.assign(reg.poolFixedBytes(id) + 512, 0xCD);
    const uintptr_t unit = std::max<uint32_t>(d.align, 16);
    first = reinterpret_cast<unsigned char*>(
        (reinterpret_cast<uintptr_t>(storage.data()) + unit - 1) / unit * unit);
    pool.init(id, desc, reg, first, withSource ? &src : nullptr);
  }

  size_t extBytes() const { return reg.bucketBytes(id, desc.extensionCount); }
  size_t firstBytes() const { return reg.poolFixedBytes(id); }
};

bool inside(const void* p, const void* base, size_t bytes) {
  const uintptr_t a = reinterpret_cast<uintptr_t>(p);
  const uintptr_t b = reinterpret_cast<uintptr_t>(base);
  return a >= b && a < b + bytes;
}

bool distinct(void* const* p, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      if (p[i] == p[j]) return false;
    }
  }
  return true;
}

// The (bucket, index) an entry pointer is, found by walking the pool's own
// accessors - independent of how the pool finds it.
bool where(const PoolCore& pool, const void* entry, uint16_t& bucket,
           uint32_t& index) {
  for (uint32_t b = 0; b < pool.bucketCount(); ++b) {
    for (uint32_t i = 0; i < pool.countIn(static_cast<uint16_t>(b)); ++i) {
      if (pool.entryAt(static_cast<uint16_t>(b), i) == entry) {
        bucket = static_cast<uint16_t>(b);
        index = i;
        return true;
      }
    }
  }
  return false;
}

// Every accessor agrees with every other.
bool consistent(const PoolCore& pool) {
  uint32_t live = 0, capacity = 0;
  for (uint32_t b = 0; b < pool.bucketCount(); ++b) {
    const uint16_t bk = static_cast<uint16_t>(b);
    capacity += pool.countIn(bk);
    for (uint32_t i = 0; i < pool.countIn(bk); ++i) {
      if (pool.entryAt(bk, i) == nullptr) return false;
      if (pool.isLive(bk, i)) ++live;
    }
  }
  return live == pool.live() && capacity == pool.capacity() &&
         pool.live() <= pool.capacity();
}

void fill(void* p, size_t n, unsigned char v) { std::memset(p, v, n); }

// ---- the brief's checks ----------------------------------------------------

void testBrief() {
  // first bucket: 4 entries, extensions of 4, at most 2
  Rig rig({"probe", Budget::FirstGame, 24, 8, 4, 4, 2, Policy::Refuse});
  PoolCore& pool = rig.pool;
  FakeSource& src = rig.src;
  const PoolId id = rig.id;
  Registry& reg = rig.reg;

  CHECK(pool.live() == 0 && pool.capacity() == 4 && pool.bucketCount() == 1 &&
        pool.exhaustions() == 0);
  CHECK(std::strcmp(pool.name(), "probe") == 0);

  void* e[12];
  for (int i = 0; i < 4; ++i) e[i] = pool.acquire();
  CHECK(src.calls == 0);
  CHECK(distinct(e, 4));
  bool insideFirst = true;
  for (int i = 0; i < 4; ++i) {
    insideFirst = insideFirst && inside(e[i], rig.first, rig.firstBytes());
  }
  CHECK(insideFirst);
  CHECK(uintptr_t(e[1]) % 8 == 0);
  // Fresh entries come in address order, one stride (24) apart.
  CHECK(uintptr_t(e[1]) - uintptr_t(e[0]) == 24);
  CHECK(uintptr_t(e[3]) - uintptr_t(e[2]) == 24);
  CHECK(pool.live() == 4 && pool.capacity() == 4);

  e[4] = pool.acquire();
  CHECK(e[4] != nullptr);
  CHECK(src.calls == 1 && src.lastBytes == reg.bucketBytes(id, 4));
  CHECK(src.lastAlign >= 16 && src.lastAlign >= rig.desc.align);
  CHECK(!src.lastPinned);
  CHECK(src.lastOwner == bucketToken(id, 1));
  CHECK(src.lastBudget == Budget::FirstGame);
  CHECK(pool.bucketCount() == 2 && pool.capacity() == 8);
  CHECK(src.lastReleased == nullptr);  // nothing has gone back yet
  CHECK(inside(e[4], pool.entryAt(1, 0), rig.extBytes()));

  for (int i = 5; i < 12; ++i) e[i] = pool.acquire();  // fills extension 2
  CHECK(src.calls == 2 && src.lastOwner == bucketToken(id, 2));
  CHECK(pool.bucketCount() == 3 && pool.capacity() == 12 && pool.live() == 12);
  CHECK(distinct(e, 12));
  for (int i = 4; i < 8; ++i) {
    CHECK(inside(e[i], pool.entryAt(1, 0), rig.extBytes()));
  }
  for (int i = 8; i < 12; ++i) {
    CHECK(inside(e[i], pool.entryAt(2, 0), rig.extBytes()));
  }
  CHECK(consistent(pool));

  CHECK(pool.acquire() == nullptr && pool.exhaustions() == 1);
  CHECK(src.calls == 2);  // the cap is the pool's own: no third request
  CHECK(pool.acquire() == nullptr && pool.exhaustions() == 2);
  CHECK(pool.live() == 12);

  // The lowest-numbered bucket with a free entry is the one that serves.
  pool.release(e[5]);
  pool.release(e[2]);
  CHECK(pool.live() == 10);
  CHECK(pool.acquire() == e[2]);
  CHECK(pool.acquire() == e[5]);
  CHECK(pool.live() == 12);

  // release, reuse, live counts; empty extensions go back, one spare stays
  for (int i = 4; i < 12; ++i) pool.release(e[i]);
  CHECK(pool.live() == 4);
  pool.releaseEmptyExtensions();
  CHECK(src.released == 1 && pool.bucketCount() == 2 && pool.live() == 4);
  CHECK(pool.capacity() == 8);

  // a moved bucket keeps working
  unsigned char* oldBase = static_cast<unsigned char*>(pool.entryAt(1, 0));
  const size_t bytes = rig.extBytes();
  void* a = pool.acquire();  // first free entry in the lowest bucket: bucket 1
  CHECK(inside(a, oldBase, bytes));
  fill(a, 24, 0x5A);
  unsigned char* newBase = static_cast<unsigned char*>(src.rawAlloc(bytes, 16));
  std::memcpy(newBase, oldBase, bytes);
  std::memset(oldBase, 0xDD, bytes);
  const size_t blocksBefore = src.blocks();
  pool.bucketMoved(1, newBase);
  src.rawFree(oldBase);
  CHECK(src.blocks() == blocksBefore - 1);
  CHECK(pool.entryAt(1, 0) == newBase);
  void* moved = static_cast<unsigned char*>(a) - oldBase + newBase;
  CHECK(pool.isLive(1, 0) || pool.isLive(1, 1) || pool.isLive(1, 2) ||
        pool.isLive(1, 3));
  CHECK(allBytes(moved, 24, 0x5A));  // the live entry came along
  void* m = pool.acquire();
  CHECK(inside(m, newBase, bytes));
  CHECK(m != moved);
  CHECK(pool.live() == 6);
  CHECK(consistent(pool));
  pool.release(moved);  // released by its new address
  pool.release(m);
  CHECK(pool.live() == 4);
  // The pool hands the bucket back by where it is now.
  pool.release(e[3]);
  pool.release(e[0]);
  pool.release(e[1]);
  pool.release(e[2]);
  CHECK(pool.live() == 0);
  pool.bucketMoved(1, newBase);  // a no-op move to the same place is fine

#if TYRA_MEMORY_CHECKS
  void* f0 = pool.acquire();
  void* f1 = pool.acquire();
  CHECK(f0 != nullptr && f1 != nullptr);
  Id old = pool.idOf(f0);
  CHECK(old.pool == id && old.bucket == 0);
  pool.release(f0);
  void* again = pool.acquire();
  CHECK(again == f0 && !pool.valid(old, again) &&
        pool.valid(pool.idOf(again), again));
  CHECK(pool.valid(pool.idOf(f1), f1));
#endif
}

// ---- tokens and the shape of an id -----------------------------------------

void testTokens() {
  CHECK(bucketToken(0, 0) == (1u << 16));
  CHECK(bucketToken(0, 5) == ((1u << 16) | 5));
  CHECK(bucketToken(3, 63) == ((4u << 16) | 63));
  CHECK(bucketToken(0, 1) != bucketToken(1, 1));
  CHECK(bucketToken(7, 2) != 0);
#if TYRA_MEMORY_CHECKS
  CHECK(sizeof(Id) == 8);
  Id i = {1, 2, 3, 4};
  CHECK(i.pool == 1 && i.bucket == 2 && i.index == 3 && i.generation == 4);
#endif
}

// ---- strides, alignment, what the source is told ---------------------------

void testStrideAndAlign() {
  {
    // typeSize 100 at align 16: stride 112.
    Rig r({"wide", Budget::FirstGame, 100, 16, 3, 0, 0, Policy::Refuse});
    void* a = r.pool.acquire();
    void* b = r.pool.acquire();
    CHECK(uintptr_t(b) - uintptr_t(a) == 112);
    CHECK(uintptr_t(a) % 16 == 0 && uintptr_t(b) % 16 == 0);
  }
  {
    // A tiny type still has room for the free-list link: stride 4.
    Rig r({"tiny", Budget::FirstGame, 1, 1, 6, 0, 0, Policy::Refuse});
    void* e[6];
    for (void*& p : e) p = r.pool.acquire();
    CHECK(distinct(e, 6));
    CHECK(uintptr_t(e[1]) - uintptr_t(e[0]) == 4);
    CHECK(r.pool.acquire() == nullptr && r.pool.exhaustions() == 1);
    for (void* p : e) r.pool.release(p);
    CHECK(r.pool.live() == 0);
    void* again[6];
    for (void*& p : again) p = r.pool.acquire();
    CHECK(distinct(again, 6));
  }
  {
    // Alignment: 8 asks the source for 16, pinned only when the budget is
    // Render or the alignment is above 16.
    struct Row {
      uint32_t align;
      Kind kind;
      size_t askedAlign;
      bool pinned;
    };
    const Row rows[] = {
        {1, Kind::Logic, 16, false},    {8, Kind::Logic, 16, false},
        {16, Kind::Logic, 16, false},   {32, Kind::Logic, 32, true},
        {64, Kind::Logic, 64, true},    {128, Kind::Logic, 128, true},
        {8, Kind::Render, 16, true},    {16, Kind::Render, 16, true},
        {1, Kind::Asset, 16, false},    {16, Kind::System, 16, false},
        {16, Kind::Scratch, 16, false}, {64, Kind::Render, 64, true},
    };
    for (const Row& row : rows) {
      Rig r({"a", Budget::FirstGame, 40, row.align, 2, 2, 1, Policy::Refuse},
            row.kind);
      void* e[4];
      for (void*& p : e) p = r.pool.acquire();
      CHECK(r.src.calls == 1);
      CHECK(r.src.lastAlign == row.askedAlign);
      CHECK(r.src.lastPinned == row.pinned);
      for (void* p : e) {
        CHECK(p != nullptr &&
              uintptr_t(p) % std::max<uint32_t>(row.align, 1) == 0);
      }
      CHECK(distinct(e, 4));
    }
  }
  {
    // The budget's kind is read from the registry, not from the pool.
    Rig r({"a", Budget::Renderer, 40, 8, 1, 1, 1, Policy::Refuse},
          Kind::Render);
    r.pool.acquire();
    r.pool.acquire();
    CHECK(r.src.lastPinned && r.src.lastBudget == Budget::Renderer);
  }
}

// ---- Halt ------------------------------------------------------------------

void testHalt() {
  installThrowingFailure();
  Rig r({"halting", Budget::FirstGame, 24, 8, 2, 2, 1, Policy::Halt});
  void* e[4];
  for (void*& p : e) p = r.pool.acquire();
  CHECK(r.pool.live() == 4 && r.pool.exhaustions() == 0);
  CHECK(capturedFailureLines().empty());

  CHECK(throwsFailure([&] { r.pool.acquire(); }, "pool exhausted"));
  CHECK(capturedFailureLines().size() == 1);
  const std::string line = capturedFailureLines().back();
  CHECK(line.find("[MEM] FAIL pool exhausted name=halting size=24 caller=0x") ==
        0);
  CHECK(line.find("caller=0x0") == std::string::npos);  // a real address

  // The pool is untouched: a release makes room, then it serves again.
  CHECK(r.pool.live() == 4 && consistent(r.pool));
  CHECK(r.pool.exhaustions() == 1);  // counted, whatever the policy
  r.pool.release(e[1]);
  CHECK(r.pool.acquire() == e[1]);
  CHECK(r.src.calls == 1);

  // A source that refuses leaves Halt with nothing to do but halt.
  Rig q({"halting2", Budget::FirstGame, 24, 8, 1, 4, 2, Policy::Halt});
  q.src.refuse = true;
  CHECK(q.pool.acquire() != nullptr);
  CHECK(throwsFailure([&] { q.pool.acquire(); }, "pool exhausted"));
  CHECK(q.src.calls == 1);
  q.src.refuse = false;
  CHECK(q.pool.acquire() != nullptr);  // and it takes the extension after all
  CHECK(q.pool.bucketCount() == 2);
}

// ---- Refuse ----------------------------------------------------------------

void testRefuse() {
  {
    // No extensions at all: the first bucket is the pool.
    Rig r({"fixed", Budget::FirstGame, 16, 16, 3, 0, 0, Policy::Refuse});
    for (int i = 0; i < 3; ++i) CHECK(r.pool.acquire() != nullptr);
    CHECK(r.pool.acquire() == nullptr);
    CHECK(r.pool.exhaustions() == 1 && r.src.calls == 0);
  }
  {
    // An extension size of 0 means none, whatever the cap says.
    Rig r({"fixed", Budget::FirstGame, 16, 16, 2, 0, 4, Policy::Refuse});
    r.pool.acquire();
    r.pool.acquire();
    CHECK(r.pool.acquire() == nullptr && r.src.calls == 0);
  }
  {
    // No source: the same.
    Rig r({"nosrc", Budget::FirstGame, 16, 16, 2, 4, 4, Policy::Refuse},
          Kind::Logic, false);
    r.pool.acquire();
    r.pool.acquire();
    CHECK(r.pool.acquire() == nullptr && r.pool.exhaustions() == 1);
  }
  {
    // A source that has no room: refused, counted, and nothing changes; when
    // it has room again the pool grows.
    Rig r({"full", Budget::FirstGame, 16, 16, 1, 2, 2, Policy::Refuse});
    CHECK(r.pool.acquire() != nullptr);
    r.src.refuse = true;
    CHECK(r.pool.acquire() == nullptr && r.pool.exhaustions() == 1);
    CHECK(r.pool.acquire() == nullptr && r.pool.exhaustions() == 2);
    CHECK(r.src.calls == 2 && r.pool.bucketCount() == 1);
    CHECK(r.pool.capacity() == 1 && r.pool.live() == 1 && consistent(r.pool));
    r.src.refuse = false;
    CHECK(r.pool.acquire() != nullptr);
    CHECK(r.pool.bucketCount() == 2 && r.pool.capacity() == 3);
    CHECK(r.pool.exhaustions() == 2);
  }
  {
    // A pool with no first bucket: everything is an extension.
    Rig r({"lazy", Budget::FirstGame, 16, 16, 0, 2, 2, Policy::Refuse});
    CHECK(r.pool.capacity() == 0 && r.pool.bucketCount() == 1);
    CHECK(r.pool.countIn(0) == 0 && r.pool.entryAt(0, 0) == nullptr);
    void* a = r.pool.acquire();
    CHECK(a != nullptr && r.src.calls == 1);
    CHECK(r.src.lastOwner == bucketToken(r.id, 1));
    CHECK(r.pool.bucketCount() == 2 && r.pool.capacity() == 2);
    CHECK(r.pool.acquire() != nullptr);
    CHECK(r.pool.acquire() != nullptr);
    CHECK(r.src.calls == 2 && r.src.lastOwner == bucketToken(r.id, 2));
    r.pool.acquire();
    CHECK(r.pool.acquire() == nullptr && r.pool.exhaustions() == 1);
  }
  {
    // Nothing at all: an empty pool refuses, it does not crash.
    Rig r({"none", Budget::FirstGame, 16, 16, 0, 0, 0, Policy::Refuse});
    CHECK(r.pool.acquire() == nullptr && r.pool.exhaustions() == 1);
    CHECK(r.pool.capacity() == 0 && r.pool.live() == 0);
  }
}

// ---- RecycleOldest ---------------------------------------------------------

struct HookLog {
  std::vector<void*> seen;
  std::vector<void*> ctxs;
  bool bytesIntact = true;
  unsigned char expect = 0;
  size_t bytes = 0;
};

void recycleHook(void* entry, void* ctx) {
  HookLog* log = static_cast<HookLog*>(ctx);
  log->seen.push_back(entry);
  log->ctxs.push_back(ctx);
  // The entry is still the object it was: nothing was written over it yet.
  const unsigned char* b = static_cast<const unsigned char*>(entry);
  for (size_t i = 0; i < log->bytes; ++i) {
    if (b[i] != log->expect) log->bytesIntact = false;
  }
}

void testRecycleOldest() {
  {
    Rig r({"decal", Budget::FirstGame, 24, 8, 3, 0, 0, Policy::RecycleOldest});
    HookLog log;
    log.bytes = 24;
    log.expect = 0x77;
    r.pool.setRecycle(&recycleHook, &log);

    void* a = r.pool.acquire();
    void* b = r.pool.acquire();
    void* c = r.pool.acquire();
    fill(a, 24, 0x77);
    fill(b, 24, 0x77);
    fill(c, 24, 0x77);
    CHECK(log.seen.empty() && r.pool.exhaustions() == 0);

    // Full: the oldest is recycled, handed to the hook, and returned.
    void* x = r.pool.acquire();
    CHECK(x == a && log.seen.size() == 1 && log.seen[0] == a);
    CHECK(log.ctxs[0] == &log && log.bytesIntact);
    CHECK(r.pool.live() == 3 && r.pool.exhaustions() == 1);
    fill(x, 24, 0x77);
    // Now b is the oldest, then c, then the recycled a again.
    CHECK(r.pool.acquire() == b);
    CHECK(r.pool.acquire() == c);
    CHECK(r.pool.acquire() == a);
    CHECK(r.pool.acquire() == b);
    CHECK(log.seen.size() == 5 && r.pool.exhaustions() == 5);
    CHECK(r.pool.live() == 3 && consistent(r.pool));
    CHECK(r.src.calls == 0);

    // A free entry is used before anything is recycled.
    r.pool.release(c);
    const size_t calls = log.seen.size();
    CHECK(r.pool.acquire() == c && log.seen.size() == calls);
    CHECK(r.pool.exhaustions() == 5);

    // Age is when an entry was acquired, not where it is: after c's reuse the
    // oldest is a (acquired before b and c).
    CHECK(r.pool.acquire() == a);

    // No hook: the entry is still handed back.
    r.pool.setRecycle(nullptr, nullptr);
    void* y = r.pool.acquire();
    CHECK(y == b);
    CHECK(log.seen.size() == calls + 1);
  }
  {
    // Extensions first, recycling when the pool cannot grow any more; the
    // oldest is the oldest of all buckets.
    Rig r({"fx", Budget::FirstGame, 24, 8, 2, 2, 1, Policy::RecycleOldest});
    HookLog log;
    r.pool.setRecycle(&recycleHook, &log);
    void* e[4];
    for (void*& p : e) p = r.pool.acquire();
    CHECK(r.src.calls == 1 && log.seen.empty());
    CHECK(r.pool.bucketCount() == 2 && r.pool.capacity() == 4);

    // Age by acquisition, not by position: release and take the first entry
    // again and it is the newest.
    r.pool.release(e[0]);
    CHECK(r.pool.acquire() == e[0]);
    CHECK(r.pool.acquire() == e[1]);  // the oldest of all, in the first bucket
    CHECK(r.pool.acquire() == e[2]);  // then the one in the extension
    CHECK(r.pool.acquire() == e[3]);
    CHECK(r.pool.acquire() == e[0]);
    CHECK(r.src.calls == 1 && log.seen.size() == 4);

    // A source with no room recycles too.
    Rig q({"fx2", Budget::FirstGame, 24, 8, 1, 2, 2, Policy::RecycleOldest});
    q.src.refuse = true;
    void* a = q.pool.acquire();
    CHECK(q.pool.acquire() == a);
    CHECK(q.src.calls == 1 && q.pool.exhaustions() == 1);
  }
  {
    // A bucket that was moved took its stamps along: the oldest is still the
    // oldest, and it is found at the bucket's new address.
    Rig r({"fx", Budget::FirstGame, 24, 8, 1, 2, 1, Policy::RecycleOldest});
    void* a = r.pool.acquire();
    void* b = r.pool.acquire();
    void* c = r.pool.acquire();
    unsigned char* oldBase = static_cast<unsigned char*>(r.pool.entryAt(1, 0));
    unsigned char* newBase =
        static_cast<unsigned char*>(r.src.rawAlloc(r.extBytes(), 16));
    std::memcpy(newBase, oldBase, r.extBytes());
    std::memset(oldBase, 0xDD, r.extBytes());
    r.pool.bucketMoved(1, newBase);
    r.src.rawFree(oldBase);
    b = newBase + (static_cast<unsigned char*>(b) - oldBase);
    c = newBase + (static_cast<unsigned char*>(c) - oldBase);
    CHECK(r.pool.acquire() == a);  // oldest, in the first bucket
    CHECK(r.pool.acquire() == b);  // then the moved bucket's, by new address
    CHECK(r.pool.acquire() == c);
    CHECK(r.pool.acquire() == a);
    CHECK(r.pool.exhaustions() == 4 && r.src.calls == 1);
  }
  {
    // Nothing to recycle: an empty pool refuses.
    Rig r({"none", Budget::FirstGame, 24, 8, 0, 0, 0, Policy::RecycleOldest});
    CHECK(r.pool.acquire() == nullptr && r.pool.exhaustions() == 1);
  }
#if TYRA_MEMORY_CHECKS
  {
    // A recycled entry is a new object: ids of the old one stop being valid.
    Rig r({"decal", Budget::FirstGame, 24, 8, 2, 0, 0, Policy::RecycleOldest});
    void* a = r.pool.acquire();
    void* b = r.pool.acquire();
    Id ia = r.pool.idOf(a), ib = r.pool.idOf(b);
    void* x = r.pool.acquire();
    CHECK(x == a);
    CHECK(!r.pool.valid(ia, a) && r.pool.valid(r.pool.idOf(a), a));
    CHECK(r.pool.valid(ib, b));
  }
#endif
}

// ---- releasing extensions --------------------------------------------------

void fillPool(Rig& r, std::vector<void*>& out, uint32_t n) {
  for (uint32_t i = 0; i < n; ++i) {
    void* p = r.pool.acquire();
    CHECK(p != nullptr);
    out.push_back(p);
  }
}

// The entries of bucket `b`, among `all`.
std::vector<void*> entriesIn(const Rig& r, const std::vector<void*>& all,
                             uint16_t b) {
  std::vector<void*> v;
  for (void* p : all) {
    uint16_t bk;
    uint32_t i;
    if (where(r.pool, p, bk, i) && bk == b) v.push_back(p);
  }
  return v;
}

void testReleaseEmptyExtensions() {
  {
    // Nothing to release: only the first bucket.
    Rig r({"p", Budget::FirstGame, 24, 8, 4, 4, 4, Policy::Refuse});
    r.pool.releaseEmptyExtensions();
    CHECK(r.src.released == 0 && r.pool.bucketCount() == 1);
    // An empty first bucket is never released.
    std::vector<void*> v;
    fillPool(r, v, 4);
    for (void* p : v) r.pool.release(p);
    r.pool.releaseEmptyExtensions();
    CHECK(r.src.released == 0 && r.pool.capacity() == 4);
  }
  {
    // One empty extension is the spare and stays.
    Rig r({"p", Budget::FirstGame, 24, 8, 2, 2, 4, Policy::Refuse});
    std::vector<void*> v;
    fillPool(r, v, 4);
    r.pool.release(v[2]);
    r.pool.release(v[3]);
    r.pool.releaseEmptyExtensions();
    CHECK(r.src.released == 0 && r.pool.bucketCount() == 2);
    // It serves the next entry without asking the source.
    CHECK(r.pool.acquire() != nullptr && r.src.calls == 1);
  }
  {
    // Four extensions; 1, 3 and 4 are emptied: 1 is the spare, 3 and 4 go.
    Rig r({"p", Budget::FirstGame, 24, 8, 2, 2, 4, Policy::Refuse});
    std::vector<void*> v;
    fillPool(r, v, 2 + 4 * 2);
    CHECK(r.pool.bucketCount() == 5 && r.src.calls == 4);
    void* base3 = r.pool.entryAt(3, 0);
    void* base4 = r.pool.entryAt(4, 0);
    for (uint16_t b : {1, 3, 4}) {
      for (void* p : entriesIn(r, v, b)) r.pool.release(p);
    }
    r.pool.releaseEmptyExtensions();
    CHECK(r.src.released == 2);
    CHECK(r.src.lastReleased == base3 || r.src.lastReleased == base4);
    // The tail is gone: the numbering ends at the last bucket still held.
    CHECK(r.pool.bucketCount() == 3);
    CHECK(r.pool.live() == 2 + 2);
    CHECK(r.pool.capacity() == 2 + 2 * 2);
    CHECK(r.pool.countIn(1) == 2 && r.pool.countIn(2) == 2);
    CHECK(r.pool.countIn(3) == 0 && r.pool.entryAt(3, 0) == nullptr);
    CHECK(consistent(r.pool));
  }
  {
    // A bucket released from the middle leaves a gap that keeps its number;
    // the next extension takes the gap.
    Rig r({"p", Budget::FirstGame, 24, 8, 2, 2, 4, Policy::Refuse});
    std::vector<void*> v;
    fillPool(r, v, 2 + 4 * 2);
    for (uint16_t b : {2, 3}) {
      for (void* p : entriesIn(r, v, b)) r.pool.release(p);
    }
    // 2 is the spare, 3 goes: a gap below the still-held 4.
    r.pool.releaseEmptyExtensions();
    CHECK(r.src.released == 1);
    CHECK(r.pool.bucketCount() == 5);  // numbers 0..4 still name buckets
    CHECK(r.pool.countIn(3) == 0 && r.pool.entryAt(3, 1) == nullptr &&
          !r.pool.isLive(3, 0));
    CHECK(r.pool.countIn(2) == 2 && r.pool.countIn(4) == 2);
    CHECK(r.pool.capacity() == 2 + 3 * 2);
    CHECK(consistent(r.pool));
    CHECK(r.pool.live() == 2 + 2 + 2);

    // Fill the spare (bucket 2), and the next entry needs an extension: the
    // gap (3) is taken, not a sixth bucket.
    CHECK(r.pool.acquire() != nullptr && r.pool.acquire() != nullptr);
    CHECK(r.src.calls == 4);
    void* x = r.pool.acquire();
    CHECK(x != nullptr && r.src.calls == 5);
    CHECK(r.src.lastOwner == bucketToken(r.id, 3));
    CHECK(inside(x, r.pool.entryAt(3, 0), r.extBytes()));
    CHECK(r.pool.bucketCount() == 5 && r.pool.capacity() == 2 + 4 * 2);
    CHECK(consistent(r.pool));
  }
  {
    // The cap counts buckets held, not buckets ever made: after giving back,
    // the pool can grow again up to the cap.
    Rig r({"p", Budget::FirstGame, 24, 8, 1, 1, 2, Policy::Refuse});
    for (int round = 0; round < 3; ++round) {
      std::vector<void*> v;
      while (void* p = r.pool.acquire()) v.push_back(p);
      CHECK(v.size() == 3 && r.pool.live() == 3 && r.pool.capacity() == 3);
      for (void* p : v) r.pool.release(p);
      r.pool.releaseEmptyExtensions();
      // Two extensions were empty: one is the spare, one went back.
      CHECK(r.src.released == round + 1);
      CHECK(r.pool.bucketCount() == 2 && r.pool.capacity() == 2);
      CHECK(r.src.calls == round + 2);
    }
  }
  {
    // A moved bucket is given back by its new address.
    Rig r({"p", Budget::FirstGame, 24, 8, 1, 2, 2, Policy::Refuse});
    std::vector<void*> v;
    fillPool(r, v, 3);
    void* oldBase = r.pool.entryAt(1, 0);
    void* newBase = r.src.rawAlloc(r.extBytes(), 16);
    std::memcpy(newBase, oldBase, r.extBytes());
    r.pool.bucketMoved(1, newBase);
    r.src.rawFree(oldBase);
    r.pool.release(static_cast<unsigned char*>(newBase));
    r.pool.release(static_cast<unsigned char*>(newBase) + 24);
    r.pool.release(v[0]);
    // 1 is the only extension and empty: it is the spare. Take a second.
    r.pool.releaseEmptyExtensions();
    CHECK(r.src.released == 0);
    std::vector<void*> w;
    fillPool(r, w, 5);
    CHECK(r.pool.bucketCount() == 3);
    for (void* p : w) r.pool.release(p);
    r.pool.releaseEmptyExtensions();
    CHECK(r.src.released == 1 && r.pool.bucketCount() == 2);
    CHECK(r.src.lastReleased != nullptr);
    CHECK(r.pool.entryAt(1, 0) == newBase);
  }
}

// ---- the accessors at their edges ------------------------------------------

void testAccessors() {
  Rig r({"p", Budget::FirstGame, 24, 8, 3, 2, 2, Policy::Refuse});
  CHECK(r.pool.bucketCount() == 1);
  CHECK(r.pool.countIn(0) == 3);
  CHECK(r.pool.entryAt(0, 0) == r.first);
  CHECK(r.pool.entryAt(0, 2) == r.first + 48);
  CHECK(r.pool.entryAt(0, 3) == nullptr);  // past the bucket
  CHECK(r.pool.entryAt(1, 0) == nullptr);  // no such bucket yet
  CHECK(r.pool.entryAt(63, 0) == nullptr);
  CHECK(r.pool.entryAt(0xFFFF, 0) == nullptr);
  CHECK(!r.pool.isLive(0, 0) && !r.pool.isLive(0, 3) && !r.pool.isLive(9, 0));
  CHECK(r.pool.countIn(1) == 0 && r.pool.countIn(0xFFFF) == 0);

  void* a = r.pool.acquire();
  CHECK(a == r.pool.entryAt(0, 0) && r.pool.isLive(0, 0) &&
        !r.pool.isLive(0, 1));
  void* b = r.pool.acquire();
  r.pool.release(a);
  CHECK(!r.pool.isLive(0, 0) && r.pool.isLive(0, 1));
  CHECK(r.pool.live() == 1);
  r.pool.release(b);
  CHECK(r.pool.live() == 0 && r.pool.capacity() == 3);
  for (int i = 0; i < 4; ++i) r.pool.acquire();
  CHECK(r.pool.countIn(1) == 2 && r.pool.entryAt(1, 1) != nullptr);
  CHECK(r.pool.entryAt(1, 2) == nullptr);
  CHECK(r.pool.live() == 4 && r.pool.capacity() == 5);
}

// ---- init again, several pools ---------------------------------------------

void testReinitAndPools() {
  Rig r({"p", Budget::FirstGame, 24, 8, 2, 2, 2, Policy::Refuse});
  void* a = r.pool.acquire();
  r.pool.acquire();
  r.pool.acquire();
  CHECK(r.pool.live() == 3 && r.pool.exhaustions() == 0);
  r.pool.acquire();
  r.pool.acquire();
  CHECK(r.pool.exhaustions() == 0);
  // init forgets everything the pool held.
  r.pool.init(r.id, r.desc, r.reg, r.first, &r.src);
  CHECK(r.pool.live() == 0 && r.pool.bucketCount() == 1 &&
        r.pool.capacity() == 2 && r.pool.exhaustions() == 0);
  CHECK(r.pool.acquire() == a);

  // Two pools of one registry share a source; each names its own tokens.
  Registry reg;
  reg.budget(Budget::FirstGame, "Probe", Kind::Logic);
  PoolDesc d0 = {"zero", Budget::FirstGame, 16, 16, 1, 2, 2, Policy::Refuse};
  PoolDesc d1 = {"one", Budget::FirstGame, 32, 16, 1, 2, 2, Policy::Refuse};
  PoolId i0 = reg.pool(d0), i1 = reg.pool(d1);
  std::vector<unsigned char> s0(reg.poolFixedBytes(i0) + 64);
  std::vector<unsigned char> s1(reg.poolFixedBytes(i1) + 64);
  unsigned char* f0 = reinterpret_cast<unsigned char*>(
      (reinterpret_cast<uintptr_t>(s0.data()) + 15) / 16 * 16);
  unsigned char* f1 = reinterpret_cast<unsigned char*>(
      (reinterpret_cast<uintptr_t>(s1.data()) + 15) / 16 * 16);
  FakeSource src;
  PoolCore p0, p1;
  p0.init(i0, d0, reg, f0, &src);
  p1.init(i1, d1, reg, f1, &src);
  p0.acquire();
  p1.acquire();
  p0.acquire();
  CHECK(src.lastOwner == bucketToken(i0, 1) &&
        src.lastBytes == reg.bucketBytes(i0, 2));
  p1.acquire();
  CHECK(src.lastOwner == bucketToken(i1, 1) &&
        src.lastBytes == reg.bucketBytes(i1, 2));
  CHECK(std::strcmp(p0.name(), "zero") == 0 &&
        std::strcmp(p1.name(), "one") == 0);
  CHECK(p0.live() == 2 && p1.live() == 2);
}

// ---- a seeded random run against a model -----------------------------------

struct Slot {
  uint16_t bucket;
  uint32_t index;
  unsigned char pattern;
#if TYRA_MEMORY_CHECKS
  Id id;
#endif
};

struct Rng {
  uint32_t s;
  uint32_t next() {
    s = s * 1664525u + 1013904223u;
    return s >> 8;
  }
};

void testStress() {
  const PoolDesc desc = {"stress", Budget::FirstGame, 24, 8, 5, 7,
                         6,        Policy::Refuse};
  Rig r(desc);
  Rng rng = {12345};
  std::vector<Slot> live;
  unsigned char pattern = 1;
  uint32_t refused = 0, moves = 0, gaps = 0;
#if TYRA_MEMORY_CHECKS
  std::vector<Id> stale;
#endif

  auto ptr = [&](const Slot& s) { return r.pool.entryAt(s.bucket, s.index); };

  for (int step = 0; step < 30000; ++step) {
    // Phases: fill the pool up into its extensions, then drain it, so buckets
    // empty out and go back.
    const bool filling = (step / 1500) % 2 == 0;
    const uint32_t op = rng.next() % 100;
    if (op < (filling ? 66u : 22u)) {
      void* p = r.pool.acquire();
      if (p == nullptr) {
        ++refused;
        CHECK(r.pool.live() == r.pool.capacity() || r.src.refuse);
      } else {
        Slot s;
        s.pattern = pattern++;
        CHECK(where(r.pool, p, s.bucket, s.index));
        for (const Slot& o : live) CHECK(ptr(o) != p);
        fill(p, desc.typeSize, s.pattern);
#if TYRA_MEMORY_CHECKS
        s.id = r.pool.idOf(p);
        CHECK(r.pool.valid(s.id, p));
#endif
        live.push_back(s);
      }
    } else if (op < 90) {
      if (!live.empty()) {
        const size_t k = rng.next() % live.size();
        void* p = ptr(live[k]);
        CHECK(allBytes(p, desc.typeSize, live[k].pattern));
#if TYRA_MEMORY_CHECKS
        CHECK(r.pool.valid(live[k].id, p));
        stale.push_back(live[k].id);
#endif
        r.pool.release(p);
        live[k] = live.back();
        live.pop_back();
      }
    } else if (op < 94) {
      r.pool.releaseEmptyExtensions();
      for (uint32_t b = 1; b < r.pool.bucketCount(); ++b) {
        if (r.pool.countIn(static_cast<uint16_t>(b)) == 0) ++gaps;
      }
    } else if (op < 99) {
      // Move a held extension: copy, poison and free the old one, tell the
      // pool.
      std::vector<uint16_t> held;
      for (uint32_t b = 1; b < r.pool.bucketCount(); ++b) {
        if (r.pool.countIn(static_cast<uint16_t>(b)) != 0) {
          held.push_back(static_cast<uint16_t>(b));
        }
      }
      if (!held.empty()) {
        const uint16_t b = held[rng.next() % held.size()];
        void* oldBase = r.pool.entryAt(b, 0);
        void* newBase = r.src.rawAlloc(r.extBytes(), 16);
        std::memcpy(newBase, oldBase, r.extBytes());
        r.pool.bucketMoved(b, newBase);
        std::memset(oldBase, 0xDD, r.extBytes());
        r.src.rawFree(oldBase);
        ++moves;
      }
    } else {
      r.src.refuse = !r.src.refuse;
    }

    CHECK(r.pool.live() == live.size());
    if (step % 97 == 0) {
      CHECK(consistent(r.pool));
      for (const Slot& s : live) {
        CHECK(r.pool.isLive(s.bucket, s.index));
        CHECK(allBytes(ptr(s), desc.typeSize, s.pattern));
#if TYRA_MEMORY_CHECKS
        CHECK(r.pool.valid(s.id, ptr(s)));
#endif
      }
#if TYRA_MEMORY_CHECKS
      for (size_t i = 0; i < stale.size(); i += 7) {
        const Id& id = stale[i];
        CHECK(!r.pool.valid(id, r.pool.entryAt(id.bucket, id.index)));
      }
#endif
    }
  }
  // The run really exercised what it is for.
  CHECK(refused > 100 && moves > 100 && gaps > 0);
  CHECK(r.src.calls > 50 && r.src.released > 10);

  // Everything back, nothing lost.
  for (const Slot& s : live) r.pool.release(ptr(s));
  CHECK(r.pool.live() == 0);
  r.src.refuse = false;
  std::vector<void*> all;
  for (uint32_t i = 0; i < r.pool.capacity(); ++i)
    all.push_back(r.pool.acquire());
  CHECK(distinct(all.data(), all.size()));
  CHECK(r.pool.live() == r.pool.capacity());
}

// ---- init: what is refused in every build ----------------------------------

void testInitErrors() {
  installThrowingFailure();
  {
    // A pool the registry never issued.
    Registry reg;
    reg.budget(Budget::FirstGame, "Probe");
    PoolDesc d = {"ghost", Budget::FirstGame, 16, 16, 1, 0, 0, Policy::Refuse};
    alignas(16) unsigned char buf[64];
    PoolCore p;
    CHECK(throwsFailure([&] { p.init(3, d, reg, buf, nullptr); }, "registry"));
  }
  {
    // Past the 64 buckets a pool can number.
    Registry reg;
    reg.budget(Budget::FirstGame, "Probe");
    PoolDesc d = {"huge", Budget::FirstGame, 16, 16, 1, 1, 64, Policy::Refuse};
    PoolId id = reg.pool(d);
    std::vector<unsigned char> buf(reg.poolFixedBytes(id) + 32);
    unsigned char* b = reinterpret_cast<unsigned char*>(
        (reinterpret_cast<uintptr_t>(buf.data()) + 15) / 16 * 16);
    PoolCore p;
    CHECK(throwsFailure([&] { p.init(id, d, reg, b, nullptr); },
                        "too many extensions"));
    d.maxExtensions = 63;
    PoolId id2 = reg.pool(d);
    std::vector<unsigned char> buf2(reg.poolFixedBytes(id2) + 32);
    unsigned char* b2 = reinterpret_cast<unsigned char*>(
        (reinterpret_cast<uintptr_t>(buf2.data()) + 15) / 16 * 16);
    FakeSource src;
    PoolCore q;
    q.init(id2, d, reg, b2, &src);  // 1 + 63 buckets: the most there is
    std::vector<void*> v;
    for (int i = 0; i < 64; ++i) v.push_back(q.acquire());
    CHECK(distinct(v.data(), v.size()) && q.bucketCount() == 64);
    CHECK(q.acquire() == nullptr && src.calls == 63);
    CHECK(src.lastOwner == bucketToken(id2, 63));
  }
  {
    // Entries need a first bucket.
    Registry reg;
    reg.budget(Budget::FirstGame, "Probe");
    PoolDesc d = {"nobucket", Budget::FirstGame, 16, 16, 4, 0,
                  0,          Policy::Refuse};
    PoolId id = reg.pool(d);
    PoolCore p;
    CHECK(throwsFailure([&] { p.init(id, d, reg, nullptr, nullptr); },
                        "first bucket"));
  }
}

// ---- what only the checks see ----------------------------------------------

#if TYRA_MEMORY_CHECKS

void testChecks() {
  installThrowingFailure();

  Rig r({"chk", Budget::FirstGame, 24, 8, 3, 3, 2, Policy::Refuse});
  void* a = r.pool.acquire();
  void* b = r.pool.acquire();
  void* c = r.pool.acquire();
  void* d = r.pool.acquire();  // bucket 1
  CHECK(d != nullptr && r.pool.live() == 4);

  // ids
  Id ia = r.pool.idOf(a), id = r.pool.idOf(d);
  CHECK(ia.pool == r.id && ia.bucket == 0 && ia.index == 0);
  CHECK(id.pool == r.id && id.bucket == 1 && id.index == 0);
  CHECK(r.pool.idOf(b).index == 1 && r.pool.idOf(c).index == 2);
  CHECK(r.pool.valid(ia, a) && r.pool.valid(id, d));
  // wrong raw, null raw, another entry's raw
  CHECK(!r.pool.valid(ia, b) && !r.pool.valid(ia, nullptr) &&
        !r.pool.valid(id, a));
  // each field of an id
  Id bad = ia;
  bad.pool = static_cast<uint16_t>(ia.pool + 1);
  CHECK(!r.pool.valid(bad, a));
  bad = ia;
  bad.bucket = 5;
  CHECK(!r.pool.valid(bad, a));
  bad.bucket = 0xFFFF;
  CHECK(!r.pool.valid(bad, a));
  bad = ia;
  bad.index = 3;  // past the first bucket's 3 entries
  CHECK(!r.pool.valid(bad, a));
  bad.index = 0xFFFF;
  CHECK(!r.pool.valid(bad, a));
  bad = ia;
  bad.generation = static_cast<uint16_t>(ia.generation + 1);
  CHECK(!r.pool.valid(bad, a));
  bad = ia;
  bad.index = 1;  // the right slot of someone else's entry
  CHECK(!r.pool.valid(bad, a));
  Id none = {kNoPool, 0, 0, 0};
  CHECK(!r.pool.valid(none, a));

  // A released entry is not valid, and neither is the id after reuse; every
  // release is a new generation.
  uint16_t gen = ia.generation;
  for (int i = 0; i < 5; ++i) {
    Id before = r.pool.idOf(a);
    r.pool.release(a);
    CHECK(!r.pool.valid(before, a));
    void* again = r.pool.acquire();
    CHECK(again == a);
    CHECK(!r.pool.valid(before, a) && r.pool.valid(r.pool.idOf(a), a));
    CHECK(r.pool.idOf(a).generation == static_cast<uint16_t>(gen + i + 1));
  }

  // An id of an entry in a moved bucket is still good at its new address.
  Id moved = r.pool.idOf(d);
  void* oldBase = r.pool.entryAt(1, 0);
  void* newBase = r.src.rawAlloc(r.extBytes(), 16);
  std::memcpy(newBase, oldBase, r.extBytes());
  r.pool.bucketMoved(1, newBase);
  r.src.rawFree(oldBase);
  CHECK(!r.pool.valid(moved, oldBase) && r.pool.valid(moved, newBase));

  // An id of a bucket that was given back stays dead when its number is used
  // again by a new bucket.
  Rig h({"gen", Budget::FirstGame, 1, 4, 1, 2, 2, Policy::Refuse});
  std::vector<void*> hv;
  for (int i = 0; i < 5; ++i) hv.push_back(h.pool.acquire());
  std::vector<Id> hold;
  for (void* p : hv) hold.push_back(h.pool.idOf(p));
  for (int round = 0; round < 3; ++round) {
    for (size_t i = 1; i < hv.size(); ++i) h.pool.release(hv[i]);
    // the second extension (bucket 2) goes, bucket 1 is the spare
    h.pool.releaseEmptyExtensions();
    CHECK(h.src.released == round + 1);
    CHECK(h.pool.entryAt(2, 0) == nullptr);
    CHECK(!h.pool.valid(hold[3], hv[3]));
    // A new bucket 2: its entries do not accept ids of the old one.
    for (size_t i = 1; i < hv.size(); ++i) hv[i] = h.pool.acquire();
    CHECK(h.pool.bucketCount() == 3);
    for (size_t i = 3; i < hv.size(); ++i) {
      CHECK(!h.pool.valid(hold[i], hv[i]));
    }
    for (size_t i = 0; i < hv.size(); ++i) hold[i] = h.pool.idOf(hv[i]);
  }

  // idOf of what is not an entry of this pool.
  alignas(16) unsigned char stranger[128];
  CHECK(throwsFailure([&] { r.pool.idOf(stranger); }, "foreign"));
  CHECK(throwsFailure([&] { r.pool.idOf(nullptr); }, "foreign"));
  CHECK(throwsFailure(
      [&] {
        r.pool.idOf(static_cast<unsigned char*>(r.pool.entryAt(0, 1)) + 1);
      },
      "foreign"));
  {
    const std::string& line = capturedFailureLines().back();
    CHECK(line.find("[MEM] FAIL pool id of a foreign pointer name=chk ") == 0);
  }

  // release of what is not a live entry of this pool
  const uint32_t live = r.pool.live();
  CHECK(throwsFailure([&] { r.pool.release(stranger); }, "outside the pool"));
  CHECK(throwsFailure([&] { r.pool.release(nullptr); }, "outside the pool"));
  CHECK(throwsFailure(
      [&] {
        r.pool.release(static_cast<unsigned char*>(r.pool.entryAt(0, 1)) + 4);
      },
      "misplaced"));
  void* freed = r.pool.entryAt(0, 2);
  r.pool.release(freed);
  CHECK(throwsFailure([&] { r.pool.release(freed); }, "released twice"));
  CHECK(r.pool.live() == live - 1 && consistent(r.pool));
  CHECK(r.pool.acquire() == freed);  // the double release did not corrupt it
  CHECK(r.pool.acquire() != freed);

  // a write over a released entry is found when the pool reads its link
  Rig w({"wild", Budget::FirstGame, 24, 8, 3, 0, 0, Policy::Refuse});
  void* w0 = w.pool.acquire();
  void* w1 = w.pool.acquire();
  void* w2 = w.pool.acquire();
  w.pool.release(w0);
  w.pool.release(w1);         // the list is w1 -> w0
  std::memset(w1, 0x7F, 24);  // use after release
  CHECK(throwsFailure([&] { w.pool.acquire(); }, "free list"));
  CHECK(w.pool.live() == 1 && w2 != nullptr);
  w.pool.init(w.id, w.desc, w.reg, w.first, &w.src);
  CHECK(w.pool.acquire() != nullptr);

  // bucketMoved: the first bucket does not move, a bucket that is not there
  // cannot, and the new place must be aligned
  CHECK(throwsFailure([&] { r.pool.bucketMoved(0, newBase); }, "moved"));
  CHECK(throwsFailure([&] { r.pool.bucketMoved(5, newBase); }, "moved"));
  CHECK(throwsFailure([&] { r.pool.bucketMoved(0xFFFF, newBase); }, "moved"));
  CHECK(throwsFailure(
      [&] { r.pool.bucketMoved(1, static_cast<unsigned char*>(newBase) + 4); },
      "moved"));
  CHECK(throwsFailure([&] { r.pool.bucketMoved(1, nullptr); }, "moved"));
  CHECK(r.pool.entryAt(1, 0) == newBase);

  // init
  {
    Registry reg;
    reg.budget(Budget::FirstGame, "Probe");
    PoolDesc pd = {"p", Budget::FirstGame, 24, 8, 4, 4, 2, Policy::Refuse};
    PoolId pid = reg.pool(pd);
    std::vector<unsigned char> buf(reg.poolFixedBytes(pid) + 64);
    unsigned char* aligned = reinterpret_cast<unsigned char*>(
        (reinterpret_cast<uintptr_t>(buf.data()) + 15) / 16 * 16);
    PoolCore p;
    // a first bucket that is not aligned
    CHECK(throwsFailure([&] { p.init(pid, pd, reg, aligned + 4, nullptr); },
                        "misaligned"));
    // a row that is not the registry's
    PoolDesc other = pd;
    other.typeSize = 40;
    CHECK(throwsFailure([&] { p.init(pid, other, reg, aligned, nullptr); },
                        "registry"));
    other = pd;
    other.policy = Policy::RecycleOldest;
    CHECK(throwsFailure([&] { p.init(pid, other, reg, aligned, nullptr); },
                        "registry"));
    other = pd;
    other.firstCount = 5;
    CHECK(throwsFailure([&] { p.init(pid, other, reg, aligned, nullptr); },
                        "registry"));
    p.init(pid, pd, reg, aligned, nullptr);
    CHECK(p.acquire() != nullptr);

    // an index is 16 bits
    PoolDesc big = {"big", Budget::FirstGame, 4, 4, 70000, 0,
                    0,     Policy::Refuse};
    PoolId bid = reg.pool(big);
    std::vector<unsigned char> bbuf(reg.poolFixedBytes(bid) + 64);
    unsigned char* baligned = reinterpret_cast<unsigned char*>(
        (reinterpret_cast<uintptr_t>(bbuf.data()) + 15) / 16 * 16);
    PoolCore q;
    CHECK(throwsFailure([&] { q.init(bid, big, reg, baligned, nullptr); },
                        "ids"));
  }

  // a source that hands back a bucket that is not aligned
  {
    struct BadSource : BucketSource {
      std::vector<unsigned char> buf = std::vector<unsigned char>(4096);
      void* allocateBucket(size_t, size_t, bool, uint32_t, BudgetId) override {
        return buf.data() + 4 +
               (16 - reinterpret_cast<uintptr_t>(buf.data()) % 16) % 16;
      }
      void releaseBucket(void*) override {}
    };
    BadSource bad;
    Registry reg;
    reg.budget(Budget::FirstGame, "Probe");
    PoolDesc pd = {"p", Budget::FirstGame, 24, 8, 1, 4, 2, Policy::Refuse};
    PoolId pid = reg.pool(pd);
    std::vector<unsigned char> buf(reg.poolFixedBytes(pid) + 64);
    unsigned char* aligned = reinterpret_cast<unsigned char*>(
        (reinterpret_cast<uintptr_t>(buf.data()) + 15) / 16 * 16);
    PoolCore p;
    p.init(pid, pd, reg, aligned, &bad);
    p.acquire();
    CHECK(throwsFailure([&] { p.acquire(); }, "misaligned"));
    CHECK(p.bucketCount() == 1 && p.live() == 1);
  }
}

#else

// Without the checks nothing fails: a bad call is ignored where the pool can
// tell, and the pool is as it was.
void testNoChecks() {
  installThrowingFailure();
  Rig r({"chk", Budget::FirstGame, 24, 8, 3, 3, 2, Policy::Refuse});
  void* a = r.pool.acquire();
  void* b = r.pool.acquire();
  void* c = r.pool.acquire();
  (void)b;
  (void)c;
  alignas(16) unsigned char stranger[128];
  r.pool.release(stranger);
  r.pool.release(nullptr);
  CHECK(r.pool.live() == 3);
  r.pool.release(a);
  r.pool.release(a);  // twice
  CHECK(r.pool.live() == 2 && consistent(r.pool));
  CHECK(r.pool.acquire() == a);
  void* x = r.pool.acquire();
  CHECK(x != a && r.pool.live() == 4 && consistent(r.pool));
  r.pool.bucketMoved(0, stranger);  // the first bucket cannot move
  r.pool.bucketMoved(9, stranger);  // there is no such bucket
  CHECK(r.pool.entryAt(0, 0) == r.first);
  CHECK(r.pool.entryAt(1, 0) != static_cast<void*>(stranger));
  CHECK(capturedFailureLines().empty());
}

#endif

}  // namespace

int main() {
  installThrowingFailure();
  testBrief();
  testTokens();
  testStrideAndAlign();
  testHalt();
  testRefuse();
  testRecycleOldest();
  testReleaseEmptyExtensions();
  testAccessors();
  testReinitAndPools();
  testStress();
  testInitErrors();
#if TYRA_MEMORY_CHECKS
  testChecks();
#else
  testNoChecks();
#endif
  return finish("pool");
}
