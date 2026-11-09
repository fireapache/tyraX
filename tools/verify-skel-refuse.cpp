// Host verifier for the indexed skin packaging under a refusing budget
// (renderer/3d/mesh/dynamic/skel_indexed.hpp, skel_indexed_shared.hpp; docs/animated-models.md,
// "Memory that is refused").
//
// The Skinning budget refuses a nothrow request when it is full and HALTS on a throwing one
// (docs/memory-manager.md). This program replaces the global operator new with a byte-limited
// one that behaves the same way: a nothrow new over the limit answers null, a throwing new over
// the limit counts a halt (and then serves, so the run goes on to count them all). It then
// builds the shared packaging of
//   - a flat-shaded part (every corner its own vertex: the unique set is as large as the corner
//     list, the case an estimate of "a quarter of the corners" gets wrong), and
//   - a smooth part (about one corner in five is unique),
// at every limit from nothing to the build's peak, and checks that
//   - no throwing request is ever made over the limit (halts == 0);
//   - a build either succeeds with the same tables as the unlimited one or answers false with
//     refused() set and leaves nothing behind;
//   - the pose buffer pool grows nothrow too: acquire answers null when it cannot make a
//     buffer, and retire / reuse never allocate.
//
// Built and run by tools/verify-tyrax2-host.py --only skel-refuse.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <vector>

#include "renderer/3d/mesh/dynamic/skel_indexed.hpp"
#include "renderer/3d/mesh/dynamic/skel_indexed_shared.hpp"

namespace {

// ---- the byte-limited global allocator ---------------------------------------------------

struct Counters {
  size_t limit = ~static_cast<size_t>(0);  // live bytes the budget holds
  size_t live = 0, peak = 0;
  unsigned halts = 0;     // throwing requests over the limit
  unsigned refusals = 0;  // nothrow requests over the limit
  unsigned calls = 0;     // every request served or refused
  bool armed = false;     // the counters only run while a case is being measured
};
Counters g;

constexpr size_t kHeader = 16;  // the block's size, 16-byte aligned (the manager's header)
constexpr size_t kPerBlock = 16;  // what the budget charges besides the bytes (its own header)

void* take(size_t size, bool nothrow) {
  if (!g.armed) {
    void* raw = std::malloc(size + kHeader);
    if (raw == nullptr) std::abort();
    *static_cast<size_t*>(raw) = ~static_cast<size_t>(0);  // not counted
    return static_cast<char*>(raw) + kHeader;
  }
  ++g.calls;
  const size_t charge = size + kPerBlock;
  if (g.live + charge > g.limit) {
    if (nothrow) {
      ++g.refusals;
      return nullptr;
    }
    ++g.halts;  // the manager would stop the game here
  }
  void* raw = std::malloc(size + kHeader);
  if (raw == nullptr) std::abort();
  *static_cast<size_t*>(raw) = charge;
  g.live += charge;
  if (g.live > g.peak) g.peak = g.live;
  return static_cast<char*>(raw) + kHeader;
}

void give(void* p) {
  if (p == nullptr) return;
  char* raw = static_cast<char*>(p) - kHeader;
  const size_t charge = *reinterpret_cast<size_t*>(raw);
  if (charge != ~static_cast<size_t>(0)) g.live -= charge;
  std::free(raw);
}

}  // namespace

void* operator new(size_t n) { return take(n, false); }
void* operator new[](size_t n) { return take(n, false); }
void* operator new(size_t n, const std::nothrow_t&) noexcept { return take(n, true); }
void* operator new[](size_t n, const std::nothrow_t&) noexcept { return take(n, true); }
void operator delete(void* p) noexcept { give(p); }
void operator delete[](void* p) noexcept { give(p); }
void operator delete(void* p, size_t) noexcept { give(p); }
void operator delete[](void* p, size_t) noexcept { give(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { give(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { give(p); }

namespace {

using namespace Tyra;
namespace Ix = SkelSkin::Indexed;

int checks = 0, failures = 0;
void check(bool ok, const char* what) {
  ++checks;
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
  }
}

struct Part {
  std::vector<u32> source;  // skinSource: the first identical corner, itself when unique
  std::vector<s16> stq;     // [corners][3]
  StaPipCompactLayout layout;
  u32 corners = 0;
  u32 package = 0;
};

// `repeat` corners in `repeat`+1... a corner repeats an earlier one every `period`-th place; 0 =
// flat-shaded (every corner unique).
Part makePart(u32 triangles, u32 period, u32 package) {
  Part p;
  p.corners = triangles * 3;
  p.package = package;
  p.source.resize(p.corners);
  for (u32 v = 0; v < p.corners; ++v) {
    // a repeat points at a corner a few places back, as a shared vertex of a strip does
    p.source[v] = (period != 0 && v >= 8 && v % period != 0) ? v - 6 - (v % 3) : v;
    // a repeat of a repeat resolves to the first
    if (p.source[v] != v) p.source[v] = p.source[p.source[v]];
  }
  p.stq.assign((size_t)p.corners * 3, 0);
  p.layout = StaPipCompactLayout::forClass(true, false, true, false, package, false);
  return p;
}

struct Built {
  bool ok = false, refused = false;
  size_t bytes = 0;
  std::vector<u32> first, src;
};

// One build under a limit; the Shared is destroyed before the counters are read back.
Built buildUnder(const Part& part, size_t limit, unsigned* halts, size_t* peakOut = nullptr) {
  Built out;
  g = Counters();
  g.armed = true;
  g.limit = limit;
  {
    std::unique_ptr<Ix::Shared> shared(new (std::nothrow) Ix::Shared());
    if (!shared) {
      out.refused = true;  // not even the object
    } else {
      out.ok = shared->build(part.source.data(), part.corners, part.package, part.stq.data(),
                             part.layout);
      out.refused = shared->refused();
      g.armed = false;  // the copies below are the test's, not the budget's
      if (out.ok) {
        const Ix::Level& lv = shared->level();
        out.first = lv.first;
        out.src = lv.src;
      }
    }
  }
  g.armed = false;
  *halts = g.halts;
  out.bytes = g.live;  // what is left behind once the Shared is gone
  if (peakOut != nullptr) *peakOut = g.peak;
  return out;
}

void sweep(const char* name, const Part& part) {
  unsigned halts = 0;
  size_t peak = 0;
  const Built full = buildUnder(part, ~static_cast<size_t>(0), &halts, &peak);
  std::printf("%s: %u corners, packages of %u, build peak %zu bytes, %zu package-uniques\n", name,
              part.corners, part.package, peak, full.src.size());
  check(full.ok, "the unlimited build succeeds");
  check(full.bytes == 0, "the unlimited build leaves nothing behind");
  unsigned worstHalts = 0, refusedBuilds = 0, okBuilds = 0, mismatched = 0, leaked = 0,
           notRefused = 0;
  const size_t steps = 60;  // up to 1.5 x the build's peak: the requests are asked for in blocks
  for (size_t k = 0; k <= steps; ++k) {
    const size_t limit = peak * 3 * k / (2 * steps);
    const Built b = buildUnder(part, limit, &halts);
    if (halts > worstHalts) worstHalts = halts;
    if (b.bytes != 0) ++leaked;
    if (b.ok) {
      ++okBuilds;
      if (b.first != full.first || b.src != full.src) ++mismatched;
    } else {
      ++refusedBuilds;
      if (!b.refused) ++notRefused;
    }
  }
  std::printf("  %zu limits: %u built, %u refused, worst throwing requests over the limit %u\n",
              steps + 1, okBuilds, refusedBuilds, worstHalts);
  check(worstHalts == 0, "no throwing request is made over the budget (nothing halts)");
  check(okBuilds > 0 && refusedBuilds > 0, "the sweep covers both outcomes");
  check(mismatched == 0, "a build that succeeds under a limit has the unlimited build's tables");
  check(leaked == 0, "a refused build leaves nothing behind");
  check(notRefused == 0, "a build that fails for want of memory says so (refused())");
}

// The pose buffer pool: made nothrow, reused without allocating.
void pool() {
  const Part part = makePart(600, 5, 48);
  unsigned halts = 0;
  size_t peak = 0;
  buildUnder(part, ~static_cast<size_t>(0), &halts, &peak);
  g = Counters();
  g.armed = true;
  std::unique_ptr<Ix::Shared> shared(new Ix::Shared());
  const bool ok = shared->build(part.source.data(), part.corners, part.package, part.stq.data(),
                                part.layout);
  check(ok, "the pool test part builds");
  const size_t base = g.live;
  const u32 bufBytes = shared->poseBytes();
  // room for the packaging plus three pose buffers and their bookkeeping, not four
  g.limit = base + 3 * (bufBytes + 15 + 4 * kPerBlock) + 3 * 64;
  Ix::PoseBuf* held[8];  // (an array: the test's own bookkeeping makes no request)
  size_t heldCount = 0;
  for (int i = 0; i < 8; ++i) {
    Ix::PoseBuf* b = shared->acquire();
    if (b == nullptr) break;
    held[heldCount++] = b;
  }
  check(heldCount >= 2 && heldCount <= 4, "the pool stops at the limit");
  check(shared->acquire() == nullptr, "acquire answers null when the pool cannot grow");
  check(g.halts == 0, "growing the pool never makes a throwing request over the budget");
  // retire everything, then take it all back: no request is made
  g.calls = 0;
  for (size_t i = 0; i < heldCount; ++i) shared->retire(held[i]);
  check(g.calls == 0, "retire allocates nothing");
  for (size_t i = 0; i < heldCount; ++i) check(shared->acquire() != nullptr, "a retired buffer is reused");
  check(g.calls == 0, "reusing a retired buffer allocates nothing");
  check(g.halts == 0, "no halt over the whole pool run");
  shared.reset();
  g.armed = false;
  check(g.live == 0, "the pool leaves nothing behind");
}

}  // namespace

int main() {
  // 1200 triangles: 3600 corners; 48-corner packages (a flat 75-corner package does not fit)
  sweep("flat-shaded", makePart(1200, 0, 48));
  sweep("smooth", makePart(1200, 5, 48));
  sweep("smooth, 75-corner packages", makePart(1200, 5, 75));
  pool();
  std::printf("verify-skel-refuse: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
