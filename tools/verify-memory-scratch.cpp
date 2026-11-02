// Added by TyraX: host verifier for the per-frame scratch
// (inc/memory/scratch.hpp, src/memory/scratch.cpp): the bump region, its two
// instances, the std allocators and the containers on them. Built and run by
// tools/verify-memory.py, once with TYRA_MEMORY_CHECKS=0 and once with =1.

#include <cstdint>
#include <cstring>
#include <new>
#include <string>
#include <type_traits>
#include <vector>

#include "memory/scratch.hpp"
#include "verify-memory-check.hpp"

using namespace Tyra;

namespace {

constexpr size_t kFrameBytes = 64 * 1024;
constexpr size_t kLoadBytes = 16 * 1024;

alignas(64) unsigned char g_frameBuf[kFrameBytes];
alignas(64) unsigned char g_loadBuf[kLoadBytes];

// A fresh, empty frame and load scratch, over the static buffers.
void freshRegions() {
  Scratch::frame().init(g_frameBuf, sizeof(g_frameBuf));
  Scratch::load().init(g_loadBuf, sizeof(g_loadBuf));
}

bool inFrame(const void* p) {
  const unsigned char* b = static_cast<const unsigned char*>(p);
  return b >= g_frameBuf && b < g_frameBuf + sizeof(g_frameBuf);
}

bool inLoad(const void* p) {
  const unsigned char* b = static_cast<const unsigned char*>(p);
  return b >= g_loadBuf && b < g_loadBuf + sizeof(g_loadBuf);
}

// The failure f() ends in, if it does.
template <class F>
bool failureOf(F f, Failure& out) {
  try {
    f();
  } catch (const ThrownFailure& t) {
    out = t.f;
    return true;
  }
  return false;
}

// A container kept past its frame cannot be destroyed under the throwing halt:
// its destructor gives the block back, the allocator reports the stale epoch,
// and a throw out of a destructor is std::terminate. (In the game the halt
// never returns.) So these tests build it in raw storage and never destroy it;
// the bytes it owns are in the static region, so nothing leaks.
template <class C>
struct Immortal {
  alignas(C) unsigned char bytes[sizeof(C)];
  C& make() { return *new (bytes) C(); }
};

#if TYRA_MEMORY_CHECKS
void ignoreLine(const char*) {}
void returningHalt() {}
#endif

struct alignas(64) Wide {
  unsigned char c[64];
};

// ---- the brief's checks ----------------------------------------------------

void testBriefChecks() {
  freshRegions();
  auto& s = Scratch::frame();
  void* a = s.allocate(100, 16);
  void* b = s.allocate(100, 64);
  CHECK(a != nullptr && b != nullptr);
  CHECK(uintptr_t(b) % 64 == 0 && (char*)b >= (char*)a + 100);
  s.deallocate(b, 100);
  void* c = s.allocate(100, 64);
  CHECK(c == b);  // last block rolled back
  memset(a, 0x33, 100);
  size_t used = s.used();
  s.reset();
  CHECK(s.used() == 0 && s.peak() >= used);
  CHECK(allBytes(a, 100, 0x33));  // the user's rule: nothing erased
  {
    Scratch::vector<int> v;
    for (int i = 0; i < 1000; ++i) v.push_back(i);
    CHECK(v[999] == 999 && s.used() >= 4000);
  }
  {
    Scratch::string t = "abc";
    t += std::string(200, 'x');
    Scratch::map<int, int> m{{1, 2}};
    Scratch::unordered_map<int, int> u{{3, 4}};
    Scratch::stringstream ss;
    ss << 42;
    CHECK(ss.str() == "42");
    CHECK(t.size() == 203 && m.at(1) == 2 && u.at(3) == 4);
  }
  s.reset();
  CHECK(
      throwsFailure([] { Scratch::vector<char> big(70000); }, "scratch full"));
#if TYRA_MEMORY_CHECKS
  s.reset();
  {
    Immortal<Scratch::vector<int>> slot;
    Scratch::vector<int>& kept = slot.make();
    kept.reserve(4);
    s.reset();
    CHECK(throwsFailure([&] { kept.reserve(64); },
                        "scratch kept past its frame"));
  }
#endif
}

// ---- the region ------------------------------------------------------------

void testRegion() {
  alignas(64) static unsigned char buf[1024];

  // A region nobody initialised has no room.
  {
    Scratch::Region r;
    CHECK(r.capacity() == 0 && r.used() == 0 && r.peak() == 0);
    CHECK(r.allocate(1, 1) == nullptr);
    CHECK(r.allocate(0, 1) == nullptr);
    r.deallocate(nullptr, 1);  // nothing to give back; must be harmless
    CHECK(r.used() == 0);
  }

  // init: cursor, peak and capacity; the epoch moves, the bytes do not.
  Scratch::Region r;
  memset(buf, 0x5A, sizeof(buf));
  const uint32_t e0 = r.epoch();
  r.init(buf, sizeof(buf));
  CHECK(r.capacity() == 1024 && r.used() == 0 && r.peak() == 0);
  CHECK(r.epoch() != e0);
  CHECK(allBytes(buf, sizeof(buf), 0x5A));

  // alignment is of the address, padding counts as used
  void* p1 = r.allocate(1, 1);
  CHECK(p1 == buf && r.used() == 1);
  void* p8 = r.allocate(8, 8);
  CHECK(uintptr_t(p8) % 8 == 0 && p8 == buf + 8 && r.used() == 16);
  void* p64 = r.allocate(10, 64);
  CHECK(p64 == buf + 64 && r.used() == 74);
  CHECK(r.peak() == 74);

  // rolling back the last block takes its padding with it
  r.deallocate(p64, 10);
  CHECK(r.used() == 16);
  CHECK(r.peak() == 74);  // peak stays
  // a block that is not the last stays used...
  r.deallocate(p8, 8);
  CHECK(r.used() == 16);
  void* q = r.allocate(4, 4);
  CHECK(q == buf + 16);
  // ...and so does the last one when the size is wrong
  r.deallocate(q, 8);
  CHECK(r.used() == 20);
  r.deallocate(q, 4);
  CHECK(r.used() == 16);
  // only ONE block is remembered: giving back the one before it later is a
  // no-op
  r.deallocate(q, 4);
  CHECK(r.used() == 16);
  void* q2 = r.allocate(4, 4);
  CHECK(q2 == q);

  // a range that does not start aligned: the address is aligned, not the offset
  Scratch::Region odd;
  odd.init(buf + 3, 200);
  void* o1 = odd.allocate(1, 1);
  CHECK(o1 == buf + 3);
  void* o64 = odd.allocate(5, 64);
  CHECK(uintptr_t(o64) % 64 == 0 && o64 >= static_cast<void*>(buf + 4));
  CHECK(uintptr_t(o64) - uintptr_t(buf + 3) + 5 == odd.used());
  void* o16 = odd.allocate(7, 16);
  CHECK(uintptr_t(o16) % 16 == 0 && uintptr_t(o16) >= uintptr_t(o64) + 5);
  // an alignment whose next address lies past the end is a full region, not a
  // wrap
  const size_t oddUsed = odd.used();
  void* far = odd.allocate(1, 4096);
  CHECK(far == nullptr ? odd.used() == oddUsed : uintptr_t(far) % 4096 == 0);

  // exactly full, then one byte more
  Scratch::Region t;
  t.init(buf, 256);
  CHECK(t.allocate(256, 1) == buf && t.used() == 256 && t.peak() == 256);
  CHECK(t.allocate(1, 1) == nullptr);
  CHECK(t.used() == 256);  // a failed allocation changes nothing
  t.reset();
  CHECK(t.allocate(255, 1) == buf);
  CHECK(t.allocate(2, 1) == nullptr && t.used() == 255);
  CHECK(t.allocate(1, 1) == buf + 255 && t.used() == 256);

  // sizes that would wrap around the range are refused, not served
  t.reset();
  CHECK(t.allocate(static_cast<size_t>(-1), 1) == nullptr);
  CHECK(t.allocate(static_cast<size_t>(-1) - 8, 16) == nullptr);
  CHECK(t.allocate(256 + 1, 1) == nullptr && t.used() == 0);
  CHECK(t.allocate(1, static_cast<size_t>(1) << (sizeof(size_t) * 8 - 1)) ==
        nullptr);
  CHECK(t.used() == 0);
  // ...also when a padding already in the way would make the sum wrap to zero
  CHECK(t.allocate(1, 1) == buf);
  CHECK(t.allocate(static_cast<size_t>(-1), 2) == nullptr);
  CHECK(t.allocate(static_cast<size_t>(-1) - 1, 4) == nullptr);
  CHECK(t.used() == 1);

  // zero bytes are served as one: distinct pointers, and they give back
  t.reset();
  CHECK(t.used() == 0);
  void* z1 = t.allocate(0, 1);
  void* z2 = t.allocate(0, 1);
  CHECK(z1 != nullptr && z2 != nullptr && z1 != z2 && t.used() == 2);
  t.deallocate(z2, 0);
  CHECK(t.used() == 1);

  // a bad alignment is refused (with checks it is also reported)
#if TYRA_MEMORY_CHECKS
  installThrowingFailure();
  CHECK(throwsFailure([&] { t.allocate(8, 3); }, "power of two"));
  CHECK(t.used() == 1);
#else
  CHECK(t.allocate(8, 3) == nullptr);
#endif

  // reset: cursor 0, peak kept, epoch up, nothing erased
  memset(buf, 0x77, 64);
  t.reset();
  const uint32_t e1 = t.epoch();
  t.allocate(64, 1);
  t.reset();
  CHECK(t.epoch() == e1 + 1 && t.used() == 0 && t.peak() == 256);
  void* again = t.allocate(64, 1);
  CHECK(again == buf && allBytes(again, 64, 0x77));  // reused as it was

  // a block from before a reset is nobody's: giving it back changes nothing
  t.reset();
  t.allocate(10, 1);
  void* lastOne = t.allocate(20, 1);  // the last block, with a cursor before it
  t.reset();
  t.deallocate(lastOne, 20);
  CHECK(t.used() == 0);

  // init starts over: peak 0, a new epoch, the old block is nobody's
  t.allocate(10, 1);
  void* lastTwo = t.allocate(20, 1);
  const uint32_t e2 = t.epoch();
  t.init(buf, 128);
  CHECK(t.capacity() == 128 && t.used() == 0 && t.peak() == 0 &&
        t.epoch() != e2);
  t.deallocate(lastTwo, 20);
  CHECK(t.used() == 0);
  t.deallocate(again, 64);
  CHECK(t.used() == 0);
  // a null base has no room, whatever the size said
  t.init(nullptr, 128);
  CHECK(t.capacity() == 0 && t.allocate(1, 1) == nullptr);
}

// ---- the two instances -----------------------------------------------------

void testInstances() {
  freshRegions();
  CHECK(&Scratch::frame() != &Scratch::load());
  CHECK(&Scratch::frame() == &Scratch::frame());
  CHECK(Scratch::frame().capacity() == kFrameBytes);
  CHECK(Scratch::load().capacity() == kLoadBytes);
  void* a = Scratch::frame().allocate(10, 1);
  CHECK(inFrame(a) && Scratch::frame().used() == 10 &&
        Scratch::load().used() == 0);
  void* b = Scratch::load().allocate(10, 1);
  CHECK(inLoad(b) && Scratch::load().used() == 10 &&
        Scratch::frame().used() == 10);
}

// ---- the containers --------------------------------------------------------

void testContainers() {
  freshRegions();
  auto& s = Scratch::frame();

  // vector: data in the frame region, correct after many reallocations
  {
    Scratch::vector<int> v;
    for (int i = 0; i < 1500; ++i) v.push_back(i * 3);
    bool ok = true;
    for (int i = 0; i < 1500; ++i) ok = ok && v[i] == i * 3;
    CHECK(ok && inFrame(v.data()) && inFrame(v.data() + 1499));
    Scratch::vector<int> copy = v;  // same epoch: copy is on the scratch too
    CHECK(copy == v && inFrame(copy.data()) && copy.data() != v.data());
    Scratch::vector<int> moved = std::move(copy);
    CHECK(moved == v && copy.empty());
    moved.swap(v);
    CHECK(v.size() == 1500);
  }

  // a container that never allocates costs nothing
  s.reset();
  {
    Scratch::vector<int> e;
    Scratch::string es;
    Scratch::map<int, int> em;
    CHECK(e.empty() && es.empty() && em.empty() && s.used() == 0);
  }

  // over-aligned types are aligned (a byte in front makes sure it takes
  // padding)
  s.reset();
  s.allocate(1, 1);
  {
    Scratch::vector<Wide> w;
    w.resize(3);
    CHECK(uintptr_t(w.data()) % 64 == 0 && inFrame(w.data()) &&
          s.used() >= 64 + 192);
    Scratch::vector<double> d(5, 1.5);
    CHECK(uintptr_t(d.data()) % alignof(double) == 0 && d[4] == 1.5);
    Scratch::vector<uint16_t> h(3, 7);
    CHECK(uintptr_t(h.data()) % alignof(uint16_t) == 0 && h[2] == 7);
  }

  // string: short stays inline, long goes to the region
  s.reset();
  {
    Scratch::string small = "abc";
    CHECK(s.used() == 0);
    Scratch::string big = small;
    big += std::string(500, 'q');
    CHECK(big.size() == 503 && inFrame(big.data()) && s.used() >= 503);
    CHECK(big.compare(0, 3, "abc") == 0 && big.back() == 'q');
  }

  // deque, map, unordered_map: nodes and chunks in the region
  s.reset();
  {
    Scratch::deque<int> dq;
    for (int i = 0; i < 1000; ++i) dq.push_back(i);
    for (int i = 0; i < 500; ++i) dq.pop_front();
    CHECK(dq.size() == 500 && dq.front() == 500 && dq.back() == 999);
    CHECK(inFrame(&dq.front()) && inFrame(&dq.back()));

    Scratch::map<int, int> m;
    for (int i = 0; i < 200; ++i) m[i * 7 % 200] = i;
    CHECK(m.size() == 200 && m.begin()->first == 0 && inFrame(&*m.begin()));
    Scratch::map<std::string, int, std::greater<std::string>> named;
    named["a"] = 1;
    named["b"] = 2;
    CHECK(named.begin()->first == "b" && inFrame(&*named.begin()));

    Scratch::unordered_map<int, int> u;
    for (int i = 0; i < 200; ++i) u[i] = i * 2;
    bool ok = true;
    for (int i = 0; i < 200; ++i) ok = ok && u.at(i) == i * 2;
    CHECK(ok && u.size() == 200 && inFrame(&*u.begin()));
    u.erase(5);
    CHECK(u.count(5) == 0 && u.size() == 199);
    Scratch::unordered_map<int, int, std::hash<int>, std::equal_to<int>> u2 = u;
    CHECK(u2.size() == 199);
  }

  // stringstream: the buffer is on the scratch and the text round-trips
  s.reset();
  {
    Scratch::stringstream ss;
    ss << std::string(500, 'z') << 12345 << ' ' << 1.5;
    const Scratch::string out = ss.str();
    CHECK(out.size() == 500 + 5 + 1 + 3 && inFrame(out.data()));
    CHECK(out.substr(500) == "12345 1.5");
    CHECK(s.used() >= 500);
    int n = 0;
    Scratch::stringstream in;
    in << "77 88";
    in >> n;
    CHECK(n == 77);
  }

  // freed memory is only reclaimed when it was the last block
  s.reset();
  {
    const size_t base = s.used();
    {
      Scratch::vector<int> last;
      last.reserve(100);
      CHECK(s.used() >= base + 400);
    }
    CHECK(s.used() == base);  // it was the last: gone
    Scratch::vector<int> first;
    first.reserve(100);
    Scratch::vector<int> second;
    second.reserve(100);
    const size_t both = s.used();
    first = Scratch::vector<int>();
    first.shrink_to_fit();  // frees the first block, which is not the last
    CHECK(s.used() == both);
    CHECK(second.capacity() >= 100);
  }

  // many frames: the same work needs the same room each time, nothing creeps
  s.reset();
  size_t firstPeak = 0;
  bool steady = true;
  for (int n = 0; n < 200; ++n) {
    s.reset();
    steady = steady && s.used() == 0;
    {
      Scratch::vector<int> v;
      Scratch::map<int, Scratch::string> m;
      for (int i = 0; i < 100; ++i) {
        v.push_back(i);
        m[i] = Scratch::string(40, char('a' + i % 26));
      }
      steady = steady && v.size() == 100 && m.size() == 100;
    }
    if (n == 0) firstPeak = s.peak();
    steady = steady && s.peak() == firstPeak;
  }
  CHECK(steady && firstPeak > 0 && firstPeak <= kFrameBytes);
}

void testLoad() {
  freshRegions();
  auto& f = Scratch::frame();
  auto& l = Scratch::load();
  {
    Scratch::Load::vector<int> v;
    for (int i = 0; i < 500; ++i) v.push_back(i);
    CHECK(v[499] == 499 && inLoad(v.data()) && l.used() >= 2000 &&
          f.used() == 0);
    Scratch::Load::string t = "abc";
    t += std::string(300, 'y');
    CHECK(t.size() == 303 && inLoad(t.data()));
  }

  // the load scratch keeps its contents across a frame reset...
  l.reset();
  {
    Scratch::Load::vector<int> keep;
    keep.reserve(8);
    const uint32_t le = l.epoch();
    for (int i = 0; i < 3; ++i) f.reset();
    keep.push_back(1);
    keep.reserve(64);  // ...and an allocator made before it still works
    CHECK(l.epoch() == le && keep[0] == 1 && inLoad(keep.data()));
  }

  // a full load scratch is "load", not "frame"
  l.reset();
  Failure got{};
  CHECK(
      failureOf([] { Scratch::Load::vector<char> big(kLoadBytes + 1); }, got));
  CHECK(std::strcmp(got.what, "scratch full") == 0 &&
        std::strcmp(got.name, "load") == 0);
  CHECK(got.size == kLoadBytes + 1 && got.caller != 0);
}

// ---- failures --------------------------------------------------------------

void testFailures() {
  freshRegions();
  installThrowingFailure();
  auto& s = Scratch::frame();

  Failure got{};
  CHECK(failureOf([] { Scratch::vector<char> big(70000); }, got));
  CHECK(std::strcmp(got.what, "scratch full") == 0);
  CHECK(std::strcmp(got.name, "frame") == 0);
  CHECK(got.size == 70000 && got.caller != 0);
  CHECK(!capturedFailureLines().empty());
  CHECK(capturedFailureLines().back().rfind(
            "[MEM] FAIL scratch full name=frame size=70000 caller=0x", 0) == 0);
  CHECK(s.used() == 0);  // the failed request took nothing

  // a count whose byte size would wrap around is a full scratch, not a small
  // block
  CHECK(failureOf(
      [] {
        Scratch::Allocator<int> a;
        a.allocate(static_cast<size_t>(-1) / sizeof(int) +
                   2);  // x 4 wraps to 4
      },
      got));
  CHECK(std::strcmp(got.what, "scratch full") == 0 && s.used() == 0);
  CHECK(std::strcmp(got.name, "frame") == 0 &&
        got.size == static_cast<size_t>(-1));

  // the scratch fills up through ordinary use, and the failure says how much
  // was asked for
  CHECK(failureOf(
      [] {
        Scratch::vector<char> a(40000);
        Scratch::vector<char> b(40000);
      },
      got));
  CHECK(std::strcmp(got.what, "scratch full") == 0 && got.size == 40000);
  CHECK(s.used() == 0);  // `a` was the last block, it rolled back as it unwound

  // after a reset the same request is served
  s.reset();
  {
    Scratch::vector<char> ok(60000);
  }
  CHECK(s.used() == 0);

#if TYRA_MEMORY_CHECKS
  // kept past its frame: the epoch the allocator was made in is not the
  // region's
  s.reset();
  {
    Immortal<Scratch::vector<int>> slot;
    Scratch::vector<int>& kept = slot.make();
    kept.reserve(4);
    CHECK(kept.capacity() == 4);
    s.reset();
    const size_t before = s.used();
    CHECK(failureOf([&] { kept.reserve(64); }, got));
    CHECK(std::strcmp(got.what, "scratch kept past its frame") == 0);
    CHECK(std::strcmp(got.name, "frame") == 0 && got.size == 64 * sizeof(int));
    CHECK(got.caller != 0 && s.used() == before);
    CHECK(capturedFailureLines().back().rfind(
              "[MEM] FAIL scratch kept past its frame name=frame size=256 "
              "caller=0x",
              0) == 0);
    CHECK(kept.capacity() == 4);  // the vector is as it was
  }
  {
    Immortal<Scratch::string> slot;
    Scratch::string& kept = slot.make();
    kept.append(100, 'k');
    s.reset();
    CHECK(throwsFailure([&] { kept.append(1000, 'k'); },
                        "scratch kept past its frame"));
  }

  // a halt that returns (a custom platform may): the failed call changes
  // nothing
  {
    setFailureOutput(&ignoreLine, &returningHalt);
    setFailureHandler(nullptr);
    Scratch::Allocator<int> al;
    int* p = al.allocate(4);
    s.reset();
    CHECK(al.allocate(4) == nullptr &&
          s.used() == 0);  // stale: reported, nothing served
    int* mine = static_cast<int*>(s.allocate(16, alignof(int)));  // where p was
    CHECK(mine == p && s.used() == 16);
    al.deallocate(p, 4);  // stale: must not roll back the new frame's block
    CHECK(s.used() == 16);
    installThrowingFailure();
  }

  // the allocator itself: allocate and deallocate both check
  {
    Scratch::Allocator<int> al;
    int* p = al.allocate(4);
    CHECK(inFrame(p));
    s.reset();
    CHECK(
        throwsFailure([&] { al.allocate(4); }, "scratch kept past its frame"));
    const size_t before = s.used();
    CHECK(failureOf([&] { al.deallocate(p, 4); }, got));
    CHECK(std::strcmp(got.what, "scratch kept past its frame") == 0 &&
          got.size == 16);
    CHECK(s.used() == before);      // and it did not touch the cursor
    Scratch::Allocator<int> fresh;  // made now: fine
    int* q = fresh.allocate(4);
    CHECK(inFrame(q));
    fresh.deallocate(q, 4);
  }

  // ...the same for the load scratch
  {
    auto& l = Scratch::load();
    Scratch::LoadAllocator<int> al;
    int* p = al.allocate(4);
    l.reset();
    CHECK(failureOf([&] { al.deallocate(p, 4); }, got));
    CHECK(std::strcmp(got.what, "scratch kept past its frame") == 0);
    CHECK(std::strcmp(got.name, "load") == 0);
  }

  // a frame reset does not stale a load container, nor the other way round
  {
    Scratch::Load::vector<int> lv;
    lv.reserve(4);
    Scratch::frame().reset();
    lv.reserve(32);
    CHECK(lv.capacity() >= 32);
  }
  {
    Scratch::vector<int> fv;
    fv.reserve(4);
    Scratch::load().reset();
    fv.reserve(32);
    CHECK(fv.capacity() >= 32);
  }
#endif
}

// ---- the alignment a container gets ----------------------------------------

bool aligned16(const void* p) {
  return reinterpret_cast<uintptr_t>(p) % 16 == 0;
}

// std::allocator hands out memory aligned to at least 16 (operator new is
// malloc's, and malloc aligns to 16 - on the EE above the default new
// alignment, 8), and swapping std::vector for Scratch::vector must keep that: a
// byte buffer that lands after an odd-sized block must not start on an odd
// address (a header struct cast over it would trap on the EE).
void testContainerAlignment() {
  struct Header {
    uint32_t magic;
    uint32_t size;
  };
  static_assert(Scratch::kMinContainerAlign == 16, "malloc's alignment");
  static_assert(Scratch::Allocator<char>::kAlign == 16, "byte containers");
  static_assert(Scratch::LoadAllocator<unsigned char>::kAlign == 16, "load");
  static_assert(Scratch::Allocator<Wide>::kAlign == 64, "more when asked");

  for (size_t pre = 1; pre <= 40; ++pre) {
    freshRegions();
    Scratch::frame().allocate(pre, 1);  // an odd or even block in front
    Scratch::load().allocate(pre, 1);

    Scratch::vector<unsigned char> bytes(32);
    CHECK(aligned16(bytes.data()) && inFrame(bytes.data()));
    Scratch::vector<uint8_t> more(1);
    CHECK(aligned16(more.data()));
    Scratch::string text(100, 'x');
    CHECK(aligned16(text.data()) && inFrame(text.data()));
    Scratch::vector<uint16_t> halves(5);
    CHECK(aligned16(halves.data()));
    Scratch::deque<char> dq(1000);
    CHECK(aligned16(&dq.front()));
    Scratch::map<char, char> m{{'a', 'b'}};
    CHECK(aligned16(&*m.begin()));
    Scratch::stringstream ss;
    ss << std::string(100, 'z');
    CHECK(aligned16(ss.str().data()));
    Scratch::vector<Wide> wide(1);
    CHECK(uintptr_t(wide.data()) % 64 == 0);
    Scratch::Allocator<char> a;
    char* c = a.allocate(1);
    CHECK(aligned16(c));
    a.deallocate(c, 1);

    // the file-buffer case: bytes read into a load container, then seen as a
    // struct
    Scratch::Load::vector<uint8_t> file(sizeof(Header) * 2);
    CHECK(aligned16(file.data()) && inLoad(file.data()));
    CHECK(uintptr_t(file.data()) % alignof(Header) == 0);
    Scratch::Load::string name(64, 'n');
    CHECK(aligned16(name.data()) && inLoad(name.data()));
  }

  // the Region itself still gives exactly the alignment it is asked for
  freshRegions();
  Scratch::frame().allocate(1, 1);
  void* one = Scratch::frame().allocate(1, 1);
  CHECK(uintptr_t(one) == uintptr_t(g_frameBuf) + 1);
}

// ---- the allocator as a type ----------------------------------------------

void testAllocatorType() {
  freshRegions();
  using FA = Scratch::Allocator<int>;
  using FAc = Scratch::Allocator<char>;
  using LA = Scratch::LoadAllocator<int>;

  static_assert(std::is_same<FA::value_type, int>::value, "value_type");
  static_assert(std::is_same<FA::rebind<char>::other, FAc>::value, "rebind");
  static_assert(
      std::is_same<std::allocator_traits<FA>::rebind_alloc<char>, FAc>::value,
      "allocator_traits rebind");
  static_assert(!std::is_same<FA, LA>::value,
                "frame and load are different types");
  static_assert(std::is_nothrow_copy_constructible<FA>::value, "copy");
  static_assert(std::is_nothrow_move_constructible<FA>::value, "move");
  static_assert(std::is_constructible<FAc, const FA&>::value,
                "converting constructor");

#if TYRA_MEMORY_CHECKS
  // stateful: it holds the epoch
  static_assert(!std::is_empty<FA>::value, "holds the epoch with checks");
  static_assert(!std::allocator_traits<FA>::is_always_equal::value,
                "not always equal");
  FA a;
  FAc ac(a);
  CHECK(a == a && a == ac && ac == a && !(a != ac));
  Scratch::frame().reset();
  FA b;
  CHECK(a != b && !(a == b) && b == b);
  FAc bc(b);
  CHECK(bc == b && bc != a);
  // a rebound copy keeps the epoch of the allocator it came from
  FA a2(ac);
  CHECK(a2 == a && a2 != b);
  // the loads do not compare with the frames - their own epoch
  LA l1;
  Scratch::frame().reset();
  LA l2;
  CHECK(l1 == l2);
  Scratch::load().reset();
  LA l3;
  CHECK(l1 != l3 && l2 != l3);
#else
  // stateless: an empty class, equal to every other, and no bigger a container
  static_assert(std::is_empty<FA>::value, "empty without checks");
  static_assert(std::is_empty<LA>::value, "empty without checks");
  static_assert(std::allocator_traits<FA>::is_always_equal::value,
                "always equal");
  static_assert(sizeof(Scratch::vector<int>) == sizeof(std::vector<int>),
                "no room for the allocator");
  static_assert(sizeof(Scratch::Load::vector<int>) == sizeof(std::vector<int>),
                "no room for the allocator");
  FA a;
  FAc ac(a);
  CHECK(a == a && a == ac && !(a != ac));
  Scratch::frame().reset();
  FA b;
  CHECK(a == b && !(a != b));
  // so a block made before a reset is not a failure, it is the caller's bug
  int* p = a.allocate(4);
  Scratch::frame().reset();
  a.deallocate(p, 4);
  CHECK(Scratch::frame().used() == 0);
#endif
}

}  // namespace

int main() {
  installThrowingFailure();
  testBriefChecks();
  testRegion();
  testInstances();
  testContainers();
  testLoad();
  testFailures();
  testContainerAlignment();
  testAllocatorType();
  return finish("scratch");
}
