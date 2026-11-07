// Host verifier for the render slots (renderer/3d/mesh/dynamic/skel_render_slots.hpp,
// docs/render-slots.md): the slot table's logic, the carve arithmetic of the layout, and the
// placement of a stream in a slot's memory. It runs the engine's own code.
//
//  (a) the hold: a body that leaves at frame f and is back at f + 29 keeps its slot, one back at
//      f + 31 has lost it;
//  (b) the fall back: a full tier 0 gives tier 1, then tier 2, and the counts say so; a body that
//      holds a slot is never left without one;
//  (c) the cheapest tier sized for the bodies never refuses: 100 000 random frames of 40 bodies
//      in two views, tiers of 3 and 6 slots above it; and sized for the most bodies the hold
//      keeps at once (measured on the same frames), it still never refuses;
//  (d) the two views of a split frame: a body only view 2 sees acquires in pass 2 and is skinned
//      there; a body both see is skinned once; a body one view sees near and the other far keeps
//      its near tier instead of swapping slots twice a frame;
//  (e) a slot whose memory a frame still reads is not given to a new body;
//  (f) the carve arithmetic: floor(tierBytes / slotBytes) slots per tier, slotBytes two sides of
//      the largest model, fewer slots when the allocator refuses, every side 16-byte aligned;
//  (g) a stream placed in a slot allocates no heap, fits placedBytes exactly, refuses less, and
//      its release gives nothing back.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only slots does this).

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <random>
#include <vector>

#include "renderer/3d/mesh/dynamic/skel_render_slots.hpp"

namespace {

using namespace Tyra;

int checks = 0;
int failures = 0;

#define CHECK(cond, ...)                                  \
  do {                                                    \
    ++checks;                                             \
    if (!(cond)) {                                        \
      ++failures;                                         \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__);    \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

// Heap bytes taken through operator new[] since the last mark: the placement checks.
size_t gHeapBytes = 0;
size_t gHeapCalls = 0;

}  // namespace

void* operator new[](size_t n, const std::nothrow_t&) noexcept {
  gHeapBytes += n;
  ++gHeapCalls;
  return std::malloc(n);
}
void* operator new[](size_t n) {
  gHeapBytes += n;
  ++gHeapCalls;
  return std::malloc(n);
}
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete[](void* p, size_t) noexcept { std::free(p); }

namespace {

SkelSlotTable makeTable(u32 t0, u32 t1, u32 t2) {
  SkelSlotTable t;
  const u32 per[kMaxSlotTiers] = {t0, t1, t2, 0};
  t.configure(per, 3);
  return t;
}

void testHold() {
  SkelSlotTable t = makeTable(2, 2, 2);
  bool fresh = false;
  // Leaves at f, back at f + 29: the same slot, not fresh.
  const u32 f = 100;
  const s32 a = t.acquire(7, 1, f, &fresh);
  CHECK(a >= 0 && fresh, "first acquire gives a fresh slot");
  for (u32 g = f; g < f + 29; ++g) t.frameEnd(g);
  const s32 b = t.acquire(7, 1, f + 29, &fresh);
  CHECK(b == a && !fresh, "back at f + 29 keeps its slot (%d vs %d, fresh %d)", (int)b, (int)a,
        (int)fresh);
  // Leaves at f, back at f + 31: gone.
  SkelSlotTable u = makeTable(2, 2, 2);
  const s32 c = u.acquire(9, 1, f, &fresh);
  for (u32 g = f; g <= f + 30; ++g) u.frameEnd(g);
  CHECK(u.slotOf(9) == -1, "a body unseen for 30 frames has lost its slot");
  const s32 d = u.acquire(9, 1, f + 31, &fresh);
  CHECK(d >= 0 && fresh, "back at f + 31 it takes a slot that is fresh (%d, was %d)", (int)d,
        (int)c);
  // Drawn every frame: kept for good.
  SkelSlotTable v = makeTable(1, 1, 1);
  const s32 e = v.acquire(3, 0, 0, &fresh);
  bool same = true;
  for (u32 g = 1; g < 500; ++g) {
    same = same && v.acquire(3, 0, g, &fresh) == e && !fresh;
    v.frameEnd(g);
  }
  CHECK(same, "a body drawn every frame keeps its slot");
  // A follower's touch keeps the owner's slot.
  SkelSlotTable w = makeTable(1, 1, 1);
  w.acquire(4, 0, 0);
  for (u32 g = 1; g < 200; ++g) {
    w.touch(4, g);
    w.frameEnd(g);
  }
  CHECK(w.slotOf(4) >= 0, "touch keeps a slot");
}

void testFallBack() {
  SkelSlotTable t = makeTable(2, 1, 1);
  const s32 a = t.acquire(1, 0, 0), b = t.acquire(2, 0, 0);
  const s32 c = t.acquire(3, 0, 0);  // tier 0 full -> tier 1
  const s32 d = t.acquire(4, 0, 0);  // tier 1 full -> tier 2
  const s32 e = t.acquire(5, 0, 0);  // all full
  CHECK(a >= 0 && t.tierOf(a) == 0 && b >= 0 && t.tierOf(b) == 0, "two LOD0 slots");
  CHECK(c >= 0 && t.tierOf(c) == 1, "a full tier 0 gives tier 1 (got tier %d)", (int)t.tierOf(c));
  CHECK(d >= 0 && t.tierOf(d) == 2, "then tier 2 (got tier %d)", (int)t.tierOf(d));
  CHECK(e == -1, "past the cheapest tier -1");
  const SkelSlotTable::Stats s = t.stats();
  CHECK(s.fellBack == 2 && s.refused == 1, "fellBack %u refused %u", s.fellBack, s.refused);
  CHECK(s.inUse[0] == 2 && s.inUse[1] == 1 && s.inUse[2] == 1, "inUse per tier");
  CHECK(s.peak[0] == 2 && s.peak[1] == 1 && s.peak[2] == 1, "peak per tier");
  // The refused body is asked again later: still -1, and a body holding a slot never loses it.
  CHECK(t.acquire(3, 0, 1) == c, "a body on a cheaper slot keeps it while no better tier is free");
  // When a LOD0 slot frees up, the body on a cheaper slot takes it, fresh.
  for (u32 g = 1; g <= 40; ++g) {
    t.touch(3, g);
    t.touch(4, g);
    t.frameEnd(g);
  }
  CHECK(t.slotOf(1) == -1 && t.slotOf(2) == -1, "the two LOD0 bodies timed out");
  bool fresh = false;
  const s32 up = t.acquire(3, 0, 41, &fresh);
  CHECK(up >= 0 && t.tierOf(up) == 0 && fresh, "a body on tier 1 moves up when tier 0 is free");
  CHECK(t.stats().inUse[1] == 0, "and its tier 1 slot is free");
}

struct Lcg {
  std::mt19937 rng;
  explicit Lcg(u32 seed) : rng(seed) {}
  u32 below(u32 n) { return static_cast<u32>(rng() % n); }
};

// 40 bodies in two views over `frames` frames: a body is seen by view 1, view 2, both or none for
// stretches of random length, at a tier its distance picks. Returns the refused count and the
// most distinct bodies seen within a hold window.
struct RunResult {
  u32 refused = 0, windowMax = 0, fellBack = 0;
  u32 peak[kMaxSlotTiers] = {0, 0, 0, 0};
};

RunResult runBodies(u32 t0, u32 t1, u32 t2, u32 frames, u32 seed) {
  constexpr u32 kBodies = 40;
  SkelSlotTable t = makeTable(t0, t1, t2);
  Lcg r(seed);
  struct Body {
    u8 view = 0;      // bit 0: view 1 sees it, bit 1: view 2
    u8 tier = 0;
    u32 untilFrame = 0;
    u32 lastSeen = 0xFFFFFFFFu;
  } b[kBodies];
  RunResult out;
  for (u32 f = 0; f < frames; ++f) {
    for (u32 i = 0; i < kBodies; ++i) {
      if (f >= b[i].untilFrame) {
        static const u8 kViews[6] = {0, 0, 0, 1, 2, 3};  // none, 1, 2, both
        b[i].view = kViews[r.below(6)];
        b[i].tier = static_cast<u8>(r.below(3));
        b[i].untilFrame = f + 1 + r.below(400);
      }
    }
    // Pass 1, then pass 2: each acquires for the bodies its view sees.
    for (u32 pass = 0; pass < 2; ++pass)
      for (u32 i = 0; i < kBodies; ++i)
        if (b[i].view & (1u << pass)) {
          t.acquire(i, b[i].tier, f);
          b[i].lastSeen = f;
        }
    t.frameEnd(f);
    u32 window = 0;
    for (u32 i = 0; i < kBodies; ++i)
      if (b[i].lastSeen != 0xFFFFFFFFu && f - b[i].lastSeen < kSlotHoldFrames) ++window;
    if (window > out.windowMax) out.windowMax = window;
  }
  out.refused = t.stats().refused;
  out.fellBack = t.stats().fellBack;
  for (u32 k = 0; k < kMaxSlotTiers; ++k) out.peak[k] = t.stats().peak[k];
  return out;
}

void testNeverRefuses() {
  const u32 frames = 100000;
  RunResult a = runBodies(3, 6, 40, frames, 1234);
  CHECK(a.refused == 0, "cheapest tier sized for the 40 bodies refused %u", a.refused);
  CHECK(a.fellBack > 0, "the small tiers did fall back (%u)", a.fellBack);
  std::printf("  100000 frames, 40 bodies, tiers 3/6/40: refused %u, fell back %u, window max %u, "
              "peaks %u/%u/%u\n",
              a.refused, a.fellBack, a.windowMax, a.peak[0], a.peak[1], a.peak[2]);
  // Sized for the most bodies the hold keeps at once instead.
  RunResult b = runBodies(3, 6, a.windowMax, frames, 1234);
  CHECK(b.refused == 0, "cheapest tier sized for the hold window (%u) refused %u", a.windowMax,
        b.refused);
  // One slot fewer than the window can refuse: the check is not vacuous.
  RunResult c = runBodies(0, 0, a.windowMax > 8 ? 8 : 1, frames, 1234);
  CHECK(c.refused > 0, "an undersized cheapest tier refuses (%u)", c.refused);
}

// The template's rule, modelled on the table: a body is skinned in the pass that gives it a fresh
// slot, or when its pose is due and the slot was not written this frame.
void testViews() {
  SkelSlotTable t = makeTable(4, 4, 8);
  std::vector<u32> skinnedFrame(16, 0xFFFFFFFFu);
  std::vector<u32> skinsThisRun(16, 0);
  auto pass = [&](u32 frame, u32 body, u8 tier, bool poseDue) {
    bool fresh = false;
    const s32 s = t.acquire(body, tier, frame, &fresh);
    if (s < 0) return false;
    const bool written = skinnedFrame[s] != 0xFFFFFFFFu && !fresh;
    const bool skin = !written || (skinnedFrame[s] != frame && poseDue);
    if (skin) {
      skinnedFrame[s] = frame;
      ++skinsThisRun[body];
    }
    return skin;
  };
  // Body 1: only view 2 sees it. Pass 1 does nothing; pass 2 acquires and skins it.
  const bool skinned = pass(10, 1, 0, true);
  CHECK(skinned && t.slotOf(1) >= 0, "a body only view 2 sees is skinned in pass 2");
  // Body 2: both views, every frame, pose due: skinned in pass 1 only.
  u32 doubles = 0;
  for (u32 f = 20; f < 60; ++f) {
    const bool p1 = pass(f, 2, 0, true);
    const bool p2 = pass(f, 2, 0, true);
    if (p1 && p2) ++doubles;
    CHECK(p1 && !p2, "frame %u: view 1 skins (%d), view 2 reuses (%d)", f, (int)p1, (int)p2);
    t.frameEnd(f);
  }
  CHECK(doubles == 0, "never skinned twice a frame");
  // Body 3: near in view 2 (tier 0), far in view 1 (tier 2): one slot, assigned once, and it
  // stays the near tier.
  u32 freshCount = 0;
  s32 first = -1;
  for (u32 f = 100; f < 220; ++f) {
    bool fr1 = false, fr2 = false;
    const s32 a = t.acquire(3, 2, f, &fr1);
    const s32 b = t.acquire(3, 0, f, &fr2);
    freshCount += (fr1 ? 1u : 0u) + (fr2 ? 1u : 0u);
    if (first < 0) first = b;
    CHECK(a >= 0 && b >= 0, "split-screen body has a slot");
    if (f > 100)
      CHECK(a == b && b == first, "frame %u: one slot throughout (%d %d %d)", f, (int)a, (int)b,
            (int)first);
    t.frameEnd(f);
  }
  CHECK(freshCount == 2 || freshCount == 1,
        "a body seen near by one view and far by the other takes a slot once or twice, not each "
        "frame (%u)",
        freshCount);
  CHECK(t.tierOf(first) == 0, "and it is the near tier");
}

struct NotReady {
  bool ready[8];
};
bool readyFn(u32 slot, void* ctx) { return static_cast<NotReady*>(ctx)->ready[slot]; }

void testReady() {
  SkelSlotTable t = makeTable(2, 0, 2);
  NotReady nr;
  for (bool& r : nr.ready) r = true;
  bool fresh = false;
  const s32 a = t.acquire(1, 0, 0, &fresh, readyFn, &nr);
  t.release(1);
  nr.ready[a] = false;  // a frame still reads the slot body 1 gave back
  const s32 b = t.acquire(2, 0, 1, &fresh, readyFn, &nr);
  CHECK(b >= 0 && b != a, "a slot a frame still reads is not given away (%d vs %d)", (int)b, (int)a);
  const s32 c = t.acquire(3, 0, 1, &fresh, readyFn, &nr);
  CHECK(c >= 0 && t.tierOf(c) == 2, "the other tier-0 slot is taken, then tier 2 serves");
  nr.ready[a] = true;
  const s32 d = t.acquire(4, 0, 2, &fresh, readyFn, &nr);
  CHECK(d == a, "once the frame is done the slot is free for the next body");
}

// A model's sizes for the carve: bytes of one side per tier.
u32 sideBytesOf(const SkelModel& m, u8 tier, u32) {
  static const u32 sizes[2][3] = {{1000, 500, 250}, {1500, 300, 120}};
  const uintptr_t id = reinterpret_cast<uintptr_t>(&m) & 1u;
  return sizes[id][tier];
}

size_t gAllowedAllocs = 0;
std::vector<u8*> gTestRaws;
u8* limitedAlloc(u32 bytes) {
  if (gAllowedAllocs == 0) return nullptr;
  --gAllowedAllocs;
  u8* p = new u8[bytes];
  gTestRaws.push_back(p);
  return p;
}

void testCarve() {
  alignas(2) static char models[2];
  const SkelModel* list[2] = {reinterpret_cast<const SkelModel*>(&models[0]),
                              reinterpret_cast<const SkelModel*>(&models[1])};
  // Make the second model's id odd, the first's even: (address & 1) differs.
  const SkelModel* ms[2] = {list[0], list[1]};
  const u32 tierBytes[kMaxSlotTiers] = {10000, 6000, 3000, 0};
  gAllowedAllocs = 1000;
  SkelRenderSlots slots;
  slots.layout(ms, 2, tierBytes, 3, 48, sideBytesOf, limitedAlloc);
  // The largest model per tier: 1500 / 500 / 250 -> a slot is 2 sides: 3000 / 1008 / 512 after
  // aligning a side to 16 (1504, 512, 256).
  const u32 side[3] = {1504, 512, 256};
  for (u32 t = 0; t < 3; ++t) {
    const u32 want = tierBytes[t] / (2 * side[t]);
    CHECK(slots.slotsCarved(static_cast<u8>(t)) == want, "tier %u: %u slots, floor(%u / %u) = %u",
          t, slots.slotsCarved(static_cast<u8>(t)), tierBytes[t], 2 * side[t], want);
    CHECK(slots.slotBytes(static_cast<u8>(t)) == 2 * side[t], "tier %u slot bytes", t);
  }
  CHECK(slots.slotTable().slotCount() ==
            slots.slotsCarved(0) + slots.slotsCarved(1) + slots.slotsCarved(2),
        "the table has every carved slot");
  CHECK(slots.reservedBytes() == slots.slotsCarved(0) * 2 * side[0] +
                                     slots.slotsCarved(1) * 2 * side[1] +
                                     slots.slotsCarved(2) * 2 * side[2],
        "reserved bytes");
  bool aligned = true;
  {
    // holds(): a stream of a slot a body holds and has written; not one of a slot gone back.
    SkelSlot* s = slots.acquire(50, 0, 0);
    CHECK(s != nullptr && !slots.holds(s->current(0)), "a slot nothing is written in holds nothing");
    s->skinnedFrame = 0;
    CHECK(slots.holds(s->current(0)) && slots.holds(s->spare(1)), "a written slot of a body holds");
    slots.release(50);
    CHECK(!slots.holds(s->current(0)), "a slot given back holds nothing");
    StaPipCompactStream alone;
    CHECK(!slots.holds(&alone), "a stream of no slot is not held");
  }
  for (u32 body = 0; body < 3; ++body) {
    SkelSlot* s = slots.acquire(body, static_cast<u8>(body), 0);
    CHECK(s != nullptr, "a slot of tier %u", body);
    if (s == nullptr) continue;
    aligned = aligned && (reinterpret_cast<uintptr_t>(s->memory[0]) & 15) == 0 &&
              (reinterpret_cast<uintptr_t>(s->memory[1]) & 15) == 0 &&
              s->memory[1] == s->memory[0] + s->sideBytes;
    CHECK(s->tier == body, "slot tier");
  }
  CHECK(aligned, "every side 16-byte aligned, the second following the first");
  slots.reset();
  CHECK(!slots.active() && slots.reservedBytes() == 0, "reset frees every slot");

  // A refused allocation ends a tier's carve there.
  gAllowedAllocs = 2;
  slots.layout(ms, 2, tierBytes, 3, 48, sideBytesOf, limitedAlloc);
  CHECK(slots.slotsCarved(0) == 2 && slots.slotsCarved(1) == 0 && slots.slotsCarved(2) == 0,
        "two allocations: tier 0 gets 2 (%u %u %u)", slots.slotsCarved(0), slots.slotsCarved(1),
        slots.slotsCarved(2));
  CHECK(slots.slotTable().slotsInTier(0) == 2 && slots.slotTable().slotsInTier(2) == 0,
        "the table matches the carve");
  // Bytes under one slot give none.
  slots.reset();
  const u32 small[kMaxSlotTiers] = {2999, 0, 0, 0};
  gAllowedAllocs = 10;
  slots.layout(ms, 2, small, 1, 48, sideBytesOf, limitedAlloc);
  CHECK(slots.slotsCarved(0) == 0 && !slots.active(), "2999 bytes carve no 3008-byte slot");
  slots.reset();
}

void testPlacement() {
  StaPipCompactLayout lay = StaPipCompactLayout::forClass(true, false, true, false, 48, false);
  const float lo[3] = {-1.0f, -1.0f, -1.0f}, hi[3] = {1.0f, 1.0f, 1.0f};
  const Compact::Box box = Compact::modelBox(lo, hi);
  const u32 n = 1000;
  const u32 bytes = StaPipCompactStream::placedBytes(lay, n);
  CHECK(bytes > 0 && (bytes & 15) == 0, "placedBytes %u is a multiple of 16", bytes);
  // Heap-allocated stream, to compare its size with the placed one.
  StaPipCompactStream plain;
  CHECK(plain.allocate(lay, n, box), "a plain stream allocates");
  CHECK(plain.qwords() * 16 < bytes, "the block is under the placed bytes (the boxes follow)");
  plain.release();

  alignas(16) static u8 memory[1 << 16];
  std::memset(memory, 0xAB, sizeof(memory));
  gHeapBytes = gHeapCalls = 0;
  StaPipCompactStream placed;
  CHECK(placed.allocateIn(lay, n, box, memory, bytes), "a stream is placed in exactly placedBytes");
  CHECK(gHeapCalls == 0, "placing a stream allocates nothing (%zu calls)", gHeapCalls);
  CHECK(placed.isPlaced() && placed.allocated(), "placed and allocated");
  CHECK(static_cast<const void*>(placed.data()) == static_cast<const void*>(memory),
        "its block is the memory");
  CHECK(placed.vertexCount() == n, "its vertex count");
  // The block was zeroed, the run headers written, the boxes inside the memory.
  const u8* partBoxes = reinterpret_cast<const u8*>(placed.partBoxes());
  CHECK(partBoxes >= memory && partBoxes + placed.partCount() * 24 <= memory + bytes,
        "the boxes lie inside the placed bytes");
  StaPipCompactStream tooSmall;
  CHECK(!tooSmall.allocateIn(lay, n, box, memory, bytes - 16), "16 bytes less is refused");
  CHECK(!tooSmall.allocated(), "and nothing is kept");
  CHECK(!tooSmall.allocateIn(lay, n, box, memory + 8, bytes), "unaligned memory is refused");
  // Writing through the stream stays inside the memory.
  const u8 guard = memory[bytes];
  std::memset(placed.run(0, 0), 0x55, 16);
  CHECK(memory[bytes] == guard, "the memory past placedBytes is untouched");
  gHeapBytes = gHeapCalls = 0;
  placed.release();
  CHECK(!placed.allocated() && !placed.isPlaced(), "released");
  CHECK(gHeapCalls == 0, "release frees nothing it did not allocate");
  // Another body: the same memory is placed again, zeroed.
  CHECK(placed.allocateIn(lay, n, box, memory, bytes), "placed again");
  CHECK(placed.run(0, 0)[0] == 0, "and zeroed again");
  // A different layout, a different size.
  StaPipCompactLayout lay75 = StaPipCompactLayout::forClass(true, false, true, false, 75, false);
  CHECK(StaPipCompactStream::placedBytes(lay75, n) != bytes, "the package size changes the bytes");
  CHECK(StaPipCompactStream::placedBytes(lay, 0) == 0, "no vertices, no bytes");
  placed.release();

  // Indexed boxes in a slot's memory: the pose buffer is the caller's too.
  const u32 ibytes = StaPipCompactStream::indexedPlacedBytes(lay75, n);
  CHECK(ibytes > 0 && (ibytes & 15) == 0 && ibytes < bytes, "indexed boxes are %u bytes", ibytes);
}

}  // namespace

int main() {
  testHold();
  testFallBack();
  testNeverRefuses();
  testViews();
  testReady();
  testCarve();
  testPlacement();
  std::printf("%d checks, %d failures\n", checks, failures);
  if (failures != 0) {
    std::printf("FAIL verify-skel-render-slots\n");
    return 1;
  }
  std::printf("PASS verify-skel-render-slots\n");
  return 0;
}
