// Host verifier for the skinned far tier: the game template's two tier functions, extracted from
// src/game_templates.inc (never copied) into actual-tier.inc by tools/verify-tyrax2-host.py, and
// the render slots' handling of tier 3.
//
//  (a) skelTierFor: the tier boundaries at meshLod, 2 x meshLod and farLod; tier 3 never without
//      a level 3 (levels <= 3) or with farLod <= 0; tier 3 with meshLod off; a farLod nearer than
//      2 x meshLod skips tier 2; the tier never falls as the distance grows;
//  (b) skelRefreshDue: a far (tier 3) body refreshes exactly once in any 8 consecutive ticks,
//      bodies 0..7 on 8 different ticks, whatever the distance and the animation LOD; a body
//      entering a view refreshes at once, in any tier; tiers 0..2 keep the pose cadence's rule
//      (skel_cadence.hpp) tick for tick, including the held pose of a body seen for 4 frames;
//  (c) the render slots: a tier-3 body takes a tier-3 slot (the cheapest), a full tier 3 refuses
//      only past the cheapest tier, a tier-2 body falls back to tier 3 when tier 2 is full, and
//      with only three tiers configured a tier-3 body takes the cheapest configured slot.
//
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel-tier does this).

#include <cstdio>

#include "renderer/3d/mesh/dynamic/skel_cadence.hpp"
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

// The template's functions, as they are in the game template.
#include "actual-tier.inc"

}  // namespace

int main() {
  // ---- (a) skelTierFor: meshLod 10 m (m2 = 100), 2 x = 20 m (400), farLod 40 m (1600), 4 levels.
  const float mesh = 10.0F, far = 40.0F;
  CHECK(skelTierFor(0.0F, mesh, far, 4) == 0, "at the camera: tier 0");
  CHECK(skelTierFor(100.0F, mesh, far, 4) == 0, "at meshLod: still tier 0");
  CHECK(skelTierFor(100.5F, mesh, far, 4) == 1, "past meshLod: tier 1");
  CHECK(skelTierFor(400.0F, mesh, far, 4) == 1, "at 2 x meshLod: still tier 1");
  CHECK(skelTierFor(400.5F, mesh, far, 4) == 2, "past 2 x meshLod: tier 2");
  CHECK(skelTierFor(1600.0F, mesh, far, 4) == 2, "at farLod: still tier 2");
  CHECK(skelTierFor(1600.5F, mesh, far, 4) == 3, "past farLod: tier 3");
  CHECK(skelTierFor(1.0e8F, mesh, far, 4) == 3, "far away: tier 3");
  CHECK(skelTierFor(1.0e8F, mesh, far, 5) == 3, "five levels: tier 3 is still the far tier");

  bool neverWithout = true;
  for (float d2 = 0.0F; d2 < 1.0e7F; d2 += 37.0F) {
    if (skelTierFor(d2, mesh, far, 3) == 3 || skelTierFor(d2, mesh, far, 1) == 3 ||
        skelTierFor(d2, mesh, far, 0) == 3 || skelTierFor(d2, mesh, 0.0F, 4) == 3 ||
        skelTierFor(d2, mesh, -1.0F, 4) == 3)
      neverWithout = false;
  }
  CHECK(neverWithout, "tier 3 never without a level 3 (levels <= 3) or with farLod <= 0");
  CHECK(skelTierFor(1.0e8F, mesh, far, 3) == 2, "three levels: the farthest body is tier 2");
  CHECK(skelTierFor(1.0e8F, mesh, 0.0F, 4) == 2, "farLod 0: the farthest body is tier 2");
  CHECK(skelTierFor(1.0e8F, 0.0F, 0.0F, 4) == 0, "both off: tier 0 everywhere");
  CHECK(skelTierFor(10.0F, 0.0F, far, 4) == 0, "meshLod off, near: tier 0");
  CHECK(skelTierFor(1600.5F, 0.0F, far, 4) == 3, "meshLod off, past farLod: tier 3");
  CHECK(skelTierFor(300.0F, mesh, 15.0F, 4) == 3 && skelTierFor(200.0F, mesh, 15.0F, 4) == 1,
        "a farLod nearer than 2 x meshLod skips tier 2");

  bool monotone = true;
  for (int k = 0; k < 8; ++k) {
    const float meshes[4] = {0.0F, 6.0F, 10.0F, 25.0F};
    const float fars[2] = {0.0F, 70.0F};
    const float m = meshes[k & 3], f = fars[k >> 2 & 1];
    u8 last = 0;
    for (float d = 0.0F; d < 400.0F; d += 0.25F) {
      const u8 t = skelTierFor(d * d, m, f, 4);
      if (t < last) monotone = false;
      last = t;
    }
  }
  CHECK(monotone, "the tier never falls as the distance grows");

  // ---- (b) skelRefreshDue.
  bool onceIn8 = true, distinct = true;
  const float dists[4] = {0.0F, 70.0F, 1.0e4F, 1.0e9F};
  const float lods[3] = {0.0F, 8.0F, 30.0F};
  for (float d2 : dists)
    for (float al : lods)
      for (u32 body = 0; body < 40; ++body)
        for (u32 start = 0; start < 24; ++start) {
          u32 n = 0;
          for (u32 t = start; t < start + 8; ++t)
            if (skelRefreshDue(t, body, 3, d2, al, false)) ++n;
          if (n != 1) onceIn8 = false;
        }
  CHECK(onceIn8, "a far body refreshes exactly once in any 8 consecutive ticks");
  for (u32 base = 0; base < 40; base += 8)
    for (u32 start = 0; start < 16; ++start) {
      bool taken[8] = {false, false, false, false, false, false, false, false};
      for (u32 k = 0; k < 8; ++k)
        for (u32 t = start; t < start + 8; ++t)
          if (skelRefreshDue(t, base + k, 3, 1.0e4F, 8.0F, false)) {
            if (taken[t - start]) distinct = false;
            taken[t - start] = true;
          }
      for (bool b : taken)
        if (!b) distinct = false;
    }
  CHECK(distinct, "bodies 0..7 (any 8 in a row) refresh on 8 different ticks");
  CHECK(skelRefreshDue(3, 5, 3, 1.0e4F, 8.0F, false) && !skelRefreshDue(4, 5, 3, 1.0e4F, 8.0F, false),
        "tick + body = 8 is the turn of a far body");

  bool enteredAlways = true;
  for (u8 tier = 0; tier < 4; ++tier)
    for (float d2 : dists)
      for (float al : lods)
        for (u32 t = 0; t < 16; ++t)
          if (!skelRefreshDue(t, 3, tier, d2, al, true)) enteredAlways = false;
  CHECK(enteredAlways, "a body entering a view refreshes at once, in every tier");

  // Tiers 0..2: the pose cadence's rule, tick for tick (a held body seen last frame).
  bool sameAsCadence = true;
  for (u8 tier = 0; tier < 3; ++tier)
    for (float d2 : dists)
      for (float al : lods)
        for (u32 body = 0; body < 9; ++body)
          for (u32 t = 5; t < 5 + 16; ++t) {
            bool expect = true;
            if (al > 0.0F) {
              const float lod2 = al * al;
              expect = SkelSkin::Cadence::repostThisFrame(
                  t, body, SkelSkin::Cadence::periodFor(d2, lod2), true, t - 1, 4, false);
            }
            if (skelRefreshDue(t, body, tier, d2, al, false) != expect) sameAsCadence = false;
          }
  CHECK(sameAsCadence, "tiers 0..2 refresh as the pose cadence says");
  CHECK(skelRefreshDue(7, 1, 1, 1.0e4F, 0.0F, false), "animation LOD off: a near-tier body refreshes every frame");

  // ---- (c) the render slots: a tier-3 body takes a tier-3 slot.
  {
    SkelSlotTable table;
    const u32 per[kMaxSlotTiers] = {1, 1, 1, 2};
    table.configure(per, 4);
    const u8 tierFar = skelTierFor(1.0e6F, mesh, far, 4);
    CHECK(tierFar == 3, "the far body asks for tier 3");
    const s32 a = table.acquire(10, tierFar, 100);
    CHECK(a >= 0 && table.tierOf(a) == 3, "a tier-3 body takes a tier-3 slot");
    const s32 b = table.acquire(11, tierFar, 100);
    CHECK(b >= 0 && table.tierOf(b) == 3 && b != a, "a second far body takes the other tier-3 slot");
    CHECK(table.stats().refused == 0 && table.stats().fellBack == 0, "no refusal, no fall back yet");
    const s32 c = table.acquire(12, tierFar, 100);
    CHECK(c < 0 && table.stats().refused == 1, "a full tier 3 (the cheapest) refuses");
    // a tier-2 body when tier 2 is full falls back to tier 3 when it has a slot
    SkelSlotTable fall;
    const u32 per2[kMaxSlotTiers] = {0, 0, 1, 1};
    fall.configure(per2, 4);
    const s32 x = fall.acquire(1, 2, 10);
    const s32 y = fall.acquire(2, 2, 10);
    CHECK(x >= 0 && fall.tierOf(x) == 2 && y >= 0 && fall.tierOf(y) == 3 && fall.stats().fellBack == 1,
          "a tier-2 body falls back to tier 3 when tier 2 is full");
    CHECK(fall.stats().inUse[3] == 1 && fall.stats().peak[3] == 1, "the stats count tier 3");
    // three tiers configured (a project with no tier-3 bytes): the far body takes the cheapest
    SkelSlotTable three;
    const u32 per3[kMaxSlotTiers] = {1, 1, 2, 0};
    three.configure(per3, 3);
    const s32 z = three.acquire(1, 3, 10);
    CHECK(z >= 0 && three.tierOf(z) == 2, "with three tiers a tier-3 body takes the cheapest slot");
    // a body keeps its slot as it walks out to tier 3 and back
    const s32 held = table.acquire(10, 3, 101);
    CHECK(held == a, "a far body keeps its tier-3 slot");
  }

  std::printf("verify-skel-tier: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
