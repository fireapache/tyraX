// Host verifier for the pose cadence (renderer/3d/mesh/dynamic/skel_cadence.hpp).
//
// Checks:
//  - the period by distance band: 1 within the animation-LOD distance, 2 beyond it, 4 beyond
//    twice it, and 1 when the distance is off (lodDist2 <= 0);
//  - the stagger: over `period` consecutive frames every body index is due exactly once, and
//    on any frame the due bodies are at most ceil(bodies / period);
//  - the held pose is never older than period - 1 frames (every body is due at least once in
//    any window of `period` frames);
//  - a body with no held pose, a body not updated for more than maxGap frames, and a body whose
//    mesh LOD changed are re-posed at once, whatever its turn.
// Build and run from the TyraX root (tools/verify-tyrax2-host.py --only skel does this).

#include <cstdio>

#include "renderer/3d/mesh/dynamic/skel_cadence.hpp"

namespace {

using namespace Tyra;
using namespace Tyra::SkelSkin::Cadence;

int checks = 0;
int failures = 0;

void check(bool ok, const char* what) {
  ++checks;
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
  }
}

}  // namespace

int main() {
  // Periods by band: lod distance 8 m, lod2 = 64 m^2.
  check(periodFor(0.0F, 64.0F) == 1, "a body at the camera is re-posed every frame");
  check(periodFor(64.0F, 64.0F) == 1, "a body at the band edge is still every frame");
  check(periodFor(65.0F, 64.0F) == 2, "beyond the distance: every 2nd frame");
  check(periodFor(256.0F, 64.0F) == 2, "at twice the distance: still every 2nd frame");
  check(periodFor(257.0F, 64.0F) == 4, "beyond twice the distance: every 4th frame");
  check(periodFor(1.0e6F, 64.0F) == 4, "far away: every 4th frame");
  check(periodFor(1.0e6F, 0.0F) == 1, "a distance of 0 (off): every frame");

  // Stagger and staleness, for 1..12 bodies and periods 2 and 4.
  const u32 periods[2] = {2, 4};
  bool everyWindowDue = true;
  bool spread = true;
  bool bounded = true;
  for (u32 period : periods)
    for (u32 bodies = 1; bodies <= 12; ++bodies) {
      for (u32 index = 0; index < bodies; ++index) {
        u32 dueCount = 0;
        for (u32 tick = 0; tick < period; ++tick)
          if (dueOnFrame(tick, index, period)) ++dueCount;
        if (dueCount != 1) spread = false;
        // staleness: the longest run of frames without a turn is period - 1
        u32 gap = 0, worst = 0;
        for (u32 tick = 0; tick < 64; ++tick) {
          if (dueOnFrame(tick, index, period)) gap = 0;
          else if (++gap > worst) worst = gap;
        }
        if (worst != period - 1) bounded = false;
      }
      for (u32 tick = 0; tick < 64; ++tick) {
        u32 due = 0;
        for (u32 index = 0; index < bodies; ++index)
          if (dueOnFrame(tick, index, period)) ++due;
        const u32 limit = (bodies + period - 1) / period;
        if (due > limit) spread = false;
      }
      for (u32 index = 0; index < bodies; ++index) {
        bool any = false;
        for (u32 tick = 0; tick < period; ++tick)
          if (repostThisFrame(tick, index, period, true, tick, 4, false)) any = true;
        if (!any) everyWindowDue = false;
      }
    }
  check(spread, "each body is due once per period; a frame's due bodies are at most ceil(n / period)");
  check(bounded, "a held pose is never older than period - 1 frames");
  check(everyWindowDue, "every body is re-posed within each period window");

  // Re-posing overrides the turn: no held pose, a gap longer than maxGap, a LOD change.
  check(repostThisFrame(5, 3, 4, false, 0, 4, false), "a body with no held pose is re-posed now");
  check(repostThisFrame(9, 0, 4, true, 2, 4, false), "a body not updated for more than 4 frames is re-posed now");
  check(repostThisFrame(6, 0, 4, true, 5, 4, true), "a body whose mesh LOD changed is re-posed now");
  check(!repostThisFrame(6, 0, 4, true, 5, 4, false) == !dueOnFrame(6, 0, 4),
        "otherwise the turn decides");
  check(dueOnFrame(7, 1, 4) && !dueOnFrame(7, 2, 4), "stagger: index 1 is due on frame 7 (period 4), index 2 is not");
  check(dueOnFrame(3, 5, 1) && dueOnFrame(4, 0, 1), "period 1 is every frame");

  std::printf("verify-skel-cadence: %d checks, %d failed\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
