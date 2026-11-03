// Host harness for the blend-space weight function the engine (SkelInstance)
// and the editor (the 2D Animation Blend Tool) share, and the tool preview's
// move-speed rule (src/blendspace.hpp):
//   g++ -std=c++20 -O1 -Wall -Wextra -o verify-blendspace tools/verify-blendspace.cpp
//   ./verify-blendspace     -> "ALL n CHECKS PASSED", exit 0
// The layout used throughout is Gunner's combatent (.plan/48 in ../gunner):
// the idle at the centre, eight strides on one ring at 0.5 m/s.
#include <cmath>
#include <cstdio>

#include "../vendor/tyra/engine/inc/renderer/3d/mesh/dynamic/blend_space_weights.hpp"
#include "../src/blendspace.hpp"

using namespace Tyra::blendspace;

static int checks = 0, failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    ++checks;                                             \
    if (!(cond)) {                                        \
      ++failures;                                         \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__);   \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)

static bool near(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }

// The weight a sample got (0 when it is not among the blended ones).
static float weightOf(const Weights& w, int sample) {
  for (int i = 0; i < w.count; ++i)
    if (w.sample[i] == sample) return w.weight[i];
  return 0.0F;
}

static float sumOf(const Weights& w) {
  float s = 0.0F;
  for (int i = 0; i < w.count; ++i) s += w.weight[i];
  return s;
}

enum { kIdle, kJog, kJogFR, kStrafeR, kJogBR, kJogBack, kJogBL, kStrafeL, kJogFL, kCount };
static const Sample kCombatent[kCount] = {
    {0, 0.0F, 0.0F, true},     {1, 0.0F, 0.5F, false},   {2, 45.0F, 0.5F, false},
    {3, 90.0F, 0.5F, false},   {4, 135.0F, 0.5F, false}, {5, 180.0F, 0.5F, false},
    {6, -135.0F, 0.5F, false}, {7, -90.0F, 0.5F, false}, {8, -45.0F, 0.5F, false},
};

static Weights at(float dir, float speed) { return compute(kCombatent, kCount, dir, speed); }

static void single_sample_plays_alone() {
  const Sample one[1] = {{4, 0.0F, 0.5F, false}};
  const Weights w = compute(one, 1, 37.0F, 3.0F);
  CHECK(w.count == 1 && w.sample[0] == 0 && near(w.weight[0], 1.0F),
        "single sample: count %d weight %f", w.count, w.count ? w.weight[0] : -1.0F);
}

static void exact_direction_is_one_sample() {
  const Weights w = at(45.0F, 0.5F);
  CHECK(w.count == 1 && near(weightOf(w, kJogFR), 1.0F), "45 deg: count %d jog_fr %f", w.count,
        weightOf(w, kJogFR));
}

static void between_neighbours() {
  const Weights w = at(22.5F, 0.5F);
  CHECK(w.count == 2, "22.5 deg: count %d", w.count);
  CHECK(near(weightOf(w, kJog), 0.5F) && near(weightOf(w, kJogFR), 0.5F), "22.5 deg: jog %f jog_fr %f",
        weightOf(w, kJog), weightOf(w, kJogFR));
}

static void wraps() {
  const Weights a = at(-170.0F, 0.5F), b = at(190.0F, 0.5F);
  CHECK(near(weightOf(a, kJogBack), weightOf(b, kJogBack)) &&
            near(weightOf(a, kJogBL), weightOf(b, kJogBL)) && near(weightOf(a, kJogBack), 1.0F - 10.0F / 45.0F),
        "-170 vs 190: back %f/%f bl %f/%f", weightOf(a, kJogBack), weightOf(b, kJogBack),
        weightOf(a, kJogBL), weightOf(b, kJogBL));
  CHECK(near(weightOf(at(180.0F, 0.5F), kJogBack), 1.0F), "180 is jog_back");
  CHECK(near(weightOf(at(-180.0F, 0.5F), kJogBack), 1.0F), "-180 is jog_back");
  CHECK(near(wrapDeg(190.0F), -170.0F) && near(wrapDeg(-180.0F), 180.0F) && near(wrapDeg(720.0F), 0.0F),
        "wrapDeg: %f %f %f", wrapDeg(190.0F), wrapDeg(-180.0F), wrapDeg(720.0F));
}

static void centre_blend() {
  const Weights a = at(0.0F, 0.25F);
  CHECK(a.count == 2 && near(weightOf(a, kIdle), 0.5F) && near(weightOf(a, kJog), 0.5F),
        "(0, 0.25): count %d idle %f jog %f", a.count, weightOf(a, kIdle), weightOf(a, kJog));
  const Weights b = at(30.0F, 0.25F);
  CHECK(b.count == 3 && near(weightOf(b, kIdle), 0.5F) && near(weightOf(b, kJog), 0.5F * (15.0F / 45.0F)) &&
            near(weightOf(b, kJogFR), 0.5F * (30.0F / 45.0F)),
        "(30, 0.25): count %d idle %f jog %f jog_fr %f", b.count, weightOf(b, kIdle), weightOf(b, kJog),
        weightOf(b, kJogFR));
}

static void clamps() {
  CHECK(near(weightOf(at(0.0F, 9.0F), kJog), 1.0F), "past the ring: the ring alone");
  CHECK(near(weightOf(at(0.0F, -1.0F), kIdle), 1.0F), "negative speed: the centre");
  CHECK(near(weightOf(at(123.0F, 0.0F), kIdle), 1.0F), "speed 0: the centre whatever the direction");
}

static void no_centre_below_ring() {
  const Weights w = compute(kCombatent + 1, kCount - 1, 0.0F, 0.1F);
  CHECK(w.count == 1 && w.sample[0] == kJog - 1 && near(w.weight[0], 1.0F),
        "ring only, below it: count %d sample %d", w.count, w.count ? w.sample[0] : -1);
}

// Rings at 0.5 and 4.0 m/s, four directions each, plus a centre.
static const Sample kTwoRings[9] = {
    {0, 0.0F, 0.0F, true},   {1, 0.0F, 0.5F, false},  {2, 90.0F, 0.5F, false},
    {3, 180.0F, 0.5F, false}, {4, -90.0F, 0.5F, false}, {5, 0.0F, 4.0F, false},
    {6, 90.0F, 4.0F, false},  {7, 180.0F, 4.0F, false}, {8, -90.0F, 4.0F, false},
};

static void two_rings_at_most_three() {
  int worstCount = 0;
  float worstSum = 0.0F;
  for (int di = -180; di <= 180; ++di)
    for (int si = 0; si <= 100; ++si) {
      const Weights w = compute(kTwoRings, 9, (float)di, (float)si * 0.05F);
      if (w.count > worstCount) worstCount = w.count;
      const float err = std::fabs(sumOf(w) - 1.0F);
      if (err > worstSum) worstSum = err;
    }
  CHECK(worstCount <= 3, "two rings: up to %d clips", worstCount);
  CHECK(worstSum <= 1e-5F, "two rings: weights sum off by %g", worstSum);
  // On the inner ring exactly, and halfway out along a sample's direction.
  const Weights a = compute(kTwoRings, 9, 90.0F, 0.5F);
  CHECK(near(weightOf(a, 2), 1.0F), "inner ring sample alone: %f", weightOf(a, 2));
  const Weights b = compute(kTwoRings, 9, 90.0F, 2.25F);
  CHECK(near(weightOf(b, 2), 0.5F) && near(weightOf(b, 6), 0.5F), "halfway out at 90: %f %f",
        weightOf(b, 2), weightOf(b, 6));
}

// No sample's weight jumps between neighbouring points of a fine grid: the
// snap this whole feature exists to remove. A legitimate step is at most the
// grid step over the samples' spacing (1 deg of 45 = 0.022; 0.05 m/s of the
// 0.5 m/s ring = 0.1), plus a dropped light weight (< kMinWeight); a snap is
// a whole weight.
static void continuity() {
  const Sample* sets[2] = {kCombatent, kTwoRings};
  const int counts[2] = {kCount, 9};
  for (int set = 0; set < 2; ++set) {
    float worstDir = 0.0F, worstSpeed = 0.0F;
    int dirD = 0, dirS = 0, spdD = 0, spdS = 0;
    for (int di = -180; di <= 180; ++di)
      for (int si = 0; si <= 100; ++si) {
        const float d = (float)di, s = (float)si * 0.05F;
        const Weights here = compute(sets[set], counts[set], d, s);
        const Weights right = compute(sets[set], counts[set], d + 1.0F, s);
        const Weights up = compute(sets[set], counts[set], d, s + 0.05F);
        for (int k = 0; k < counts[set]; ++k) {
          const float a = std::fabs(weightOf(here, k) - weightOf(right, k));
          const float b = std::fabs(weightOf(here, k) - weightOf(up, k));
          if (a > worstDir) worstDir = a, dirD = di, dirS = si;
          if (b > worstSpeed) worstSpeed = b, spdD = di, spdS = si;
        }
      }
    CHECK(worstDir < 0.05F, "set %d: a weight jumps %f per degree near (%d deg, %.2f m/s)", set,
          worstDir, dirD, (float)dirS * 0.05F);
    CHECK(worstSpeed < 0.15F, "set %d: a weight jumps %f per 0.05 m/s near (%d deg, %.2f m/s)", set,
          worstSpeed, spdD, (float)spdS * 0.05F);
  }
}

static void min_weight_dropped() {
  const Weights w = at(0.3F, 0.5F);  // jog_fr would get 0.3 / 45 = 0.0067
  CHECK(w.count == 1 && near(weightOf(w, kJog), 1.0F), "0.3 deg: count %d jog %f", w.count,
        weightOf(w, kJog));
}

static void heaviest_hysteresis() {
  CHECK(heaviest(at(23.0F, 0.5F), kJog, 0.1F) == kJog, "23 deg keeps jog");
  CHECK(heaviest(at(30.0F, 0.5F), kJog, 0.1F) == kJogFR, "30 deg switches to jog_fr");
  CHECK(heaviest(at(30.0F, 0.5F), -1, 0.1F) == kJogFR, "no current: the heaviest");
  CHECK(heaviest(at(100.0F, 0.5F), kJog, 0.1F) == kStrafeR, "current not blended: the heaviest");
}

static void smoothing_dt_zero_holds() {
  const Point p = toPoint(-90.0F, 0.5F), t = toPoint(90.0F, 0.5F);
  const Point q = smooth(p, t, 0.0F, 0.15F);
  CHECK(near(q.x, p.x) && near(q.y, p.y), "dt 0 (paused) holds the point");
  const Point r = smooth(p, t, 0.016F, 0.0F);
  CHECK(near(r.x, t.x) && near(r.y, t.y), "smoothing 0 snaps to the input");
}

static void smoothing_reversal_passes_centre() {
  // Strafe left -> strafe right: the smoothed POINT crosses the centre (the
  // idle) instead of spinning round the ring through jog or jog_back.
  Point p = toPoint(-90.0F, 0.5F);
  const Point t = toPoint(90.0F, 0.5F);
  float minSpeed = 1.0F;
  for (int i = 0; i < 120; ++i) {
    p = smooth(p, t, 1.0F / 60.0F, 0.15F);
    float d, s;
    fromPoint(p, &d, &s);
    if (s < minSpeed) minSpeed = s;
  }
  CHECK(minSpeed < 0.05F, "the reversal's slowest point %f m/s", minSpeed);
  float d, s;
  fromPoint(p, &d, &s);
  CHECK(near(d, 90.0F, 0.5F) && near(s, 0.5F, 0.01F), "converged to (90, 0.5): (%f, %f)", d, s);
  fromPoint(toPoint(37.0F, 2.0F), &d, &s);
  CHECK(near(d, 37.0F) && near(s, 2.0F), "point round trip: (%f, %f)", d, s);
}

// A start: 0 -> 4.6 m/s (Gunner's run) at 60 Hz, smoothing 0.15 s. The point
// the smoothing chases is clamped to the fastest ring, so the idle hands over
// to the stride over several frames instead of one - unclamped, the point
// would chase 4.6 m/s, nine rings out, and cross the 0.5 m/s ring at once.
static void smoothing_start_is_gradual() {
  Point p = inputPoint(kCombatent, kCount, 0.0F, 0.0F);
  float prevIdle = 1.0F, worstStep = 0.0F;
  int between = 0;
  for (int i = 0; i < 60; ++i) {
    p = smooth(p, inputPoint(kCombatent, kCount, 0.0F, 4.6F), 1.0F / 60.0F, 0.15F);
    float d, s;
    fromPoint(p, &d, &s);
    const float idle = weightOf(compute(kCombatent, kCount, d, s), kIdle);
    const float step = std::fabs(idle - prevIdle);
    worstStep = step > worstStep ? step : worstStep;
    between += idle > 0.1F && idle < 0.9F;
    prevIdle = idle;
  }
  CHECK(worstStep < 0.15F, "a start moves the idle's weight %f in one frame", worstStep);
  CHECK(between >= 8, "a start passes %d frames between idle and stride (want >= 8)", between);
  CHECK(prevIdle < 0.01F, "a start ends on the stride (idle %f)", prevIdle);
  const Point q = inputPoint(kCombatent, kCount, 90.0F, 4.6F);
  float d, s;
  fromPoint(q, &d, &s);
  CHECK(near(s, 0.5F) && near(d, 90.0F, 0.01F), "the input clamps to the fastest ring: (%f, %f)", d, s);
}

// The blend LOD's distance has a band: a body at 8 m +- a little (a bot
// holding there, the camera bobbing) must not flip blended / single every
// frame - each flip restarts a crossfade and costs a pose evaluation.
static void blend_lod_hysteresis() {
  CHECK(!lodSingle(false, 8.5F * 8.5F, 8.0F), "blended, 8.5 m: still blended (inside the band)");
  CHECK(lodSingle(false, 9.0F * 9.0F, 8.0F), "blended, 9 m: one sample");
  CHECK(lodSingle(true, 7.5F * 7.5F, 8.0F), "one sample, 7.5 m: still one sample (inside the band)");
  CHECK(!lodSingle(true, 7.0F * 7.0F, 8.0F), "one sample, 7 m: blended again");
  CHECK(!lodSingle(true, 50.0F * 50.0F, 0.0F), "a 0 distance: always blend");
  int flips = 0;
  bool single = false;
  for (int i = 0; i < 200; ++i) {  // pacing 7.7 <-> 8.3 m
    const float d = 8.0F + 0.3F * std::sin(0.3F * (float)i);
    const bool next = lodSingle(single, d * d, 8.0F);
    flips += next != single;
    single = next;
  }
  CHECK(flips == 0, "pacing across 8 m flipped %d times", flips);
}

// The tool's preview: the compass ends just past the ring, but a body really
// moves at running speeds - where the engine clamps the weights to the ring.
// On or past the ring the preview moves at the Move speed (default: the
// clips' own speed); inside it, at the point's share of the ring of it - a
// point 87 % of the way to the jog moves at 87 % of the jog's speed, and the
// speed never jumps on the way out (a user report: it crept, then leapt 7x).
static void preview_move_speed() {
  blendspace::Asset a;
  a.stride = 2.11644F;
  a.samples = {{"idle", 0.0F, 0.0F, true}, {"jog", 0.0F, 0.5F, false}, {"strafe_r", 90.0F, 0.5F, false}};
  CHECK(near(blendspace::previewMoveSpeed(a, 0.25F, 3.2F), 1.6F), "half way to the ring: half the move speed (%f)",
        blendspace::previewMoveSpeed(a, 0.25F, 3.2F));
  CHECK(near(blendspace::previewMoveSpeed(a, 0.435F, 3.2F), 2.784F), "87 %% of the way: 87 %% of it (%f)",
        blendspace::previewMoveSpeed(a, 0.435F, 3.2F));
  CHECK(near(blendspace::previewMoveSpeed(a, 0.5F, 3.2F), 3.2F), "on the ring: the move speed");
  CHECK(near(blendspace::previewMoveSpeed(a, 6.8F, 3.2F), 6.8F), "typed past the ring and faster: that speed");
  float worst = 0.0F, prev = blendspace::previewMoveSpeed(a, 0.0F, 3.2F);
  for (int i = 1; i <= 400; ++i) {  // 0 -> 2 m/s in 5 mm/s steps
    const float v = blendspace::previewMoveSpeed(a, 0.005F * (float)i, 3.2F);
    worst = std::fmax(worst, std::fabs(v - prev));
    prev = v;
  }
  CHECK(worst < 0.05F, "dragging out from the centre never jumps the speed: worst step %f m/s", worst);
  CHECK(near(blendspace::previewMoveSpeed(a, 0.3F, 0.2F), 0.3F), "a move speed below the ring: the point's own");
  blendspace::Asset c;  // a centre only: no ring to be on
  c.samples = {{"idle", 0.0F, 0.0F, true}};
  CHECK(near(blendspace::previewMoveSpeed(c, 0.3F, 3.2F), 0.3F), "no ring: the point's speed");
  const float cycles[2] = {0.6667F, 0.6667F};
  CHECK(near(blendspace::authoredSpeed(a.stride, cycles, 2), 3.1745F, 0.001F), "authored: stride / cycle = %f",
        blendspace::authoredSpeed(a.stride, cycles, 2));
  CHECK(blendspace::authoredSpeed(0.0F, cycles, 2) == 0.0F, "no stride: none");
  CHECK(blendspace::authoredSpeed(2.0F, cycles, 0) == 0.0F, "no sync clip: none");
}

// --- the GRID layout (docs/blend-spaces.md): direction across, speed up, a
// clip on an intersection; an empty intersection takes the nearest clip on
// its row (user ruling). Cells blend like the radial compass, so a grid laid
// out like a radial asset must blend EXACTLY like it.

// The weight a CLIP got (samples naming one clip summed): a grid repeats the
// idle along its bottom row, the compass has it once, at the centre.
static float clipWeight(const Sample* s, const Weights& w, int clip) {
  float sum = 0.0F;
  for (int i = 0; i < w.count; ++i)
    if (s[w.sample[i]].clip == clip) sum += w.weight[i];
  return sum;
}

// The combatent's compass as a 9 x 2 grid: -180..180 every 45 deg, 0 and
// 0.5 m/s; the idle along the whole bottom row, jog_back at both ends.
static const Grid kCombatentGrid{9, 2, -180.0F, 180.0F, 0.0F, 0.5F};
static const Sample kCombatentGridSamples[18] = {
    {kIdle, -180.0F, 0.0F, true},       {kIdle, -135.0F, 0.0F, true},   {kIdle, -90.0F, 0.0F, true},
    {kIdle, -45.0F, 0.0F, true},        {kIdle, 0.0F, 0.0F, true},      {kIdle, 45.0F, 0.0F, true},
    {kIdle, 90.0F, 0.0F, true},         {kIdle, 135.0F, 0.0F, true},    {kIdle, 180.0F, 0.0F, true},
    {kJogBack, -180.0F, 0.5F, false},   {kJogBL, -135.0F, 0.5F, false}, {kStrafeL, -90.0F, 0.5F, false},
    {kJogFL, -45.0F, 0.5F, false},      {kJog, 0.0F, 0.5F, false},      {kJogFR, 45.0F, 0.5F, false},
    {kStrafeR, 90.0F, 0.5F, false},     {kJogBR, 135.0F, 0.5F, false},  {kJogBack, 180.0F, 0.5F, false},
};

// Every clip's weight, grid against radial, over the compass (every 0.25 deg,
// every 0.01 m/s to past the ring): returns the worst difference.
static float worstGridVsRadial(const Sample* rad, int rn, const Sample* grid, int gn, const Grid& g,
                               int clips, float maxSpeed, int* worstCount, float* atDir, float* atSpeed) {
  float worst = 0.0F;
  *worstCount = 0;
  for (int di = -720; di <= 720; ++di)
    for (int si = 0; si <= (int)(maxSpeed * 100.0F); ++si) {
      const float d = 0.25F * (float)di, sp = 0.01F * (float)si;
      const Weights r = compute(rad, rn, d, sp);
      const Weights w = computeGrid(grid, gn, g, d, sp);
      if (w.count > *worstCount) *worstCount = w.count;
      for (int c = 0; c < clips; ++c) {
        const float e = std::fabs(clipWeight(rad, r, c) - clipWeight(grid, w, c));
        if (e > worst) worst = e, *atDir = d, *atSpeed = sp;
      }
    }
  return worst;
}

static void grid_matches_radial() {
  int most = 0;
  float d = 0.0F, s = 0.0F;
  const float worst =
      worstGridVsRadial(kCombatent, kCount, kCombatentGridSamples, 18, kCombatentGrid, kCount, 0.7F, &most, &d, &s);
  CHECK(worst < 1e-5F, "grid vs radial (combatent): a clip's weight off by %f at (%.2f deg, %.2f m/s)", worst, d, s);
  CHECK(most <= 3, "grid blends at most 3 clips: %d", most);
}

// Two evenly spaced rings (a grid's rows are evenly spaced): between them the
// radial compass splits each cell along its diagonal - the grid must too.
static const Sample kEvenRings[9] = {
    {0, 0.0F, 0.0F, true},    {1, 0.0F, 0.5F, false},  {2, 90.0F, 0.5F, false},
    {3, 180.0F, 0.5F, false}, {4, -90.0F, 0.5F, false}, {5, 0.0F, 1.0F, false},
    {6, 90.0F, 1.0F, false},  {7, 180.0F, 1.0F, false}, {8, -90.0F, 1.0F, false},
};
static const Grid kEvenRingsGrid{5, 3, -180.0F, 180.0F, 0.0F, 1.0F};
static const Sample kEvenRingsGridSamples[11] = {
    {0, 0.0F, 0.0F, true},                                     // one idle: the row borrows it
    {3, -180.0F, 0.5F, false}, {4, -90.0F, 0.5F, false}, {1, 0.0F, 0.5F, false}, {2, 90.0F, 0.5F, false},
    {3, 180.0F, 0.5F, false},  {7, -180.0F, 1.0F, false}, {8, -90.0F, 1.0F, false}, {5, 0.0F, 1.0F, false},
    {6, 90.0F, 1.0F, false},   {7, 180.0F, 1.0F, false},
};

static void grid_matches_radial_two_rings() {
  int most = 0;
  float d = 0.0F, s = 0.0F;
  const float worst =
      worstGridVsRadial(kEvenRings, 9, kEvenRingsGridSamples, 11, kEvenRingsGrid, 9, 1.2F, &most, &d, &s);
  CHECK(worst < 1e-5F, "grid vs radial (two rings): a clip's weight off by %f at (%.2f deg, %.2f m/s)", worst, d, s);
  CHECK(most <= 3, "grid blends at most 3 clips between two rows: %d", most);
}

// The smallest grid: a square, four clips on its corners.
static void grid_minimal_square() {
  const Grid g{2, 2, -90.0F, 90.0F, 0.0F, 1.0F};
  const Sample s[4] = {{0, -90.0F, 0.0F, false}, {1, 90.0F, 0.0F, false}, {2, -90.0F, 1.0F, false},
                       {3, 90.0F, 1.0F, false}};
  Weights w = computeGrid(s, 4, g, -90.0F, 0.0F);
  CHECK(w.count == 1 && s[w.sample[0]].clip == 0, "a corner plays its clip alone (%d clips)", w.count);
  w = computeGrid(s, 4, g, 90.0F, 1.0F);
  CHECK(w.count == 1 && s[w.sample[0]].clip == 3, "the opposite corner too");
  w = computeGrid(s, 4, g, 0.0F, 0.0F);
  CHECK(near(clipWeight(s, w, 0), 0.5F) && near(clipWeight(s, w, 1), 0.5F), "the bottom edge's middle: half each");
  w = computeGrid(s, 4, g, 0.0F, 0.5F);
  CHECK(w.count <= 3 && near(sumOf(w), 1.0F), "the middle: at most 3 clips, weights sum to 1 (%d, %f)", w.count,
        sumOf(w));
}

// An empty intersection takes the nearest clip on its row (user ruling): one
// idle on the bottom row blends exactly like the idle on all of it.
static void grid_empty_takes_nearest_on_row() {
  Sample one[10];
  one[0] = {kIdle, 0.0F, 0.0F, true};
  for (int i = 0; i < 9; ++i) one[1 + i] = kCombatentGridSamples[9 + i];
  int most = 0;
  float d = 0.0F, s = 0.0F;
  const float worst = worstGridVsRadial(kCombatent, kCount, one, 10, kCombatentGrid, kCount, 0.7F, &most, &d, &s);
  CHECK(worst < 1e-5F, "one idle on the bottom row = the idle along all of it: off by %f at (%.2f, %.2f)", worst, d, s);
  // a gap on the top row: 90 deg is empty, 45 and 135 tie - the lower column wins
  Sample gap[17];
  int n = 0;
  for (const Sample& x : kCombatentGridSamples)
    if (!(x.clip == kStrafeR)) gap[n++] = x;
  const Weights w = computeGrid(gap, n, kCombatentGrid, 90.0F, 0.5F);
  CHECK(w.count == 1 && gap[w.sample[0]].clip == kJogFR, "an empty spot borrows the nearest on its row (%d clips)",
        w.count);
}

// A row with no clip at all takes the nearest row's (ties: the lower row).
static void grid_empty_row_takes_nearest_row() {
  const Grid g{2, 3, -90.0F, 90.0F, 0.0F, 1.0F};
  const Sample s[4] = {{0, -90.0F, 0.0F, false}, {1, 90.0F, 0.0F, false}, {2, -90.0F, 1.0F, false},
                       {3, 90.0F, 1.0F, false}};
  const Weights w = computeGrid(s, 4, g, -90.0F, 0.5F);
  CHECK(w.count == 1 && s[w.sample[0]].clip == 0, "the empty middle row: the lower row's clip (%d clips)", w.count);
}

// Outside the grid: clamped to its edges.
static void grid_clamps() {
  Weights w = computeGrid(kCombatentGridSamples, 18, kCombatentGrid, 0.0F, 4.6F);
  CHECK(w.count == 1 && kCombatentGridSamples[w.sample[0]].clip == kJog, "past the top row: the top row");
  w = computeGrid(kCombatentGridSamples, 18, kCombatentGrid, 30.0F, -1.0F);
  CHECK(w.count == 1 && kCombatentGridSamples[w.sample[0]].clip == kIdle, "below the bottom row: the bottom row");
  const Grid narrow{3, 2, -90.0F, 90.0F, 0.0F, 0.5F};
  const Sample s[4] = {{0, -90.0F, 0.0F, true}, {1, -90.0F, 0.5F, false}, {2, 0.0F, 0.5F, false},
                       {3, 90.0F, 0.5F, false}};
  w = computeGrid(s, 4, narrow, 170.0F, 0.5F);
  CHECK(w.count == 1 && s[w.sample[0]].clip == 3, "a direction past the grid: its edge column");
  const Grid bad{1, 2, -90.0F, 90.0F, 0.0F, 0.5F};
  CHECK(computeGrid(s, 4, bad, 0.0F, 0.5F).count == 0, "fewer than 2 lines: no blend");
}

// weightsFor: the compass without a grid, the grid with one.
static void weights_for_picks_the_layout() {
  const Weights a = weightsFor(kCombatent, kCount, nullptr, 30.0F, 0.3F);
  const Weights b = compute(kCombatent, kCount, 30.0F, 0.3F);
  CHECK(a.count == b.count && a.sample[0] == b.sample[0] && near(a.weight[0], b.weight[0]), "no grid: the compass");
  const Weights c = weightsFor(kCombatentGridSamples, 18, &kCombatentGrid, 30.0F, 0.3F);
  const Weights d = computeGrid(kCombatentGridSamples, 18, kCombatentGrid, 30.0F, 0.3F);
  CHECK(c.count == d.count && c.sample[0] == d.sample[0] && near(c.weight[0], d.weight[0]), "a grid: the grid");
}

// The combatent's locomotion as its asset says it (src/blendspace.hpp).
static blendspace::Asset radialLocomotion() {
  blendspace::Asset a;
  a.model = "res/models/combatent.glb";
  a.name = "locomotion";
  a.stride = 2.11644F;
  a.smoothing = 0.15F;
  a.samples = {{"idle", 0.0F, 0.0F, true},       {"jog", 0.0F, 0.5F, false},     {"jog_fr", 45.0F, 0.5F, false},
               {"strafe_r", 90.0F, 0.5F, false}, {"jog_br", 135.0F, 0.5F, false}, {"jog_back", 180.0F, 0.5F, false},
               {"jog_bl", -135.0F, 0.5F, false}, {"strafe_l", -90.0F, 0.5F, false}, {"jog_fl", -45.0F, 0.5F, false}};
  return a;
}

// The weight a clip NAME got (rows naming it summed).
static float nameWeight(const blendspace::Asset& a, const Weights& w, const std::string& clip) {
  float sum = 0.0F;
  for (int i = 0; i < w.count; ++i)
    if (a.samples[(size_t)w.sample[i]].clip == clip) sum += w.weight[i];
  return sum;
}

// The tool's "Save as grid": the radial asset's grid version - 9 x 2 over
// -180..180 deg and 0..0.5 m/s, the idle along the bottom row, jog_back at
// both ends - blends exactly like the compass, through weightSamples (the
// editor's own rows, a clip's rows counting once).
static void to_grid_blends_like_the_compass() {
  const blendspace::Asset r = radialLocomotion();
  blendspace::Asset g;
  std::string err;
  CHECK(blendspace::toGrid(r, g, err), "the combatent converts: %s", err.c_str());
  CHECK(g.grid && g.lines.columns == 9 && g.lines.rows == 2 && near(g.lines.dirMin, -180.0F) &&
            near(g.lines.dirMax, 180.0F) && near(g.lines.speedMin, 0.0F) && near(g.lines.speedMax, 0.5F),
        "9 x 2 over -180..180 deg, 0..0.5 m/s: %d x %d", g.lines.columns, g.lines.rows);
  CHECK(g.samples.size() == 18, "the idle along the bottom row, jog_back at both ends: %d samples",
        (int)g.samples.size());
  CHECK(g.name == r.name && near(g.stride, r.stride) && g.model == r.model, "the rest is the radial asset's");
  const std::vector<Sample> rs = blendspace::weightSamples(r), gs = blendspace::weightSamples(g);
  float worst = 0.0F, wd = 0.0F, wsp = 0.0F;
  for (int di = -360; di <= 360; ++di)
    for (int si = 0; si <= 70; ++si) {
      const float d = 0.5F * (float)di, sp = 0.01F * (float)si;
      const Weights a = weightsFor(rs.data(), (int)rs.size(), nullptr, d, sp);
      const Weights b = weightsFor(gs.data(), (int)gs.size(), &g.lines, d, sp);
      for (const blendspace::AssetSample& s : r.samples) {
        const float e = std::fabs(nameWeight(r, a, s.clip) - nameWeight(g, b, s.clip));
        if (e > worst) worst = e, wd = d, wsp = sp;
      }
    }
  CHECK(worst < 1e-5F, "toGrid blends like the compass: off by %f at (%.1f deg, %.2f m/s)", worst, wd, wsp);
}

// A layout no grid reproduces is refused, saying why: rings not evenly spaced
// from 0 (an empty row in between would take the nearest row, not blend),
// and a ring missing a direction the others have.
static void to_grid_refuses_what_no_grid_reproduces() {
  blendspace::Asset uneven = radialLocomotion();
  uneven.samples.push_back({"jog", 0.0F, 1.2F, false});
  blendspace::Asset g;
  std::string err;
  CHECK(!blendspace::toGrid(uneven, g, err) && !err.empty(), "uneven rings: refused (%s)", err.c_str());
  blendspace::Asset gap = radialLocomotion();
  for (const char* c : {"jog", "jog_fr", "strafe_r", "jog_br", "jog_back", "jog_bl", "strafe_l", "jog_fl"})
    gap.samples.push_back({c, 0.0F, 1.0F, false});
  for (size_t k = 0; k < 8; ++k) gap.samples[9 + k].dir = gap.samples[1 + k].dir;
  gap.samples.pop_back();  // the 1 m/s ring lacks -45 deg
  err.clear();
  CHECK(!blendspace::toGrid(gap, g, err) && !err.empty(), "a ring with a gap: refused (%s)", err.c_str());
}

// Back on the compass: what toGrid made comes back sample for sample - the
// bottom row's clip once at the centre, the -180 deg copy dropped.
static void to_radial_undoes_to_grid() {
  const blendspace::Asset r = radialLocomotion();
  blendspace::Asset g;
  std::string err;
  blendspace::toGrid(r, g, err);
  const blendspace::Asset back = blendspace::toRadial(g);
  CHECK(!back.grid && back.samples.size() == r.samples.size(), "back on the compass: %d samples, %s",
        (int)back.samples.size(), back.grid ? "still a grid" : "radial");
  int missing = 0;
  for (const blendspace::AssetSample& s : r.samples) {
    bool found = false;
    for (const blendspace::AssetSample& o : back.samples)
      found |= o.clip == s.clip && near(wrapDeg(o.dir), wrapDeg(s.dir)) && near(o.speed, s.speed) && o.own == s.own;
    missing += !found;
  }
  CHECK(missing == 0, "every radial sample comes back as it was: %d missing", missing);
}

// The Layout switch greys Radial out unless the compass blends EXACTLY like
// the grid: canRadial says what is in the way and what to do about it.
static blendspace::Asset gridLocomotion() {
  blendspace::Asset g;
  std::string err;
  blendspace::toGrid(radialLocomotion(), g, err);
  return g;
}
static blendspace::AssetSample* at(blendspace::Asset& a, float dir, float speed) {
  for (blendspace::AssetSample& s : a.samples)
    if (near(s.dir, dir) && near(s.speed, speed)) return &s;
  return nullptr;
}

static void can_radial_accepts_what_to_grid_made() {
  std::string why;
  CHECK(blendspace::canRadial(gridLocomotion(), why), "toGrid's own grid goes back: %s", why.c_str());
  // one idle on the bottom row, the rest of it empty: still one centre
  blendspace::Asset one = gridLocomotion();
  std::vector<blendspace::AssetSample> kept;
  for (const blendspace::AssetSample& s : one.samples)
    if (s.speed > 0.0F || near(s.dir, 0.0F)) kept.push_back(s);
  one.samples = kept;
  why.clear();
  CHECK(blendspace::canRadial(one, why), "one idle on the bottom row goes back: %s", why.c_str());
  // and the compass then blends exactly like that grid
  const blendspace::Asset r = blendspace::toRadial(one);
  const std::vector<Sample> gs = blendspace::weightSamples(one), rs = blendspace::weightSamples(r);
  float worst = 0.0F;
  for (int di = -180; di <= 180; ++di)
    for (int si = 0; si <= 70; ++si) {
      const float d = (float)di, sp = 0.01F * (float)si;
      const Weights a = weightsFor(gs.data(), (int)gs.size(), &one.lines, d, sp);
      const Weights b = weightsFor(rs.data(), (int)rs.size(), nullptr, d, sp);
      for (const blendspace::AssetSample& s : one.samples)
        worst = std::fmax(worst, std::fabs(nameWeight(one, a, s.clip) - nameWeight(r, b, s.clip)));
    }
  CHECK(worst < 1e-5F, "back on the compass it blends like the grid: off by %f", worst);
}

static void can_radial_refuses_what_the_compass_cannot_do() {
  std::string why;
  blendspace::Asset two = gridLocomotion();  // two clips on the bottom row
  at(two, 90.0F, 0.0F)->clip = "crouch";
  CHECK(!blendspace::canRadial(two, why) && why.find("bottom row") != std::string::npos,
        "two clips on the bottom row: refused, saying so (%s)", why.c_str());
  CHECK(why.find("crouch") != std::string::npos && why.find("idle") != std::string::npos,
        "... naming the clips: %s", why.c_str());
  why.clear();
  blendspace::Asset gap = gridLocomotion();  // a hole in the ring row
  for (size_t i = 0; i < gap.samples.size(); ++i)
    if (near(gap.samples[i].dir, 45.0F) && near(gap.samples[i].speed, 0.5F)) gap.samples.erase(gap.samples.begin() + (long)i);
  CHECK(!blendspace::canRadial(gap, why) && why.find("45") != std::string::npos,
        "an empty spot on a moving row: refused, naming it (%s)", why.c_str());
  why.clear();
  blendspace::Asset ends = gridLocomotion();  // -180 and 180 disagree
  at(ends, -180.0F, 0.5F)->clip = "jog_bl";
  CHECK(!blendspace::canRadial(ends, why) && why.find("-180") != std::string::npos,
        "different clips at -180 and 180 deg: refused (%s)", why.c_str());
  why.clear();
  blendspace::Asset half = gridLocomotion();  // only half the way round
  half.lines.dirMin = -90.0F, half.lines.dirMax = 90.0F;
  CHECK(!blendspace::canRadial(half, why) && why.find("-180..180") != std::string::npos,
        "directions not all the way round: refused, saying set -180..180 (%s)", why.c_str());
  why.clear();
  blendspace::Asset mid = gridLocomotion();  // an empty row between two full ones
  mid.lines.rows = 3, mid.lines.speedMax = 1.0F;
  for (blendspace::AssetSample& s : mid.samples)
    if (s.speed > 0.25F) s.speed = 1.0F;
  CHECK(!blendspace::canRadial(mid, why) && why.find("0.5") != std::string::npos,
        "an empty row between two others: refused, naming it (%s)", why.c_str());
}

int main() {
  can_radial_accepts_what_to_grid_made();
  can_radial_refuses_what_the_compass_cannot_do();
  to_radial_undoes_to_grid();
  weights_for_picks_the_layout();
  to_grid_blends_like_the_compass();
  to_grid_refuses_what_no_grid_reproduces();
  grid_matches_radial();
  grid_matches_radial_two_rings();
  grid_minimal_square();
  grid_empty_takes_nearest_on_row();
  grid_empty_row_takes_nearest_row();
  grid_clamps();
  preview_move_speed();
  single_sample_plays_alone();
  blend_lod_hysteresis();
  smoothing_start_is_gradual();
  exact_direction_is_one_sample();
  between_neighbours();
  wraps();
  centre_blend();
  clamps();
  no_centre_below_ring();
  two_rings_at_most_three();
  continuity();
  min_weight_dropped();
  heaviest_hysteresis();
  smoothing_dt_zero_holds();
  smoothing_reversal_passes_centre();
  if (failures) {
    std::printf("%d of %d CHECKS FAILED\n", failures, checks);
    return 1;
  }
  std::printf("ALL %d CHECKS PASSED\n", checks);
  return 0;
}
