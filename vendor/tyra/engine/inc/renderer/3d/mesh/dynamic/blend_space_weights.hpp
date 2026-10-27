/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Added by TyraX: the 2D blend-space weight function (docs/blend-spaces.md).
*/

#pragma once

// Header-only and free of every Tyra type on purpose: the engine
// (SkelInstance's blend-space mode) and the editor (the 2D Animation Blend
// Tool's preview, the --blendspace CLI) include this ONE file, so the preview
// and the console can never disagree on a weight. tools/verify-blendspace.cpp
// pins it on the host.

#include <cmath>

namespace Tyra {
namespace blendspace {

/** One clip placed on the compass: `dirDeg` relative to the object's facing
 * (+ = its right, wraps at +-180), `speed` in m/s (0 = the centre, every
 * direction). `ownTime` samples (an idle) loop on their own clock; the rest
 * share the blend space's phase. `clip` is the caller's index (a model clip
 * at runtime, an asset row in the editor) - compute() never reads it. */
struct Sample {
  int clip;
  float dirDeg;
  float speed;
  bool ownTime;
};

/** Up to 3 blended samples (indices into the Sample array, not clips),
 * heaviest first, weights summing to 1. count 0 = nothing to show. */
struct Weights {
  int sample[3] = {-1, -1, -1};
  float weight[3] = {0.0F, 0.0F, 0.0F};
  int count = 0;
};

/** The compass plane: x = speed toward the right, y = speed forward. The
 * input is smoothed as a point here, so a reversal crosses the centre. */
struct Point {
  float x, y;
};

constexpr int kMaxSamples = 32;        // extra samples are ignored
constexpr float kRingEpsilon = 0.05F;  // m/s: speeds closer than this are one ring
constexpr float kMinWeight = 0.01F;    // lighter weights are dropped (renormalised)

/** Degrees into (-180, 180]. */
inline float wrapDeg(float d) {
  d = std::fmod(d, 360.0F);
  if (d <= -180.0F) d += 360.0F;
  else if (d > 180.0F) d -= 360.0F;
  return d;
}

namespace detail {

inline float mod360(float d) {
  float r = std::fmod(d, 360.0F);
  if (r < 0.0F) r += 360.0F;
  if (r >= 360.0F) r -= 360.0F;
  return r;
}

// The two angular neighbours of `d` on a ring: weight (1 - f) on `a`, f on
// `b`. A ring of one sample answers it alone (a == b).
struct RingPick {
  int a = -1, b = -1;
  float f = 0.0F;
};

inline RingPick pick(const Sample* s, const int* ring, int n, float d) {
  RingPick p;
  float da = 361.0F, db = 361.0F;
  for (int i = 0; i < n; ++i) {
    const float before = mod360(d - s[ring[i]].dirDeg);  // how far d is past it
    if (before < da) da = before, p.a = ring[i];
  }
  if (da <= 0.0F || n == 1) {
    p.b = p.a;
    return p;
  }
  for (int i = 0; i < n; ++i) {
    const float after = mod360(s[ring[i]].dirDeg - d);  // how far ahead of d
    if (after > 0.0F && after < db) db = after, p.b = ring[i];
  }
  p.f = da / (da + db);
  return p;
}

// A small accumulator: merges a sample named twice, then keeps the 3
// heaviest above kMinWeight, renormalised, heaviest first.
struct Acc {
  int idx[6];
  float w[6];
  int n = 0;
  void add(int i, float weight) {
    if (i < 0 || weight <= 0.0F) return;
    for (int k = 0; k < n; ++k)
      if (idx[k] == i) {
        w[k] += weight;
        return;
      }
    if (n < 6) idx[n] = i, w[n] = weight, ++n;
  }
  Weights finish() {
    for (int a = 1; a < n; ++a)  // insertion sort, heaviest first
      for (int b = a; b > 0 && w[b] > w[b - 1]; --b) {
        const float tw = w[b];
        w[b] = w[b - 1], w[b - 1] = tw;
        const int ti = idx[b];
        idx[b] = idx[b - 1], idx[b - 1] = ti;
      }
    Weights out;
    float total = 0.0F;
    for (int k = 0; k < n && out.count < 3; ++k) {
      if (w[k] < kMinWeight) break;
      out.sample[out.count] = idx[k];
      out.weight[out.count] = w[k];
      total += w[k];
      ++out.count;
    }
    if (out.count == 0 && n > 0) {  // everything tiny: the heaviest alone
      out.sample[0] = idx[0], out.weight[0] = 1.0F, out.count = 1;
      return out;
    }
    for (int k = 0; k < out.count; ++k) out.weight[k] /= total;
    return out;
  }
};

}  // namespace detail

/**
 * The samples around (dirDeg, speed) and their weights (docs/blend-spaces.md):
 * on a ring, the two angular neighbours by angle; between the centre and the
 * first ring, the speed fraction; between two rings, the four corners of the
 * cell split along a diagonal (the triangle the point is in - continuous and
 * never more than 3 clips when both rings use the same directions). Below the
 * first ring without a centre sample, that ring alone; past the last ring,
 * the last ring alone.
 */
inline Weights compute(const Sample* s, int n, float dirDeg, float speed) {
  detail::Acc acc;
  if (!s || n <= 0) return acc.finish();
  if (n > kMaxSamples) n = kMaxSamples;
  const float d = wrapDeg(dirDeg);
  const float sp = speed > 0.0F ? speed : 0.0F;

  int centre = -1;
  float ringSpeed[kMaxSamples];
  int ringCount = 0;
  for (int i = 0; i < n; ++i) {
    if (s[i].speed < kRingEpsilon) {
      if (centre < 0) centre = i;
      continue;
    }
    bool found = false;
    for (int r = 0; r < ringCount && !found; ++r)
      found = std::fabs(s[i].speed - ringSpeed[r]) < kRingEpsilon;
    if (!found) ringSpeed[ringCount++] = s[i].speed;
  }
  for (int a = 1; a < ringCount; ++a)  // slowest ring first
    for (int b = a; b > 0 && ringSpeed[b] < ringSpeed[b - 1]; --b) {
      const float t = ringSpeed[b];
      ringSpeed[b] = ringSpeed[b - 1], ringSpeed[b - 1] = t;
    }
  if (ringCount == 0) {
    acc.add(centre, 1.0F);
    return acc.finish();
  }
  // A ring's members; a sample belongs to the first ring it is close to.
  const auto members = [&](int r, int* out) {
    int m = 0;
    for (int i = 0; i < n; ++i) {
      if (s[i].speed < kRingEpsilon) continue;
      int own = -1;
      for (int k = 0; k < ringCount && own < 0; ++k)
        if (std::fabs(s[i].speed - ringSpeed[k]) < kRingEpsilon) own = k;
      if (own == r) out[m++] = i;
    }
    return m;
  };
  int ring[kMaxSamples] = {};  // members() fills it; zeroed so GCC sees it set
  const auto ringAnswer = [&](int r, float share) {
    const int m = members(r, ring);
    const detail::RingPick p = detail::pick(s, ring, m, d);
    acc.add(p.a, share * (1.0F - p.f));
    acc.add(p.b, share * p.f);
  };

  if (sp >= ringSpeed[ringCount - 1]) {
    ringAnswer(ringCount - 1, 1.0F);
    return acc.finish();
  }
  int outer = 0;
  while (ringSpeed[outer] < sp) ++outer;  // exists: sp < the last ring
  if (outer == 0) {
    if (centre < 0) {
      ringAnswer(0, 1.0F);
      return acc.finish();
    }
    const float t = sp / ringSpeed[0];
    acc.add(centre, 1.0F - t);
    ringAnswer(0, t);
    return acc.finish();
  }
  // Between two rings: bilinear corners, then the diagonal split - the mass
  // outside the triangle the point is in moves along the diagonal (A1 -> B2),
  // which zeroes A2 or B1. Exactly that triangle's barycentric weights when
  // both rings carry the same directions.
  const float sIn = ringSpeed[outer - 1], sOut = ringSpeed[outer];
  const float t = (sp - sIn) / (sOut - sIn);
  const int mIn = members(outer - 1, ring);
  const detail::RingPick a = detail::pick(s, ring, mIn, d);
  const int mOut = members(outer, ring);
  const detail::RingPick b = detail::pick(s, ring, mOut, d);
  float wA1 = (1.0F - t) * (1.0F - a.f), wA2 = (1.0F - t) * a.f;
  float wB1 = t * (1.0F - b.f), wB2 = t * b.f;
  const float m = wA2 < wB1 ? wA2 : wB1;
  wA2 -= m, wB1 -= m, wA1 += m, wB2 += m;
  acc.add(a.a, wA1);
  acc.add(a.b, wA2);
  acc.add(b.a, wB1);
  acc.add(b.b, wB2);
  return acc.finish();
}

/** A blend space laid out as a 2D GRID instead of the radial compass
 * (docs/blend-spaces.md): `columns` vertical lines evenly from dirMin to
 * dirMax (degrees, across), `rows` horizontal lines evenly from speedMin to
 * speedMax (m/s, up); a sample sits on an intersection. Fewer than 2 lines
 * either way is no grid. */
struct Grid {
  int columns = 0, rows = 0;
  float dirMin = -180.0F, dirMax = 180.0F;
  float speedMin = 0.0F, speedMax = 0.5F;
};

/**
 * The grid's samples around (dirDeg, speed) and their weights: the point's
 * cell, its four corners weighted by where it sits in the cell, exactly like
 * the radial compass's cells - and when the corners name four different clips,
 * the compass's diagonal split, so never more than 3 blend. A grid laid out
 * like a radial asset (the centre's clip along the bottom row) blends exactly
 * like it (tools/verify-blendspace.cpp). An empty intersection takes the
 * nearest clip on its ROW (ties: the lower column); a row with none, the
 * nearest row's (ties: the lower row). Samples naming one clip count once
 * (the first of them answers). Outside the grid the point is clamped to its
 * edges; a sample off the intersections is ignored.
 */
inline Weights computeGrid(const Sample* s, int n, const Grid& g, float dirDeg, float speed) {
  detail::Acc acc;
  if (!s || n <= 0 || g.columns < 2 || g.rows < 2 || !(g.dirMax > g.dirMin) || !(g.speedMax > g.speedMin))
    return acc.finish();
  if (n > kMaxSamples) n = kMaxSamples;
  const float dx = (g.dirMax - g.dirMin) / (float)(g.columns - 1);
  const float dy = (g.speedMax - g.speedMin) / (float)(g.rows - 1);
  int col[kMaxSamples], row[kMaxSamples];
  for (int i = 0; i < n; ++i) {
    const float cx = (s[i].dirDeg - g.dirMin) / dx, cy = (s[i].speed - g.speedMin) / dy;
    const int c = (int)std::floor(cx + 0.5F), r = (int)std::floor(cy + 0.5F);
    const bool on = c >= 0 && c < g.columns && r >= 0 && r < g.rows && std::fabs(cx - (float)c) < 0.01F &&
                    std::fabs(cy - (float)r) < 0.01F;
    col[i] = on ? c : -1;
    row[i] = on ? r : -1;
  }
  // the sample answering intersection (c, r)
  const auto onRow = [&](int c, int r) {
    int best = -1, bestD = 0;
    for (int i = 0; i < n; ++i) {
      if (row[i] != r) continue;
      const int d = col[i] > c ? col[i] - c : c - col[i];
      if (best < 0 || d < bestD || (d == bestD && col[i] < col[best])) best = i, bestD = d;
    }
    return best;
  };
  const auto at = [&](int c, int r) {
    for (int k = 0; k < g.rows; ++k)
      for (int side = -1; side <= 1; side += 2) {  // the lower row first
        const int rr = r + side * k;
        if (rr < 0 || rr >= g.rows) continue;
        const int i = onRow(c, rr);
        if (i >= 0) return i;
        if (k == 0) break;
      }
    return -1;
  };
  // samples naming one clip count once: the first of them answers
  const auto first = [&](int i) {
    for (int k = 0; k < i; ++k)
      if (s[k].clip == s[i].clip && s[k].ownTime == s[i].ownTime) return k;
    return i;
  };

  float x = wrapDeg(dirDeg);
  x = x < g.dirMin ? g.dirMin : (x > g.dirMax ? g.dirMax : x);
  const float y = speed < g.speedMin ? g.speedMin : (speed > g.speedMax ? g.speedMax : speed);
  int i = (int)std::floor((x - g.dirMin) / dx), j = (int)std::floor((y - g.speedMin) / dy);
  i = i < 0 ? 0 : (i > g.columns - 2 ? g.columns - 2 : i);
  j = j < 0 ? 0 : (j > g.rows - 2 ? g.rows - 2 : j);
  // the fractions as the compass takes them - distances to the two lines -
  // so a grid laid out like a radial asset rounds like it too (a weight on
  // the kMinWeight edge falls the same way)
  const auto frac = [](float v, float lo, float hi) {
    const float a = v - lo, b = hi - v;
    if (a <= 0.0F) return 0.0F;
    if (b <= 0.0F) return 1.0F;
    return a / (a + b);
  };
  const float f = frac(x, g.dirMin + (float)i * dx, g.dirMin + (float)(i + 1) * dx);
  const float t = frac(y, g.speedMin + (float)j * dy, g.speedMin + (float)(j + 1) * dy);
  const int a1 = at(i, j), a2 = at(i + 1, j), b1 = at(i, j + 1), b2 = at(i + 1, j + 1);
  if (a1 < 0) return acc.finish();  // not one sample on the grid
  const int c1 = s[a1].clip, c2 = s[a2].clip, c3 = s[b1].clip, c4 = s[b2].clip;
  // a row whose two corners are one clip (the idle along the bottom) is that
  // clip's whole share at once - the compass's centre
  float wA1 = c1 == c2 ? 1.0F - t : (1.0F - t) * (1.0F - f), wA2 = c1 == c2 ? 0.0F : (1.0F - t) * f;
  float wB1 = c3 == c4 ? t : t * (1.0F - f), wB2 = c3 == c4 ? 0.0F : t * f;
  if (c1 != c2 && c1 != c3 && c1 != c4 && c2 != c3 && c2 != c4 && c3 != c4) {
    // four clips: the compass's diagonal split (A1 -> B2), never more than 3
    const float m = wA2 < wB1 ? wA2 : wB1;
    wA2 -= m, wB1 -= m, wA1 += m, wB2 += m;
  }
  acc.add(first(a1), wA1);
  acc.add(first(a2), wA2);
  acc.add(first(b1), wB1);
  acc.add(first(b2), wB2);
  return acc.finish();
}

/** A blend space's weights in its layout: the compass without a grid, the
 * grid with one - the engine, the tool's preview and the CLI all ask here. */
inline Weights weightsFor(const Sample* s, int n, const Grid* grid, float dirDeg, float speed) {
  return grid ? computeGrid(s, n, *grid, dirDeg, speed) : compute(s, n, dirDeg, speed);
}

/** The sample to show ALONE (the blend LOD): the heaviest, but `current`
 * (a sample index, -1 = none) is kept while it is blended and no other
 * outweighs it by `hysteresis` - so a body moving along a boundary does not
 * flip (and restart a crossfade) every frame. */
inline int heaviest(const Weights& w, int current, float hysteresis) {
  if (w.count == 0) return current;
  float cw = 0.0F;
  for (int k = 0; k < w.count; ++k)
    if (w.sample[k] == current) cw = w.weight[k];
  if (current >= 0 && cw > 0.0F && w.weight[0] - cw < hysteresis) return current;
  return w.sample[0];
}

/** The blend LOD's decision for a body `dist2` (squared) from the nearest
 * camera: one sample past `lodDist` (0 = always blend), with a band - a
 * blending body goes single only past lodDist x (1 + kLodBand) and a single
 * one blends again only inside lodDist x (1 - kLodBand). Without it a bot
 * holding at the distance (or a bobbing camera) flips every frame, and every
 * flip restarts a crossfade and costs a pose evaluation. */
constexpr float kLodBand = 0.1F;
inline bool lodSingle(bool wasSingle, float dist2, float lodDist) {
  if (lodDist <= 0.0F) return false;
  const float edge = lodDist * (wasSingle ? 1.0F - kLodBand : 1.0F + kLodBand);
  return dist2 > edge * edge;
}

inline Point toPoint(float dirDeg, float speed) {
  const float r = dirDeg * 0.017453292519943295F;
  return Point{speed * std::sin(r), speed * std::cos(r)};
}

inline void fromPoint(Point p, float* dirDeg, float* speed) {
  const float s = std::sqrt(p.x * p.x + p.y * p.y);
  *speed = s;
  *dirDeg = s > 1e-6F ? std::atan2(p.x, p.y) * 57.29577951308232F : 0.0F;
}

/** The point the smoothing CHASES for an input: its speed clamped to the
 * fastest ring. Past that ring the weights no longer change, and a fighter at
 * 4.6 m/s on a 0.5 m/s ring must not put the point nine rings out - the
 * smoothing would then cross the whole idle-to-stride band in one frame (a
 * pop the start of every run would show). */
inline Point inputPoint(const Sample* s, int n, float dirDeg, float speed) {
  float top = 0.0F;
  for (int i = 0; s && i < n && i < kMaxSamples; ++i)
    if (s[i].speed > top) top = s[i].speed;
  const float sp = speed > 0.0F ? speed : 0.0F;
  return toPoint(dirDeg, sp < top ? sp : top);
}

/** Moves `cur` toward `target` with time constant `tau` seconds (0 = snap;
 * dt <= 0, a paused frame, holds it). Frame-rate independent. */
inline Point smooth(Point cur, Point target, float dt, float tau) {
  if (tau <= 0.0F) return target;
  if (dt <= 0.0F) return cur;
  const float a = 1.0F - std::exp(-dt / tau);
  return Point{cur.x + (target.x - cur.x) * a, cur.y + (target.y - cur.y) * a};
}

}  // namespace blendspace
}  // namespace Tyra
