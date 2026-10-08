/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022-2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Wellinator Carvalho <wellcoj@gmail.com>
*/

#include "physics/ray.hpp"
#include <tamtypes.h>
#include <math.h>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace Tyra {

Ray::Ray() {}
Ray::Ray(const Vec4& t_origin, const Vec4& t_direction) {
  origin.set(t_origin);
  direction.set(t_direction);
}

Ray::~Ray() {}

Vec4 Ray::at(const float& t) const { return (direction * t) + origin; }

float Ray::distanceToPoint(const Vec4& point) const {
  return origin.distanceTo(point);
}

bool Ray::intersectBox(const Vec4& minCorner, const Vec4& maxCorner,
                       float* outputDistance) const {
  auto min = (minCorner - origin) / direction;
  auto max = (maxCorner - origin) / direction;

  float tmin =
      std::max(std::max(std::min(min.x, max.x), std::min(min.y, max.y)),
               std::min(min.z, max.z));
  float tmax =
      std::min(std::min(std::max(min.x, max.x), std::max(min.y, max.y)),
               std::max(min.z, max.z));

  // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
  if (tmax < 0) {
    if (outputDistance != nullptr) {
      *outputDistance = -1.0f;
    }
    return false;
  }

  // if tmin > tmax, ray doesn't intersect AABB
  if (tmin > tmax) {
    if (outputDistance != nullptr) {
      *outputDistance = -1.0f;
    }
    return false;
  }

  if (tmin < 0) {
    if (outputDistance != nullptr) {
      *outputDistance = tmax;
    }
    return true;
  }

  if (outputDistance != nullptr) {
    *outputDistance = tmin;
  }

  return true;
}

// Added by TyraX. Moller-Trumbore, both faces.
bool Ray::intersectTriangle(const Vec4& a, const Vec4& b, const Vec4& c,
                            float* outputDistance) const {
  const Vec4 e1 = b - a;
  const Vec4 e2 = c - a;
  const float px = direction.y * e2.z - direction.z * e2.y;
  const float py = direction.z * e2.x - direction.x * e2.z;
  const float pz = direction.x * e2.y - direction.y * e2.x;
  const float det = e1.x * px + e1.y * py + e1.z * pz;
  if (det > -1e-8F && det < 1e-8F) return false;
  const float invDet = 1.0F / det;
  const Vec4 t = origin - a;
  const float u = (t.x * px + t.y * py + t.z * pz) * invDet;
  if (u < 0.0F || u > 1.0F) return false;
  const float qx = t.y * e1.z - t.z * e1.y;
  const float qy = t.z * e1.x - t.x * e1.z;
  const float qz = t.x * e1.y - t.y * e1.x;
  const float v =
      (direction.x * qx + direction.y * qy + direction.z * qz) * invDet;
  if (v < 0.0F || u + v > 1.0F) return false;
  const float dist = (e2.x * qx + e2.y * qy + e2.z * qz) * invDet;
  if (dist < 0.0F) return false;
  if (outputDistance != nullptr) *outputDistance = dist;
  return true;
}

Vec4 Ray::invDir() const {
  return Vec4(1 / this->direction.x, 1 / this->direction.y,
              1 / this->direction.z, 1);
}

void Ray::print() const {
  auto text = getPrint(nullptr);
  printf("%s\n", text.c_str());
}

void Ray::print(const char* name) const {
  auto text = getPrint(name);
  printf("%s\n", text.c_str());
}

std::string Ray::getPrint(const char* name) const {
  std::stringstream res;
  if (name) {
    res << name << "(";
  } else {
    res << "Ray(";
  }
  res << std::fixed << std::setprecision(4);
  res << origin.getPrint("origin") << ", " << direction.getPrint("direction")
      << ")";
  return res.str();
}

}  // Namespace Tyra

// Added by TyraX: tyra::helper - traceLine() and the world it traces against.
//
// `tyra::helper` (see ray.hpp) is a game-side convenience namespace, deliberately
// NOT nested in `Tyra` and deliberately lowercase. It is where the traces and
// sweeps live; `tyra::debug` next door holds what compiles away.
//
// The registered world is file-static on purpose: it is per-scene state, not a
// setting a caller owns, and the engine re-registers it on every scene load (a
// stale provider is a tracer shooting through the next level's walls).
namespace tyra {
namespace helper {
namespace {

TraceCountFn g_rayCount = nullptr;
TraceBoxFn g_rayBox = nullptr;
TraceTerrainFn g_rayTerrain = nullptr;

// 8 bisections over a 0.5 m step narrow a crossing to ~2 mm - finer than a
// bullet's meaningful precision, for 8 heightmap reads.
constexpr int kTerrainBisections = 8;

// The face the hit point sits on: the axis with the smallest gap to one of the
// box's own faces. Cheaper than tracking which slab the slab test entered
// through, and it cannot disagree with the point it is handed.
Tyra::Vec4 boxNormalAt(const Tyra::Vec4& p, const Tyra::Vec4& mn,
                       const Tyra::Vec4& mx) {
  const float dxMin = p.x - mn.x, dxMax = mx.x - p.x;
  const float dyMin = p.y - mn.y, dyMax = mx.y - p.y;
  const float dzMin = p.z - mn.z, dzMax = mx.z - p.z;
  float best = dxMin;
  int axis = 0;
  float sign = -1.0F;
  if (dxMax < best) { best = dxMax; axis = 0; sign = 1.0F; }
  if (dyMin < best) { best = dyMin; axis = 1; sign = -1.0F; }
  if (dyMax < best) { best = dyMax; axis = 1; sign = 1.0F; }
  if (dzMin < best) { best = dzMin; axis = 2; sign = -1.0F; }
  if (dzMax < best) { best = dzMax; axis = 2; sign = 1.0F; }
  return Tyra::Vec4(axis == 0 ? sign : 0.0F, axis == 1 ? sign : 0.0F,
                    axis == 2 ? sign : 0.0F, 1.0F);
}

// Below-terrain with an epsilon: an origin exactly ON the ground (a muzzle
// resting on a floor) must not read as already buried, or every shot fired
// level with the floor would "hit" it at t = 0.
inline bool underTerrain(float x, float y, float z) {
  return y < g_rayTerrain(x, z) - 1e-4F;
}

}  // namespace

void setTraceWorld(TraceCountFn countFn, TraceBoxFn boxFn,
                   TraceTerrainFn terrainFn) {
  g_rayCount = countFn;
  g_rayBox = boxFn;
  g_rayTerrain = terrainFn;
}

TraceHit traceLine(const Tyra::Vec4& start, const Tyra::Vec4& end) {
  TraceHit result;
  result.point = start;
  result.normal = Tyra::Vec4(0.0F, 1.0F, 0.0F, 1.0F);
  if (!g_rayCount || !g_rayBox || !g_rayTerrain) return result;

  const Tyra::Vec4 delta = end - start;
  const float length =
      (float)sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  // A zero-length segment (start == end) has no direction and no distance, and
  // a tracer reporting a hit there would report one every frame at the muzzle.
  if (length < 1e-5F) return result;

  const Tyra::Ray ray(start, delta / length);
  float best = length;
  Tyra::Vec4 hitMin, hitMax;

  // Boxes first: they are the cheap test and the common hit, and bounding the
  // terrain march by the nearest box keeps the common case (a wall in front of
  // the player) short.
  const int count = g_rayCount();
  for (int i = 0; i < count; ++i) {
    if (!g_rayBox(i, hitMin, hitMax)) continue;
    float t = -1.0F;
    if (!ray.intersectBox(hitMin, hitMax, &t)) continue;
    if (t < 0.0F || t >= best) continue;
    best = t;
    result.hit = true;
    result.terrain = false;
    result.objectIndex = i;
    result.t = t;
    result.point = ray.at(t);
    result.normal = boxNormalAt(result.point, hitMin, hitMax);
  }

  // Terrain: march, then bisect the crossing step. Marching (rather than
  // solving the bilinear surface) is what the flow-graph Raycast node does, so
  // the two never disagree about where the ground is.
  bool originUnder = underTerrain(start.x, start.y, start.z);
  float prev = 0.0F;
  bool prevUnder = originUnder;
  for (float t = kTraceTerrainStep; t <= best;
       t += kTraceTerrainStep) {
    const float f = t / length;
    const bool nowUnder =
        underTerrain(start.x + delta.x * f, start.y + delta.y * f,
                     start.z + delta.z * f);
    if (nowUnder && !prevUnder) {
      float lo = prev, hi = t;
      for (int k = 0; k < kTerrainBisections; ++k) {
        const float mid = (lo + hi) * 0.5F;
        const float mf = mid / length;
        if (underTerrain(start.x + delta.x * mf, start.y + delta.y * mf,
                         start.z + delta.z * mf)) {
          hi = mid;
        } else {
          lo = mid;
        }
      }
      result.hit = true;
      result.terrain = true;
      result.objectIndex = -1;
      result.t = hi;
      result.point = start + delta * (hi / length);
      result.normal = Tyra::Vec4(0.0F, 1.0F, 0.0F, 1.0F);
      break;
    }
    prevUnder = nowUnder;
    prev = t;
  }
  // An origin already under the terrain (a body in a trench, a spawn inside a
  // hill) hits it at zero length rather than reporting a miss the caller would
  // read as "clear line of fire".
  if (!result.hit && originUnder) {
    result.hit = true;
    result.terrain = true;
    result.t = 0.0F;
    result.point = start;
    result.normal = Tyra::Vec4(0.0F, 1.0F, 0.0F, 1.0F);
  }

  return result;
}
}  // namespace helper
}  // namespace tyra