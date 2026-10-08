/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022-2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# Wellington Carvalho <wellcoj@gmail.com>
*/

#include "math/vec4.hpp"
#include <string>

#pragma once

namespace Tyra {

/**
 * Class for raycasting.
 */
class Ray {
 public:
  /** @brief Constructor create a ampty ray */
  Ray();

  /**
   * @param origin The origin of the ray
   * @param direction The direction of the ray (normalized Vec4)
   */
  Ray(const Vec4& origin, const Vec4& direction);
  ~Ray();

  Vec4 origin;
  Vec4 direction;

  /**
   * @return Vec4 that is a given distance along this Ray
   * @param t - the distance along the Ray to retrieve a position for.
   */
  Vec4 at(const float& t) const;

  /** @return Distance from the Vec4 point to the origin */
  float distanceToPoint(const Vec4& point) const;

  /**
   * @param minCorner - pointer to min corner position. (bottom left)
   * @param maxCorner - pointer to max corner position. (top right)
   * @param outputDistance - optional - distance the box to intersect with.
   * @return Vec4 point of intersection
   */
  bool intersectBox(const Vec4& minCorner, const Vec4& maxCorner,
                    float* outputDistance = nullptr) const;

  /**
   * Added by TyraX. Moller-Trumbore ray/triangle test, both faces.
   * @param a,b,c triangle corners
   * @param outputDistance optional - distance along the ray to the hit.
   * @return true when the ray hits the triangle
   */
  bool intersectTriangle(const Vec4& a, const Vec4& b, const Vec4& c,
                         float* outputDistance = nullptr) const;

  /** Returns inverse direction */
  Vec4 invDir() const;

  void print() const;
  void print(const char* name) const;
  void print(const std::string& name) const { print(name.c_str()); }
  std::string getPrint(const char* name = nullptr) const;
};

}  // namespace Tyra

/**
 * Added by TyraX: trace and sweep helpers.
 *
 * `Tyra::helper` is where the helpers that SHIP live - traces now, sweeps and
 * overlap tests next. Its neighbour `Tyra::debug` (debug/debug_draw.hpp) holds
 * everything a release build strips out. Keep the two apart: one is in the
 * binary, one is not (AGENTS.md).
 */
namespace Tyra {
namespace helper {

/** The world behind a trace.
 *
 * traceLine() needs to know what is out there, and a scene keeps that in
 * engine-side tables the physics library has no business knowing about. The
 * engine registers the CURRENT scene's collidables at scene load, so a
 * bullet, a bot's line of sight and the walker all trace the same solids -
 * the alternative (each caller walking ctx.objects itself) is how bullets
 * end up going through collision: none decals.
 *
 * countFn: how many collidables the scene has. boxFn(index, min, max): the
 * collidable's world-space box, false for "not collidable" (markers,
 * lights, collision: none - the engine applies its own skip list here, so
 * callers never have to). terrainFn(x, z): the terrain height under a point.
 *
 * Registering null providers disables tracing: traceLine() then misses
 * everything rather than inventing a floor, because a phantom ground plane
 * makes a tracer silently agree with a world that is not there.
 */
using TraceCountFn = int (*)();
using TraceBoxFn = bool (*)(int index, Vec4& minCorner, Vec4& maxCorner);
using TraceTerrainFn = float (*)(float x, float z);
void setTraceWorld(TraceCountFn countFn, TraceBoxFn boxFn,
                   TraceTerrainFn terrainFn);

/** One trace/sweep result. Shared by traceLine() and the sweeps to come, so
 *  a caller can write one "did I hit" branch over both. */
struct TraceHit {
  bool hit = false;
  float t = 0.0F;        // distance from the start along the trace
  Vec4 point;            // world-space contact point (start when !hit)
  Vec4 normal;           // surface normal; terrain reports straight up
  int objectIndex = -1;  // the object hit, -1 = terrain or nothing
  bool terrain = false;
};

constexpr float kTraceTerrainStep = 0.5F;

/** One segment traced against the registered world.
 *
 * A segment (start -> end) rather than an infinite ray: a hit is only
 * interesting up to where the shot was going, and the muzzle-to-target form
 * removes a length argument from every call site. `t` comes back as the
 * DISTANCE from `start`, so point == start + (end - start) * (t / length).
 *
 * Boxes are slab-tested (one Ray::intersectBox per collidable, nearest
 * wins). The terrain is marched in kTraceTerrainStep steps and bisected
 * eight times - the same march the flow-graph Raycast node uses, so both
 * agree on where the ground is. COST, stated because it runs on a PS2: one
 * trace is O(objects) box tests + maxDist / 0.5 heightmap samples - fine per
 * shot (a fire rate, a batch of a dozen shooters), too much per frame per
 * bot, which wants a coarse variant rather than this.
 */
TraceHit traceLine(const Vec4& start, const Vec4& end);

}  // namespace helper
}  // namespace Tyra