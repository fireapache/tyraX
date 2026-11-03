/*
# _____        ____   ___
#   |     \/   ____| |___|
#   |     |   |   \  |   |
#-----------------------------------------------------------------------
# Copyright 2022, tyra - https://github.com/h4570/tyra
# Licensed under Apache License 2.0
# TyraX
#-----------------------------------------------------------------------
*/

// Added by TyraX: the compact stream with the engine's types - Vec4 and
// Color in and out of a writer or a view, and a StaPipBag's float arrays as a
// packer's source (docs/compact-vertex-format.md, "The stream"). The stream
// itself (stapip_compact_stream.hpp) takes plain floats so a host can run it.
#pragma once

#include "math/vec4.hpp"
#include "renderer/models/color.hpp"
#include "renderer/3d/pipeline/static/core/bag/stapip_bag.hpp"
#include "renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp"

namespace Tyra {
namespace Compact {

/** A float bag's arrays (its own layout comes from StaPipCompactLayout::forBag). */
inline StaPipCompactSource sourceOf(const StaPipBag& bag) {
  StaPipCompactSource source;
  source.count = bag.count;
  source.positions = bag.vertices != nullptr ? bag.vertices->xyzw : nullptr;
  source.sts = bag.texture != nullptr && bag.texture->coordinates != nullptr
                   ? bag.texture->coordinates->xyzw
                   : nullptr;
  source.colors = bag.color != nullptr && bag.color->many != nullptr
                      ? bag.color->many->rgba
                      : nullptr;
  source.normals = bag.lighting != nullptr && bag.lighting->normals != nullptr
                       ? bag.lighting->normals->xyzw
                       : nullptr;
  return source;
}

// Writing one vertex's attributes from the engine's types.
inline void position(StaPipCompactWriter& w, const Vec4& p) { w.position(p.xyzw); }
inline void st(StaPipCompactWriter& w, const Vec4& uv) { w.st(uv.x, uv.y); }
inline void color(StaPipCompactWriter& w, const Color& c) { w.color(c.rgba); }
inline void normal(StaPipCompactWriter& w, const Vec4& n) { w.normal(n.xyzw); }
inline void bbParams(StaPipCompactWriter& w, const Vec4& m) {
  w.bbParams(m.xyzw);
}

// Reading them back.
inline Vec4 position(const StaPipCompactView& v, u32 i) {
  Vec4 out;
  v.position(i, out.xyzw);
  return out;
}
/** u, v, 1, 0 - the shape of a bag's ST array. */
inline Vec4 st(const StaPipCompactView& v, u32 i) {
  float uv[2];
  v.st(i, uv);
  return Vec4(uv[0], uv[1], 1.0F, 0.0F);
}
inline Color color(const StaPipCompactView& v, u32 i) {
  Color out;
  v.color(i, out.rgba);
  return out;
}
/** x, y, z, 0. */
inline Vec4 normal(const StaPipCompactView& v, u32 i) {
  float n[3];
  v.normal(i, n);
  return Vec4(n[0], n[1], n[2], 0.0F);
}
/** Positions first .. first + n - 1 into Vec4s: aligned, so the VU0 path. */
inline void positions(const StaPipCompactView& v, u32 first, u32 n, Vec4* out) {
  v.positions(first, n, out->xyzw);
}

}  // namespace Compact
}  // namespace Tyra
