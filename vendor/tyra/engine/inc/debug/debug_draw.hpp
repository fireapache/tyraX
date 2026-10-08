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

// Added by TyraX: the debug draw helpers - the things a game wants to SEE while
// it is being made. Lines for rays and trajectories, and room for the shapes
// that come next.
//
// The whole header compiles to nothing in a release build (#ifndef NDEBUG), and
// so does the queue behind it: debug drawing on a PS2 is not a small tax
// (renderer3D.utility draws unlit, un-z-tested primitives, one draw call per
// line) and it must never be a shipping cost by accident. A call site in a
// game should therefore be written as
//
//     Tyra::DebugDraw::line(a, b);              // debug build: drawn
//                                                 // release build: not even called
//
// with no #ifdef at the call site - DebugDraw::line below is a no-op overload
// in release, so the same source compiles both ways.
//
// TIMING: the engine calls frame() once per rendered frame after the 3D pass
// (lines are depth-test free, so they must land on top of the world but under
// the HUD). frame() expires the timed lines and submits the survivors, which
// is why `time` is in seconds and the library owns no clock: it advances one
// from the frame dt it is handed.

#pragma once

#ifndef NDEBUG

#include "math/vec4.hpp"
#include "renderer/models/color.hpp"
#include "renderer/renderer.hpp"

namespace Tyra {

class Renderer;

namespace DebugDraw {

/** One frame's worth of lines. 128 is a PS2-sized budget on purpose: the queue
 *  is a fixed array, so a debug helper can never allocate or grow mid-frame.
 *  Overflow evicts the OLDEST timed line first, then the oldest of all - a
 *  debug tool that silently drops what it just drew is worse than useless. */
constexpr int kMaxLines = 128;

/** Queues a line in world space. `time` in SECONDS: 0 (the default) draws it
 *  for exactly the frame it was queued, a positive value keeps it around for
 *  that long (a tracer that outlives its frame, a failed ray drawn while you
 *  read the log). */
void line(const Vec4& from, const Vec4& to,
          const Color& color = Color(0.0F, 1.0F, 0.0F), float time = 0.0F);

/** Expires and submits. The engine calls this once per frame AFTER the 3D pass;
 *  a game does not call it. dtSeconds is the frame's real dt (a timed line is
 *  measured in wall-clock seconds, so a hitch does not make a 0.5 s tracer
 *  linger for half a minute of gameplay). */
void frame(Renderer& renderer, float dtSeconds);

/** Drops every queued line. For a game that wants the picture clean while it
 *  is not debugging anything. */
void clear();

/** Lines still queued - for a debug overlay that prints it. */
int queued();

}  // namespace DebugDraw
}  // namespace Tyra

#else  // NDEBUG - release: the calls disappear, the queue with them

#include "math/vec4.hpp"
#include "renderer/models/color.hpp"
#include "renderer/renderer.hpp"

namespace Tyra {
class Renderer;
namespace DebugDraw {

constexpr int kMaxLines = 0;

inline void line(const Vec4&, const Vec4&, const Color& = Color(0.0F, 1.0F, 0.0F),
                 float = 0.0F) {}
inline void frame(Renderer&, float) {}
inline void clear() {}
inline int queued() { return 0; }

}  // namespace DebugDraw
}  // namespace Tyra

#endif  // NDEBUG