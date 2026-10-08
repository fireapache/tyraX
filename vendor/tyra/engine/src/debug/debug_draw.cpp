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

// Added by TyraX: the debug draw queue (inc/debug/debug_draw.hpp). The whole
// translation unit is empty in a release build, so a debug-only library costs a
// release ELF nothing.

#ifndef NDEBUG

#include "debug/debug_draw.hpp"

namespace Tyra {
namespace DebugDraw {
namespace {

struct QueuedLine {
  Vec4 from;
  Vec4 to;
  Color color;
  float remaining;  // seconds left; 0 = this frame only
};

// A plain fixed array: a debug helper that allocates, or that grows while a
// frame is being built, is a worse problem than the one it is helping with.
QueuedLine g_lines[kMaxLines];
int g_count = 0;

// Which line a full queue drops: the oldest PERSISTENT one first (a stale tracer
// nobody is looking at any more), then the oldest one-shot. Without this, a
// per-frame firehose of one-shots pushes every timed line out within a second,
// which is exactly the line someone queued to look at.
int evictionSlot() {
  for (int i = 0; i < g_count; ++i) {
    if (g_lines[i].remaining > 0.0F) return i;
  }
  return 0;
}

}  // namespace

void line(const Vec4& from, const Vec4& to, const Color& color, float time) {
  int slot;
  if (g_count < kMaxLines) {
    slot = g_count++;
  } else {
    slot = evictionSlot();
    g_lines[slot] = g_lines[kMaxLines - 1];
    --g_count;
  }
  g_lines[slot].from = from;
  g_lines[slot].to = to;
  g_lines[slot].color = color;
  g_lines[slot].remaining = time > 0.0F ? time : 0.0F;
}

void frame(Renderer& renderer, float dtSeconds) {
  // Draw first, expire after: a line queued this frame is drawn this frame
  // whether it is a one-shot or a timed one, so `time` counts from the frame it
  // was queued rather than from the next one.
  for (int i = 0; i < g_count; ++i) {
    renderer.renderer3D.utility.drawLine(g_lines[i].from, g_lines[i].to,
                                        g_lines[i].color);
  }

  int out = 0;
  for (int i = 0; i < g_count; ++i) {
    if (g_lines[i].remaining <= 0.0F) continue;  // one-shot: drawn, gone
    g_lines[i].remaining -= dtSeconds;
    if (g_lines[i].remaining > 0.0F) g_lines[out++] = g_lines[i];
  }
  g_count = out;
}

void clear() { g_count = 0; }

int queued() { return g_count; }

}  // namespace DebugDraw
}  // namespace Tyra

#endif  // NDEBUG