# Debug draw

The things you want to SEE while you are making a game: the ray you just traced,
the path a character took, the volume a rule applies to. This is the library for
that — `Tyra::DebugDraw`, one call for a line and room for the shapes that come
after it.

```cpp
// A ray that missed, so you can see where it went.
Tyra::DebugDraw::line(from, to, Tyra::Color(0.2F, 0.4F, 1.0F));
// A ray that hit: green, and it stops AT the hit instead of at max distance.
Tyra::DebugDraw::line(muzzle, hit.point, Tyra::Color(0.0F, 1.0F, 0.0F));
// The same line, still there for a second - for a tracer you want to read.
Tyra::DebugDraw::line(a, b, Tyra::Color(1.0F, 0.8F, 0.0F), 1.0F);
```

`from`/`to` are world-space `Tyra::Vec4`, the colour defaults to green, and
`time` is in **seconds** (`0` = this frame only).

## Debug builds only

The whole library is `#ifndef NDEBUG`, header and implementation both. In a
release build the header declares no-op overloads, so **a call site needs no
`#ifdef`**:

```cpp
Tyra::DebugDraw::line(a, b);   // debug: drawn     release: compiles to nothing
```

Debug drawing is not a small tax (the utility lines are depth-test free, one
submission each) and it must never become a shipping cost by accident. A release
ELF contains none of it.

## Who submits the lines

The engine calls `DebugDraw::frame(renderer, dtSeconds)` once per rendered
frame, **after the 3D pass and before the HUD sprites** — the utility lines
ignore the depth buffer, so drawn earlier the world paints over them, drawn later
they paint over the text. A game never calls `frame()`; it only queues.

`frame()` does two things: submits everything queued, then expires the timed
lines against `dtSeconds`. Because the queue holds a fixed 128 lines, an
overflow evicts the oldest **persistent** line first (a stale tracer nobody is
looking at) and only then the oldest one-shot — otherwise a per-frame firehose
of one-shots would push out every timed line within a second.

`clear()` empties it, `queued()` reports the depth — for an overlay that prints
it.

## Cost

One line is one `renderer3D.utility.drawLine`: a small immediate-mode draw, no
allocation, no queue growth. Hundreds per frame is fine for a debug build; a
shipping build has none of them.

## Known limits

- **Split screen** draws the queue in the view that is current when `frame()`
  runs (the first half). Per-half drawing needs a call inside each pass of
  `renderScene`, which is not done.
- **Mesh-accurate tracing is not drawn here** — a tracer draws the segment, not
  the silhouette it tested ([game-side-collision.md](game-side-collision.md)).

## Where it lives

| Layer | Code |
|---|---|
| Header (debug + release overloads) | `vendor/tyra/engine/inc/debug/debug_draw.hpp` |
| Queue + submission | `vendor/tyra/engine/src/debug/debug_draw.cpp` |
| The one engine call site | `Tyra::DebugDraw::frame` in the frame loop (`src/game_templates.inc`, `#ifndef NDEBUG`) |
| The lines it submits through | `Renderer3DUtility::drawLine` ([the debug renderer](collision-boxes.md)) |