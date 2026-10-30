# Text drawing — `Tyra::drawText`

A drawing call for strings that are only known while the game runs: an ammo
count, a timer, a score, a name that changes.

```cpp
// The magazine count, bottom-right. 0.93/0.95 is 93% across, 95% down.
Tyra::drawText(ctx.engine, 0.93F, 0.95F, ammoText);
// Red, for an empty magazine.
Tyra::drawText(ctx.engine, 0.93F, 0.95F, ammoText, Tyra::Color(220, 70, 70));
// Up for two seconds without re-asserting it every frame - a "ROUND OVER".
Tyra::drawText(ctx.engine, 0.0F, -0.6F, "ROUND OVER", Tyra::Color(255, 200, 80), 2.0F);
```

`x`, `y` are in **`[-1, 1]` with y pointing DOWN**, and the string is **centred**
on the point. So `(0,0)` is the middle of the screen, `(-1,-1)` its top-left and
`(1,1)` its bottom-right. Centred rather than top-left because a HUD wants "put
the ammo here", not "start drawing the ammo here and hope".

**"The screen" is the whole picture in every display mode**, including the
taller ones: the full-height PAL frame (512 rows) and 1080i (540). The glyphs are
2D sprites, and the 2D pass centres its 448-row sprite space in a taller raster
(see [ps2-viewport.md](ps2-viewport.md), "2D is authored against 448 lines"), so
`drawText` takes that `(height - 448) / 2` back off. Before it did, `(1,1)` sat
32 rows below the bottom of a PAL picture and a bottom-anchored readout simply
was not on screen.

`size` is a **fraction of screen height** (`0.04` is about 18 px at 448), never
pixels — a pixel HUD is illegible on half the display modes. `color` defaults to
opaque white and tints the glyphs. `font` indexes `FONTS` and defaults to the
project's first font.

## `time` works like `debug::drawLine`

`time` is in **seconds**: `0` (the default) draws the string for exactly the
frame it was called, and a positive value keeps it up that long **without the
game re-asserting it**. The same contract as
[`Tyra::debug::drawLine`](debug-draw.md), on purpose — one mental model covers
both:

```cpp
Tyra::debug::drawLine(a, b, colour, time);   // world space, debug builds only
Tyra::drawText(engine, x, y, s, colour, time); // screen space, always ships
```

So a HUD re-asserts every frame with `time = 0` (nothing is queued, nothing can
go stale), and a one-shot announcement passes a duration and can stop thinking
about it.

Re-queuing a string that is already up **refreshes its timer** instead of adding
a duplicate, so a HUD that calls this every frame costs one queue slot, not one
per frame. The queue is a fixed 32 entries — a text call can never allocate or
grow mid-frame — and an overflow evicts whichever entry is closest to expiry.

## Where it lives, and why not in the engine library

`drawText` is generated into the project's `inc/game_runtime.gen.hpp`, beside the
`drawFontText` helper it is built on. It is **not** in `vendor/tyra/engine`,
because it needs `FONTS` and the glyph atlas — per-project generated data
(`font_data.gen.hpp`) that the engine `.a` cannot see.

The state and the maths sit in `runtime_detail`, where `FONTS` is unqualified;
only a thin forwarder is published, in the real `::Tyra`. Opening a
`namespace Tyra` from inside `runtime_detail` would declare
`runtime_detail::Tyra` instead — a silent no-op at every call site rather than a
compile error.

## The atlas always exists

Glyphs come from the project's font atlas, and an atlas is baked only for fonts
something actually draws with. `drawText` draws strings no bake could know, so
`project::atlasFontIndices()` always claims the project's **first** font for it —
the same reason it already claims the save menu's font and a vehicle HUD's font.
Without that, a runtime text silently draws nothing, which reads as a broken
feature rather than as a missing asset.

Characters outside the baked set (ASCII today) draw nothing. That is a blank gap
in a string, never a crash.

## Cost

One `renderer2D` sprite per glyph, twice if the font has a shadow — so a
six-character readout is about twelve quads. The atlas is one small texture, and
it is shared: every glyph of a font reuses a single sprite and texture link, so a
HUD full of numbers costs no extra texture binds.

## Known limits

- **Split screen** draws every string at the same screen position; the
  coordinates are whole-screen, not per-view.
- **One string at a time, no wrapping, no alignment.** Centred, clipped to the
  atlas's character set.
- Icon tokens (`{{action:jump}}`) work, because `drawText` goes through the same
  `drawFontText` the menus use.

## Where it lives

| Layer | Code |
|---|---|
| The public forwarders | `src/game_templates.inc` → generated into `inc/game_runtime.gen.hpp` |
| State, maths, queue | `src/game_templates.inc`, `runtime_detail` |
| Glyph drawing it is built on | `drawFontText` / `fontTextWidth`, same file |
| The per-frame tick | `Tyra::drawTextFrame` next to `updateAndRenderDynTexts()` in `renderScene` |
| The atlas rule | `Project::atlasFontIndices`, `src/project.cpp` |

A game never calls `drawTextFrame` or `clearDrawTexts`; the frame does.