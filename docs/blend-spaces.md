# 2D blend spaces — direction × speed, and the 2D Animation Blend Tool

A **blend space** places some of an animated model's clips on a compass of
direction and speed, and blends the clips around a game's input every frame
instead of playing one. A soldier given a jog at 0°, strafes at ±90°, a
back-pedal at 180° and an idle at the centre turns smoothly from one to the
next as its move direction swings around where it looks - no snap at a sector
line. It is TyraX's counterpart of Unreal Engine's **Blend Space (2D)**, drawn
as a compass seen from above (Unity's "2D Freeform Directional" layout).

| Unreal Engine | TyraX |
|---|---|
| Blend Space asset | `<name>.blendspace` beside the model in `res/models/` |
| Horizontal axis "Direction", Wrap Input | the compass angle - always wraps; 0 = where the object faces, **+ = its right** |
| Vertical axis "Speed" | the compass radius, m/s |
| Samples + triangulation | samples on speed **rings**, at most 3 clips blended (below) |
| Sync groups | **sync** samples share one phase; **own-time** samples (an idle) keep their clock |
| Axis "Smoothing Time" | `smoothing`: the input POINT eases toward a new input |
| — | a **blend LOD**: past a distance, only the best-fitting sample |

## Using the tool

*Tools > 2D Animation Blend Tool...* (the Asset Browser opens a `.blendspace`
on double-click; an animated model's *More actions* has *2D Animation Blend
Tool*). It is laid out like the Ragdoll Editor (a property's or a section's
help appears when the cursor rests on its label a moment):

- **Toolbar** — *Model* (an animated model) and beside it *Blend space* (that
  model's blend spaces: every `.blendspace` under `res/` naming it; picking a
  model opens its first one), *Save* (Ctrl+S), *Revert*, *Undo* / *Redo*,
  *New...* (an empty blend space for the picked model and the name a game will
  ask for: saved as `<model>_<name>.blendspace`), *Delete* (removes the
  blend space's file after a confirmation - it cannot be undone; the next
  build drops it from the model and the tool moves on to the model's next
  one), then *Play* / *Pause*, the preview speed, *Show* (mesh, wireframe, the
  moving floor, the weights on the compass) and *Reset view*. Drop a model from
  the Asset Browser anywhere on the window to pick it, or a `.blendspace` to
  open it with its model.
- **Left** — the *Samples* (clip, direction, speed, own time) and the model's
  *Clips* as the game names them (imports and clip edits applied). Drag a clip
  onto the compass to place it.
- **Centre** — the preview: the model blending live at the preview point, the
  floor lines moving under it at the speed the body moves and the point's
  direction (a planted foot that slides drifts across them; right-drag orbits,
  middle-drag pans, the wheel zooms). Under it the **compass**: up is where the model looks,
  rings every 0.5 m/s, spokes every 45°, the samples as dots (the selected one
  in the warning colour) joined by the interpolation's dashed edges, and the
  green **preview point** (the outline beside it is the smoothed point the
  blend actually reads). Drag a sample to move it - it snaps to 45° and
  0.5 m/s, Shift drags freely; drag on empty space (or Ctrl+drag anywhere) to
  move the preview point; Delete removes the selected sample. A blend space
  laid out as a **grid** (see [The grid layout](#the-grid-layout)) shows the
  grid instead: direction across, speed up, a dot on every empty
  intersection; a sample snaps to an intersection, a clip dropped on a taken
  one replaces its clip, and a sample dragged onto another swaps with it.
- **Right** — the selected sample (*Clip*, *Direction*, *Speed*, *Time*: sync
  or own time), or with nothing selected the blend space (*Name in game*,
  *Stride*, *Smoothing*, *LOD fade*, and every problem `check` finds); under
  either, the preview point, the *Move speed* and the weights as bars.

**Move speed.** The compass ends just past the fastest ring, but a game feeds
real speeds - a fighter runs at 4.6 m/s on a 0.5 m/s ring - and the engine
clamps the weights to the ring there. So on or past the ring the preview's
body moves at the *Move speed* (the phase by the Stride, and the floor), by
default the speed the clips were authored at (the Stride over their cycle:
3.17 m/s for the combatent's 2.116 m in 0.667 s) - set 4.6 to see a run.
Inside the ring it moves at the point's share of the ring of that speed - a
point 87 % of the way to the jog moves at 87 % of it - so dragging out from
the idle speeds the stride up smoothly and never jumps. Right-click the field
to reset it. Without the rule a stride sample previewed at its 0.5 m/s plays
one cycle every 4.2 s.

## The asset

A `.blendspace` is JSON, editor-only - the build never ships it (the asset
bake skips it like `.ragdoll`):

```json
{
  "asset": "tyrax.blendspace",
  "model": "res/models/combatent.glb",
  "name": "locomotion",
  "stride": 2.11644,
  "smoothing": 0.15,
  "lodFade": 0.15,
  "samples": [
    {"clip": "idle", "direction": 0, "speed": 0, "time": "own"},
    {"clip": "jog", "direction": 0, "speed": 0.5, "time": "sync"},
    {"clip": "strafe_r", "direction": 90, "speed": 0.5, "time": "sync"}
  ]
}
```

| Key | What |
|---|---|
| `model` | the animated model whose clips the samples name |
| `name` | what a game asks for (`setAnimationMode`), at most 31 characters |
| `stride` | metres one phase cycle covers when the ENGINE plays the phase: it advances by the distance the input speed covers, so planted feet stay planted. 0 = by the clips' own time (Unreal's sync-group rate) |
| `smoothing` | seconds: the time constant the input point eases with (0 = none) |
| `lodFade` | seconds: the crossfade of a blend-LOD change |
| `samples[]` | `clip` (its name in game), `direction` (degrees, + right), `speed` (m/s, 0 = the centre), `time` (`sync` / `own`) |
| `layout`, `grid` | only on a grid: `"layout": "grid"` and `"grid": {"columns": 9, "rows": 2, "direction": [-180, 180], "speed": [0, 0.5]}` - the lines and the values at the first and last of each. A grid's directions are stored as placed (-180 and 180 are two columns); a radial asset's file has neither key and reads exactly as before |

Every build resolves each blend space's clip names against the model's clips
as the game sees them - imports and clip edits applied, so a renamed clip is
named by its new name - and writes it into the model's `.tskl` as an optional
`BLSP` trailer (an engine without blend spaces stops reading before it),
followed - only when some blend space is a grid - by a `BLGR` trailer with
each grid's lines. An asset with a problem - a clip the model lacks (a clip
renamed in the Animation Editor, say), two centre samples, two samples on one
point, more than 32 samples, no name; on a grid, fewer than 2 lines either
way, an empty range, a sample off the intersections or two on one -
**fails the build**, naming the asset and the sample: a quietly dropped
blend space would put every body back on whatever the game does without it.

## How it blends

Samples sit on **rings** of equal speed (speeds within 0.05 m/s are one
ring); a sample at speed 0 is the **centre** and counts for every direction.
For an input (direction, speed):

- on a ring, the two angular neighbours of the direction share it by angle -
  at 22.5° between a 0° and a 45° sample, half each;
- between the centre and the first ring, the speed fraction mixes the centre
  with that ring's pair;
- between two rings, the four corners of the cell are split along a diagonal
  into two triangles, and the point takes the barycentric weights of the one
  it is in (Unreal's triangulation) - continuous, never more than 3 clips, as
  long as both rings use the same directions (`check` warns otherwise);
- below the first ring without a centre sample, that ring alone; past the
  last ring, the last ring alone. A single sample plays alone.

Weights under 0.01 are dropped and the rest renormalised. The one function is
`vendor/tyra/engine/inc/renderer/3d/mesh/dynamic/blend_space_weights.hpp`,
header-only: the engine, the tool's preview and the CLI all include it, so
they cannot disagree; `tools/verify-blendspace.cpp` pins it on the host
(`g++ -std=c++20 -O1 -Wall -Wextra -o verify-blendspace tools/verify-blendspace.cpp`).

The engine folds the chosen samples' poses one by one with the crossfade's own
rule (translation and scale lerped, rotations nlerped on the short arc), and
the held layer (`blendAnimation`, a lean) rides on top as always. A motorized
ragdoll's targets (`animClipModel`) see the blend too.

The tool's preview poses through `animmerge::poseMatricesBlend`, the editor
twin of that fold, and its key sampler slerps rotation keys on the short arc
like the engine's. That matters: an exporter may store a key with the
opposite sign mid-clip (q and -q are one rotation - Blender does it on the
combatent's thighs), and a plain component lerp across such a pair passes
near zero and spins the joint for a frame. `tools/verify-anim-sampling.cpp`
pins it, and given a model it sweeps every clip for a joint that jumps (the
build line is in its header; `verify-anim-sampling [model.glb]`).

**Smoothing.** The engine does not blend the raw input: it eases a point in
the compass plane toward it, with the asset's `smoothing`. Smoothing the
POINT (not each axis) means a reversal - strafe left to strafe right - passes
through the centre, the idle, instead of spinning round the ring. The point
chases the input clamped to the fastest ring: a fighter running at 4.6 m/s on
a 0.5 m/s ring would otherwise put it nine rings out, and the smoothing would
cross the whole idle-to-stride band in one frame.

**Time.** Sync samples share one phase 0..1 and are each sampled at that
fraction of their length; own-time samples loop on their own clock. The phase
is either **held by the game** (it passes one every frame - Gunner holds its
strides at the fraction its distance travelled says) or **played by the
engine** (the stride's distance, else the shown sync samples' weighted rate).

## The grid layout

The blend space's **Layout** (the tool's details) is the radial compass above
or a 2D **grid**: direction across, speed up, `columns` vertical and `rows`
horizontal lines (at least 2 each - the smallest grid is a square), evenly
spaced between the values at the first and the last; a clip sits on an
intersection. The radial compass is the special case of a grid with 2
horizontal lines, the centre's clip along the whole bottom one.

- The point's **cell** blends its four corners exactly as a compass cell
  does (the same fractions, the same diagonal split when the corners name
  four different clips), so never more than 3 clips blend.
- An **empty intersection** takes the nearest clip on its row (ties: the
  lower column); a row with no clip, the nearest row's (ties: the lower row).
  One idle on the bottom row blends like the idle along all of it.
- A clip on several intersections counts once (the idle along the bottom).
- Outside the grid the point is clamped to its edges; the smoothing, the
  phase, the stride and the blend LOD are the compass's.

`Tyra::blendspace::computeGrid` is the grid's function, beside `compute`, and
`weightsFor` picks by layout - the engine, the tool and the CLI all ask it.
**A grid laid out like a radial asset blends exactly like it**:
`tools/verify-blendspace.cpp` sweeps the combatent's compass against its 9 x 2
grid (every 0.25°, every 0.01 m/s) and two even rings against their 5 x 3 grid,
every clip's weight within 1e-5; Gunner's `combatent-blend-grid` gym holds the
radial and the grid asset at the same inputs on PCSX2 and gets the same
weights and key vertices.

The Layout is a sliding **Radial | Grid** switch, and switching converts the
asset so it blends exactly as before: a radial one becomes the grid that
blends like it - columns over -180..180° at the coarsest step every direction
sits on, rows from 0 to the fastest ring likewise, the centre's clip along
the bottom row, the 180° sample at -180° too - and a grid goes back on the
compass with its bottom row's clip as the centre. A side the asset cannot
switch to without losing a clip or blending differently is **greyed**, and
resting the cursor on it says why and what to change: for Grid, rings not
evenly spaced from 0, a ring missing a direction, more than 32 samples; for
Radial (`blendspace::canRadial`), more than one clip on the bottom (0 m/s)
row - the compass's centre plays one clip in every direction - an empty spot
on a moving row, different clips at -180° and 180°, directions not
-180..180°, an empty row between two others.
*Save a grid version* (a radial asset's details) writes that grid beside the
asset as `<asset>_grid.blendspace`, named `<name>_grid` in game, and leaves
the radial one as it is. *Lines*, *Directions* and *Speeds* edit a grid's
lines; the samples keep their place, and one left off the new intersections
is flagged.

## In a game

`inc/scripts/script.hpp` (docs/animated-models.md, "Scripts"):

```cpp
// into the model's blend space "locomotion", crossfading 0.15 s
if (!setAnimationMode(ctx, body, ANIM_MODE_BLEND_SPACE, "locomotion", 0.15F)) {
  // the model has no such blend space: keep playing clips
}
// every frame: degrees from where it FACES (+ = right), m/s, and the phase
setBlendSpaceInput(ctx, body, moveDirDeg, moveSpeed, strideFraction);
setBlendSpaceInput(ctx, body, moveDirDeg, moveSpeed);  // or: the engine plays it

int clips[3];
float weights[3];
if (blendSpaceWeights(ctx, body, clips, weights)) { /* what it showed */ }
```

`playAnimation` and `seekAnimation` take the object back to clip mode with
their own fade (a death clip needs no mode call first); `setAnimationMode(...,
ANIM_MODE_CLIP)` does it without a clip. A switch either way crossfades from a
frozen copy of the pose that was showing. While any such crossfade is in
flight the weights `blendSpaceWeights` reports sum to less than 1: the rest
of the pose is the one it fades from.

## Blend LOD

Past a distance from the **nearest** camera (both views in split screen) an
object in blend-space mode shows only its best-fitting sample - one clip
sampled instead of up to three, and a pose other bodies can share again. The
distance has a 10 % band (a blending body goes single past 1.1 x the distance,
blends again inside 0.9 x), so a body holding at the distance does not flip
every frame. A change crossfades over the asset's `lodFade`; the sample shown
alone changes only when another outweighs it by 0.1, so a body moving along a
boundary does not flip either. The distance is *Preferences > Graphics > Blend LOD
distance* (0 = always blend), overridden per object in *Properties > Distance
and LOD* (`blendLod` in the object's JSON, -1 = the preference). The decision
is made once per frame in the first render pass; a game rendering its own
split screen sets `blendLodCamera2` / `blendLodCamera2On` around it so the
second view counts (Gunner's `src/terrain_game.cpp` does).

## Cost

One clip sample is ~0.09 ms on a 37-joint character (docs/animated-models.md,
"What a pose costs"); the matrix products and the skin are paid once whatever
the clip count, so a 3-clip blend costs up to ~0.18 ms more per re-pose than
one clip. Per instance: three sample layers (their key cursors are reserved
for the model's longest clip at construction - a switch never allocates) and
one frozen pose (~1.7 KB for 41 nodes), also sized at construction for a
model that has blend spaces.

## Command line

```
tyrax-editor --blendspace <projectDir> check <asset.blendspace>
tyrax-editor --blendspace <projectDir> weights <asset.blendspace> --at <dirDeg>,<speed>
```

`check` prints the centre and the rings (a grid: its lines and each row's
clips), every error and warning against the model's clips as the game sees
them, and exits 1 on an error. `weights`
prints the samples the engine would blend at that input and their weights:

```
$ tyrax-editor --blendspace . weights res/models/combatent_locomotion.blendspace --at 30,0.25
idle 0.500  jog_fr 0.333  jog 0.167
```

## Limits of this first version

- Rings with different directions blend with a seam where a sample is
  crossed (the diagonal split is exact only for matching rings).
- No Flow Graph node yet: scripts call the three functions.
- TyraX's own third-person player (`drivePlayerAnim`) still picks its
  locomotion clip by sector; moving it onto a blend space is in
  [backlog.md](backlog.md).
- The blend LOD chooses by distance; by on-screen size is in the backlog.
