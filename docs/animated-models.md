# Animated models

TyraX imports skeletal `.glb` and `.fbx` models, previews their clips and bakes
them into the PS2-native `.tskl` format.

![The Animation Editor: clips with their usage, the preview with its transport and trim timeline, and the clip inspector with Used by.](img/animation-editor.png)

## Authoring

In Blender or another DCC tool:

- keep the rig under 256 bones and rigid mesh nodes;
- apply object scale before export;
- use one armature and triangulated meshes;
- give actions clear, unique names;
- use power-of-two textures;
- export `.glb` when possible because it carries textures with it.

Morph targets and procedural materials are not supported. FBX works, but its
external textures must be available during import.

## Import and place

Import it with the model importer ([model-import.md](model-import.md)) with
**Animated model** ticked, then place it with **Add object > Object > Animated
model...** (or drag it from the [Asset Browser](asset-browser.md)). The
inspector reports vertices, bones, clips and estimated runtime memory before you
build. The same `.glb`/`.fbx` can also be placed as a static **Model**.

![An animated model (the showcase keeper) selected in the scene: its Model field, clips, playback and LOD settings in Properties.](img/animated-model-properties.png)

**An untextured model's colour is its glTF `baseColorFactor`**, and the
viewport reads it the way the game does. That is worth saying because the two
disagreed until 1.47.0: the viewport carried only a mesh and a texture per
part, so a model with no texture at all — `examples/showcase`'s wobbler is teal
by its base colour and nothing else — was drawn in the scene light alone and
came out orange in the editor while the console drew it green.

Animated model properties include:

- start clip, autoplay, loop and speed;
- facing-axis correction;
- animation and mesh LOD overrides;
- the usual transform, draw distance, material and gameplay flags.

The importer writes the model's materials to a sibling `<name>.mtl`, which a
new Animated model entity wears automatically; edit it, or assign another `.mtl`,
to change the materials. Parts are matched by material name.

A **clip-only file** — a downloaded move with no useful mesh — is imported the
same way, but you put its animation on a character you already have rather than
placing it. See [animation-import.md](animation-import.md).

## Edit clips

Open **Tools > Animation Editor**. The model is the dropdown in the header,
beside the project's `source -> play fps`. The left column lists the model's
clips - game name, duration, the source name, and who plays it (`walk · 2
players`, or an amber `not used`) - then **Borrowed from other files**. The
preview is the centre: Shaded / Wireframe, **Bones** (the skeleton drawn over
the model - joints as dots, each bone a line to its parent, at the pose on
screen; a model without a skin has nothing to draw), the preview light and
*Reset view* on top (right-drag to orbit, middle-drag to pan, wheel to zoom), and under it the
transport: back to start, play/pause, loop, the time as the game plays it,
a preview speed (0.5x / 1x / 2x) and the **timeline** in source seconds. Its
green handles are the trim: drag them, and the cut parts are hatched; drag
anywhere else to scrub.

Clip edits are non-destructive:

- **Name in game** gives a short runtime name.
- **Time scale** fixes one clip that plays too quickly or slowly.
- **Trim start/end** cuts unwanted frames.
- **In place** removes root translation for locomotion.
- **Loop by default** sets the initial object behaviour.

**Used by** under them lists every object that plays the clip - a Player's
locomotion slots, an object's start clip, *Animation* nodes in the object's own
graph; click one to select it. An empty Idle or Walk slot plays the model's
first clip, and counts as using it (`(first clip)`). A clip nothing plays gets a
warning card and, when the scene has a Player with this model, **Set as Walk on
<player>** (one undoable edit). The footer shows the clip's keys and channels and
**Reset clip** (back to exactly what the file contains).

Project Preferences also has a global animation FPS for exports whose authored
rate was interpreted incorrectly. Fix a single clip with Time scale; use the
project setting only when every clip is wrong by the same factor.

The same window's **Borrowed from other files** section borrows clips out of a *different*
rigged file — a Mixamo download, another export from an animator — and puts them
on this model, matching bones by name and keeping this model's own proportions.
An imported clip then behaves as one of this model's clips everywhere, including
every field above. See [animation-import.md](animation-import.md).

## Ragdolls

A skinned model's physics body - which parts simulate, how far each joint
bends, how a motorized ragdoll follows a clip - is its `.ragdoll` asset, made
and previewed in *Tools > Ragdoll Editor* ([ragdoll-editor.md](ragdoll-editor.md)).
The import writes the model's `.skeleton` it is built on. A game posing the
model from a simulation uses `animPose` (below), and reads the clip's own pose
for a motorized ragdoll's targets with `animClipModel`.

## Blend spaces

A model's clips can also be placed on a compass of direction and speed - a
`.blendspace` asset made in *Tools > 2D Animation Blend Tool* - and blended by
a game's input every frame instead of played one at a time: a jog, strafes, a
back-pedal and an idle that turn into one another as the move swings round
the facing. See [blend-spaces.md](blend-spaces.md).

## Third-person players

A third-person Player can use any imported animated model as its avatar. Map
Idle and Walk, then optionally Run, Sprint, Jump, Back and strafe clips. The runtime
chooses locomotion from actual movement speed, matches playback speed and
crossfades automatically. **Run at** is the fraction of the player's full-stick
**run speed** at which the Run clip takes over; sprinting is always above it.
See [player-speeds.md](player-speeds.md).

**Face camera** enables directional locomotion. Camera styles include orbit,
top-down, isometric and fixed angle. Distance, height and shoulder offset tune
the rig; a spring arm keeps it outside collision boxes.

A flow graph or script can temporarily play a one-shot clip such as an attack.
Locomotion resumes when it finishes.

![A third-person Player using an animated GLB avatar, with locomotion clips and camera settings in Properties.](img/third-person-animation.png)

## Performance and memory

Animated models cost CPU every visible frame. Keep characters lean and use:

| Setting | Effect |
|---|---|
| Draw distance | Stops drawing the object beyond a distance |
| Animation LOD | Updates distant poses every second or fourth frame |
| Mesh LOD | Builds and selects roughly 50% and 25% vertex variants |

Start with draw distance, then animation LOD. Enable mesh LOD where the reduced
model is already small on screen. Per-object overrides let a hero stay full
quality while crowds use the project defaults. Mesh LOD switches to the first
variant past the LOD distance and to the second past twice it.

Approximate skeletal memory:

```text
bytes ~= vertices * 75 + keys * 18 + bones * 72
```

Reported vertices are expanded triangle-list vertices, so the count is higher
than an indexed Blender mesh. The skinning loop's repacked bind data (about 57
bytes per vertex per LOD level) is built once per model and shared by every
instance of it; each instance owns only its skinned output - the full mesh's
arrays, plus a LOD level's buffers the first time it draws at that level.

A LOD level's buffers are taken with `new (std::nothrow)`, so a heap with no
room for them (short, or fragmented) does not stop the game: the part skins
and draws the cheapest level it already holds - the full mesh at worst, whose
arrays are its own - and asks again the next time it is skinned at that level.
Its pose keeps updating either way; `SkelInstance::lodArrays` never allocates
and answers the level that was skinned (`LodArrays::level`), so the texture
drawn matches the vertices.

A whole instance is made the same way. `SkelInstance::create(model)` answers
null - with nothing made - when the heap has no room for one: it first takes,
in one block from the nothrow `operator new[]` (a call, which a compiler may
not elide as it may an unused `new` expression), what the constructor will take
(`SkelInstance::bytesFor`: the full mesh's arrays, the pose scratch, the
palette, the clip cursors and, for a model's first instance, its bind data
and the temporary table that repacks it - about 132 bytes a vertex of the
largest level on the EE, freed again once the bind data is built - plus a
quarter and 16 KB) and gives it straight back, so the constructor's own
allocations fit where that block did. The generated game makes every animated
object's instance through it (`setupAnimObject` -> `createAnimInstance`): an
object whose instance was refused keeps its playback state and plays on without
a body - nothing is drawn for it, and a bone read on it answers false - and
`updateAndRenderAnimObjects` asks again every frame; the frame there is room it
gets its instance and restarts its clip. That covers every way an instance is
made while a game plays: a spawned object (`spawnObjectAt`), one streamed back
in (`activateObject`) and one whose model arrived later.

### What a pose costs

Measured on Gunner's combatent (2026-10-21: 37 joints, 111 channels, authored
LOD2 of 279 triangles, PCSX2 debug build, temporary COP0 brackets around each
step of `updateAndRenderAnimObjects` and `SkelInstance::evalPose`):

| per visible instance | cost | runs |
|---|---:|---|
| `evalPose` - channel sampling | 0.09 ms | when the pose refreshes |
| `evalPose` - 41 node + 37 palette matrix products | 0.18 ms -> **0.04 ms** | when the pose refreshes |
| `skinParts` at LOD2 (798 + 36 corners, VU0) | 0.14 ms | when the pose refreshes |
| StaPip submit, body part (798 corners) | 0.145 ms | **every frame** |
| StaPip submit, helmet part (36 corners) | 0.028 ms | **every frame** |
| playback bookkeeping, lighting, pose sharing | < 0.02 ms | every frame |

The matrix products were a scalar triple loop on the EE, about 2 us each; they
run on VU0 in macro mode now (`mulM4Vu0`), the same instruction sequence as
`M4x4::cross` - which drops the pose's bit-parity with the editor's
`glbparser.cpp` by VU0 rounding (~1 ulp), as the skinning loop always has. The
golden-vertex gyms that pin Gunner's combatent poses pass unchanged.

What the table says about tuning a crowd:

- **Animation LOD skips the pose and the skin, not the submit.** Past
  `animLod` a pose refreshes every 2nd frame, past twice it every 4th; the
  submit still runs every frame for every visible instance.
- **The submit scales with corners**, about 0.011 ms per 72-vertex package plus
  a small fixed cost per bag. A mesh LOD halves it only by halving the
  expanded triangle-list vertices; a separate small part (a helmet) costs a
  bag of its own.
- **Draw distance is the only setting that removes the submit.** A culled
  instance pays only its playback bookkeeping.

### What a crowd costs (2026-10-25)

A game's stress view - 22 fighters at LOD2, ~4 re-skins a frame under
animation LOD - split by the capture's `Anim_*` rows (docs/profiling.md,
"Inside the Animation phase"). Measured on the TyraX2 renderer (PCSX2):

| the view | baked cache as 2.0 shipped it | with payload patches |
|---|---:|---:|
| frame rate | 33.8 fps | **59.7 fps** |
| EE work a frame | 18.73 ms | **12.16 ms** |
| Animation stage (capture) | 14.92 ms | **3.52 ms** |
| Terrain stage (capture) | 4.63 ms | **1.30 ms** |

What moved it, and what else is fixed here:

- **A re-skinned body's baked stream is patched, not rebuilt**
  (docs/baked-vif-stream.md): every frame's 44 bags replay whole, the
  re-skinned ones after a copy of their fresh vertices. Without it every
  re-skin rebuilt its stream with the ordinary writers.
- **`animObjectMatrix` computes its rotation's sines and cosines once**
  (`rotTrigOf` + `rotatedBy`, bit-identical to three `rotated()` calls) - it
  runs for every visible instance and every `boneWorldMatrix` read.
- **A pose follower adopts its owner's node matrices** (`SkelInstance::
  adoptPose`, called by the renderer when it shares a skin): a bone a game
  reads on a follower is the body as drawn, for a copy instead of a pose
  evaluation per read.

Tried and kept out: one submission batch for the whole pass (44 -> 11 packet
sends). The sends were never the cost (VU1 wait 0.009 ms); the overlay's
drain behind the bigger batch cost more than it saved.

### Authored LODs

By default the build decimates the LOD variants itself, riding the model's own
UVs. A model whose texture atlas cannot survive decimation (one UV island per
face, as generated meshes often have) can carry **authored** LODs instead: in
the same `.glb`, skin a reduced mesh to the same armature and give it the
material `<part material>_LOD1` (and `_LOD2` for the second level). The build
folds it into that part's chain - it is never drawn as a part of its own - and
the part's chain is no longer generated. Each authored level may carry its own
baked texture, drawn over its own UVs; the part's color (and a `.mtl`
override's tint) applies to every level. A material override keeps an
authored level's embedded texture unless it defines that level's material
(`newmtl body_LOD1`), which then wins. Levels must run `_LOD1`, `_LOD2` without
a gap; a stray one is reported and skipped. Like generated LODs, authored ones
ship only when something enables mesh LOD for the model. The `.tskl` stores
their textures in an optional `LTEX` trailer; an older engine ignores it and
draws every level with the part's texture.

### Skinning into compact streams

A skinned body's drawn vertices can be its compact stream rather than float
arrays: `SkelInstance::skinInto` writes the spare side of each part's stream
pair (`skel_skin.hpp`: positions as int16 indices in a per-pose box, normals as
int8 x 127) and the renderer binds the bag to the drawn side (`lodStream`). The
scalar skin core is the reference for the vector path. The default is on in
the game template (`TYRA_SKIN_INTO_STREAMS`, 0 keeps the float VU0 skin).

To check a stream, set `SkelSkin::gStreamCheck.enabled`: each pass then compares
every corner of the written stream with a float reference skin (the palette
applied to the bind positions decoded in the model box) and keeps the worst
difference in box steps. Rounding alone stays within half a step; a corner
that reads another corner's bind data misses by metres. The check costs one
decode and one float reference per corner, so it stays off outside a gym.

The output box is per pose: `skinIntoStream` re-boxes the stream on the grid
point at the centre of the pose's own extent, so a body flung metres off its
model origin stays inside its box. The step is the power of two that
`skinOutputBox` names for the model (2^-13 m for the combatent); only a pose
spanning more than the box's 8 m clamps, and the clamp is counted, never
wrapped.

The pose's extent is bounded, not blended: each used joint's fold is applied to
the eight corners of the part's bind index box, and the union of those images
covers every blended corner. The index box and the used joints depend only on the
bind data, so `SkelSkin::bindExtent` computes them once with the bind cache; a pose
then costs eight corners per used joint and no pass over the corners. Each corner
is blended once, to write it. The vector pass takes each part's box while it
quantizes (`StaPipCompactStream::setPartBounds` and `finishBounded`), from the
positions it has just written, so the stream does not read them a second time; the
one-time self-check compares those boxes with the scalar core's `finish` and keeps
the vector path off if they differ.

### Skinning in fixed point

The fixed-point skin keeps a skinned corner in int16 from the bind indices to the
stream. `SkelSkin::Fixed::fold` runs once per pose: each palette matrix becomes
an int16 matrix on the bind index (`A = R x bind step / output step`, Q(15 -
expA), with a pose-global exponent), a Q8 output-index translation `B`, and an
int16 rotation for normals. `SkelSkin::Fixed::cornerFixed` then blends the four
joints of a corner with Q15 weights (summing to 32768 exactly): each joint term
is pre-scaled to int16 units (`expT`, set per pose from the largest bind index),
the weighted sum is shifted to output steps, and the normal is renormalised with
a 768-entry 1/sqrt table on its squared length. Out-of-box positions clamp to
+-32767 and are counted.

Measured by `tools/verify-skel-fixed.cpp` (double-precision reference, the
combatent's own bind data and a synthetic 4-bone part): the combatent's positions
are within 1.36 output steps (worst) and its normals within 0.59 x127 units; the
synthetic ordinary poses within 1.83 steps and 1.22 units. The tolerance is 2
steps and 2 units. The joint terms must stay inside int16 units of the pre-scale:
a term beyond that saturates and the corner is reported, not held to the
tolerance (the combatent and the ordinary poses saturate nowhere). A pose whose
joint translations sit more than about 4 m from the output origin is outside
this envelope and loses precision to the coarser pre-scale.

The MMI kernel (`SkelSkin::Fixed::cornerMmi`, the lane model in
`skel_skin_mmi.hpp`) is the same blend in the EE's 128-bit lanes: each joint's
rows are PMADDH products of the bind index against the folded columns, read back
as words with PMFHL.LW, pre-scaled with word shifts, and the weighted sum over
the joints is PMADDH again over those words (their low halves are the even
lanes, so no pack). It runs only on a fold with `noSat` and `narrowN` (the
normal's terms fit int16); other folds take `cornerFixed`. The positions' box
clamp and the normal's renormalisation are the scalar tail both paths share.
`tools/verify-skel-fixed.cpp` checks it bit for bit against `cornerFixed` and
`cornerNarrowRef` over 10 000 poses of the synthetic part and of every combatent
part and LOD (boxes too): 0 mismatches.

On the EE (`TYRA_SKIN_FIXED_MMI`, off by default: it measured slower than the scalar blend) the kernel's dot products run
as PMADDH with the operands in 16-byte lane vectors the fold writes (`aV`, `rV`,
`bW`); the shifts, rounds and weighted sums are scalar integer ops, since the
EE's variable word shift (PSRAVW) moves words 0 and 2 only. The first pass per
process runs the kernel and the scalar blend corner by corner over the part and
logs `SKINMMI self-check: byte-equal` (or the first differing corner's inputs).
Measured on the six Gunner gyms: all pass, combatent-lods judges 1162 frames.

### Batch skin kernel (host model)

`skel_batch.hpp` is the batch layout of the fixed-point skin: a part's unique
corners are blended through their influence entries, laid out joint-major and
padded to blocks of eight, so one block is one 128-bit load per array. Each entry
holds its vertex's bind index, bind normal and Q15 weight (a one-influence corner's
32768 is stored as 16384 and doubled in the final pass). Per pose, each joint's
folded columns are splatted into broadcast images; a block of eight entries then
runs three PMADDH per output row, the same word shifts as the scalar blend, and a
PMULTH with the weights. Each even/odd word pair is split so the products are
stored as they come out, and a CSR per unique gathers its slots; the final pass
rounds, clamps and renormalises. The integer definition is `Batch::cornerBatchRef`
and the lane model is `Batch::runPose` plus `finishPose`: they are bit-equal on
every unique of every part and LOD over 10 000 poses (10.9 million uniques), with
no int16 truncation in the lanes.

`tools/verify-skel-batch.cpp` (run by `tools/verify-tyrax2-host.py --only skel`)
measures it against the double-precision reference: positions within 1.88 output
steps (worst, mean 0.32) and normals within 1.84 x127 (worst, mean 0.25). A pass
with the output box moved 3 m clamps about 27 000 uniques to +-32767 with the
reference's sign and counts them, bit-equal to the integer definition.

Instruction count, from the steps the lane model runs (counted, not measured on the
EE): 109 MMI-class instructions per block and 24 per joint group (the image
loads), about 42 per unique at 13 % padding entries, before the scalar final
pass.

On the EE the batch path blends a pass's unique corners in an inline-asm kernel
(`rowPosAsm` / `rowNrmAsm` in `skel_batch.hpp`: six 28-instruction MMI rows per 8-entry
block, PMULTH / PMADDH / PMFHL / PSRAW immediates / PSLLW / PSRLW / POR / SQ). The
objdump of the linked ELF shows the rows straight-line: no calls, no stack references
inside a row. The forced fold (expA 0, expT 0, expR 1: `Batch::kForced`) gives the
immediate shifts 7, 8 and 14; a pose the forced exponents do not hold takes the scalar
blend (`SKINBATCH fallbacks`). The first pass per process checks every batch unique
against `cornerFixed` (`SKINBATCH self-check`), and the scatter against the corner-wise
pass (`SKINSCATTER self-check`); both were byte-equal on every device run of the six
stages (x, then y, z, then normals, each checked on the device).

Measured on the crossings views, the same builds with the batch flag on and off (BASE =
the float VU0 skin, batch = this path; work is the `[PERF]` window mean; full bin folder
with game data, idle CPU 1.5 - 4 %, interleaved, two repeats each):

| view | BASE | batch |
|---|---|---|
| crossings-coop | 20.47 ms work, 29.96 fps, 99.9 % speed | 29.42 ms work, 27.86 fps, 92.9 % speed |
| crossings-solo | 15.50 / 15.53 ms work, 44.3 fps, 73.6 % speed | 24.85 ms work, 29.65 fps, 49.2 % speed |
| stress (CROSSINGS respawn, 22 in view) | 58.7 fps, 12.99 ms work | 39.5 fps, 16.75 ms work |

The batch path does not meet the gates (stress >= 56.7 fps, coop <= 20.87 ms, solo <=
15.53 ms). Its probe, on the solo view (EE cycles a part, 82,800 part calls, 3,601 frames):
images 5.5k, sweep 13.3k (36 blocks a part), final pass 37.0k (the scalar gather, rounding
and renormalisation). The forced fold (about 40k a part in the fixed probe, not re-measured
here) and the scatter are unchanged from the scalar pass. The final pass was 369k before
its per-normal double-precision square root was replaced by the rsqrt table
(`rsqrtTable`). Batch fallbacks: 0 of 82,944 poses. Bodies re-skinned: about 11.5 a frame
in the solo view (2 part calls a body). Skinning budget: BASE used 6320 KB, peak 6320;
batch coop used 3472, peak 6073; batch solo used 2992, peak 5899 (cap 6496).

Render captures (`render_captures.py compare`, the batch build against the vector
reference): 4 identical, 4 near, 2 for review. The review of `crossings-body-2m` is a
speckle over the fighter's body (0.9 % of pixels, up to 130 of 255) that the kernel
cannot cause on its own (it is bit-equal to the scalar blend on the same fold): it is the
forced exponents' precision (expT 0 rounds each joint term to output steps). Open.
`main-menu` has 121 pixels up to 14 of 255.

The fold, the scatter and the final pass are the stress gate's remaining cost; the
kernel rows are not. Open: the fold (about 40k a part) and the final pass (37k) need the
per-unique work cut before the stress view can pass; the forced exponents need a
precision check against the float reference across the poses.
Per-part cost on the EE (the fixed probe, six gyms, 241 parts each, means):
scalar blend 505 cycles a unique corner, MMI hybrid 655; fold 42.6k and place
15.3k a part in both; scatter 93 a unique corner. The hybrid's per-joint asm
calls (lq/sq staging and the scalar glue) cost more than the scalar multiplies
they replace; the kernel stays as the bit-exact reference for an MMI rewrite
that runs a whole corner in one asm block.

The fold and the part box after C2 and C3 (same probe, same six gyms, 241
parts): the fold 34.4k cycles a part (was 42.6k: the float conversions and
the per-joint divisions it still does are the floor), the part box 4.8k (was
15.3k: the bounded extent is separable per axis). The blend, 505 cycles a
unique corner with about 216 unique corners a part, is now the largest cost;
the scatter is 93 cycles a unique corner.

`SkelInstance::skinInto` runs this core for every streamed part (`SkelSkin::gUseFixed`,
`TYRA_SKIN_FIXED`, on by default): it reads the compact bind data, blends each
corner once and copies repeats, and writes the stream. The float bind cache is
still built for the extent and the source table; the float VU0 blend remains
selectable with `TYRA_SKIN_FIXED` set to 0.

The game template binds a skinned part's bag to the drawn stream
(`lodStream`) when `TYRA_SKIN_INTO_STREAMS` is 1 (the default). The bag keeps
its texture and light sub-bags as metadata; its vertices and normals are null.
Setting it to 0 keeps the float VU0 skin (`ensurePose`).

A level whose stream the Skinning budget refuses is not made. `lodStream` then
draws the cheapest level the body already holds, and the refusal is logged once
a second (`[MEM] refused`). Nothing halts. The `skin-refuse` gym fills the
budget to check this.

Verification: `tools/verify-skel-compact.cpp` (host) checks the skin core against
a plain reference within one LSB over 10 000 random poses, and given a `.tskl`
path, every part and LOD of that real model. The gyms `combatent-visuals`,
`combatent-lods` and `skin-refuse` check the drawn streams in the game.

Cost: on the EE a re-posed body is blended by the same VU0 corner loop as the
float skin (`skinBlendCorners`), into float scratch, and the batch quantizers
(`Compact::quantizePositions` / `quantizeNormals`) write its stream. The scalar
core stays the reference: the first vector pass of the process is checked against
it (`SkelSkin::checkVectorStream`: the quantize equals the scalar encode of the
same floats bit for bit, and the stream is within one LSB of the scalar core), and
`SkelSkin::gUseVector` switches the vector path off when the check fails. In the
stress view (CROSSINGS from the respawn camera, 22 fighters at LOD2) the stream
path reads 47.7 fps at 79.5 % game speed, with 15.40 ms of work and 16.09 ms of
render a frame (5.16 ms of animation in the capture). The float VU0 skin reads
60.0 fps at 99.9 % game speed on the same settings (7.41 ms of work, 12.83 ms of
render). Two repeats of each agree exactly (2026-11-04, paired, one emulator at a
time). The stream path is not yet as fast as the float skin on the stress view, so
the float skin is the faster choice there (`TYRA_SKIN_INTO_STREAMS` 0).

Attribution: `SkelSkin::gProbe` (`TYRA_SKIN_PROBE`, 0 by default) brackets each
phase of a skin pass with COP0 Count reads. Every 50 `skinInto` calls it logs one
`SKINPROBE` line: the calls, the early returns, the palette size, the parts
entered, written, refused and skipped, and the mean ms per written part for each
phase - the per-pose extent pass, the box, the corner loop (skin, encode and
repeat copies) and `StaPipCompactStream::finish` - with the fold per pose.

Phase figures (try build, 2026-11-03, 37-slot palette, 8 000 passes, before the
bind-time extent and the one-pass part boxes): per written part the extent pass 0.44 ms, the corner loop 0.86 ms, finish
0.12 ms and the box 0.002 ms; the fold costs 0.065 ms a pose. The corner loop is
the largest phase, and the extent pass - a second blend of every corner - is about
a third of it.

#### Pose cadence of far bodies

A far body is re-posed on its turn: its pose is evaluated and its vertices re-skinned
(`ensurePose`), and between turns it holds that pose and its stream. The rule is
`SkelSkin::Cadence` (`renderer/3d/mesh/dynamic/skel_cadence.hpp`), a pure function that the
game template calls for every posed owner (`TYRA_SKIN_CADENCE`, 1 by default; 0 re-poses every
posed body each frame).

- The period is 1 within the animation-LOD distance `d` (`ANIM_LOD_DISTANCE`, or the
  object's `animLod`), 2 beyond `d` (`kFarPeriod`), and 4 beyond `2 d` (`kFarthestPeriod`).
- Bodies are staggered by their object index: a body is due on frame `t` when
  `(t + index)` is a multiple of its period, so the re-poses of a frame spread over the periods.
- A body is re-posed at once when it has no held pose, when it was not updated for more than
  four frames (it re-entered the view), or when its mesh LOD changed (the other tier's
  buffers hold an older pose).
- Host check: `tools/verify-skel-cadence.cpp`: the periods by band, each body due once per
  period, at most ceil(n / period) bodies due on a frame, a held pose never older than
  period - 1 frames, and the three overrides.

Why a held pose is exact: a stream is skinned from a pose at one clip time, and nothing in
the skin reads the frame it is drawn on. A body drawn on a held stream shows the pose of its
last turn, for up to period - 1 frames, and the next turn shows the pose at its own clip time.
Playback time is not paused (the clip advances; the body catches up on its turn).

Gameplay: a far body's node globals (`nodeGlobal`) are the pose of its last turn. Reads that
evaluate on demand (`currentNodeGlobal`, `vertexModel`) are unchanged. The cadence only applies
beyond the animation-LOD distance; close bodies are re-posed every frame.

Not done: a screen-space rule (re-pose when the joints move more than about 0.5 px at the
camera). The numeric stale-vs-current error over the combatent clips was not measured, and the
render captures do not exercise the cadence (the capture harness runs in gym mode, where
`animLod` is forced to 0), so no capture pair shows it.

Measured, normal play (the perf harness with the gym override removed for these builds;
the scene objects carry `animLod` 8, so the cadence is on for them with the project setting
at 0 or 8 alike, A and B identical):

| run | forced off (gym mode) | cadence (normal play) |
|---|---|---|
| crossings-coop work | 23.37 ms, 98.8 % speed | 19.4 to 19.9 ms (19.6 mean), 99.2 % |
| crossings-solo work | 18.8 ms, 60 % speed | 12.87 ms, 91 % speed |
| stress view | 14.55 ms, 53.2 fps | 14.54 ms, 53.0 to 53.1 fps (no cadenced body in the view) |
| bodies re-posed per frame, coop | 8.9 | 2.2 (LOD2 84 %, LOD1 6 %, LOD0 10 % of the parts) |
| bodies re-posed per frame, solo | 11.4 | 2.8 |
| bodies re-posed per frame, stress | 4.4 | 4.4 |
| Skinning peak, coop / solo | 6320 / 6227 KB (BASE) | 6073 / 5899 KB |

The gym gates run on the gym-mode build, where the cadence is off; the stream oracle and
`combatent-lods` (1162 judged frames) are unchanged.

#### Unique-vertex skin

`SkelSkin::Unique` (`renderer/3d/mesh/dynamic/skel_unique.hpp`) blends, boxes and
quantizes only the unique corners of a part (the corners whose bind data is their
own: `skinSource[v] == v`, about 19 % of a fighter's corners), and the scatter map
(`buildScatter`) writes each result to its destinations. The bind data is decoded
once per part level into float rows (position in model space, the normal / 127, the
normalised weights, the joints). Per pose the palette's float matrices are blended in
the VU0 macro order, the box is the pose's extent centre over the unique positions,
and the quantize rounds half away from zero and clamps to +-32767 / +-127.

On the EE (`TYRA_SKIN_UNIQUE_VU0`, default 1) the pass runs the existing VU0 corner loop
(`skinBlendCorners`) over the level's unique set (`SkelBindLod::uniq`, built with the bind
cache: the uniques' bind data, skin source the identity), then `writeUniqueStream` boxes the
blended uniques at the pose's extent centre, quantizes each unique once and scatters it to
its destinations. The fixed-point pass stays selectable (`TYRA_SKIN_UNIQUE_VU0` 0). The
float fold still runs per pose for the other paths; this pass does not use it.

What changed after the first measurement (2026-11-05): the per-pose fold is not made on
the unique path (it runs only for a fixed-point or stream path); the normal is quantized
without a renormalise (the VU0 blend already writes unit normals); the destination stores
and the part bounds keep their tables in locals; the normal's quantize rounds with one
float clamp per axis. A branch-free bounds update was measured and dropped (no gain).

Measured (2026-11-05, GUNNER_PERF builds, one emulator at a time, interleaved, two
repeats each, full bin folders; BASE is the float VU0 skin):

| run | BASE | unique pass |
|---|---|---|
| stress (CROSSINGS respawn, 60 s) | 60.0 fps, 7.41 ms work, 0 % missed | 53.2 fps, 14.55 ms work, 12.7 % missed |
| crossings-coop | 20.47 ms work, 29.96 fps, 99.9 % game speed | 23.37 ms work, 29.68 fps, 98.8 % |
| crossings-solo | 15.53 ms work, 44.16 fps, 73.3 % game speed | 18.73 to 18.81 ms work, 36.0 to 36.4 fps, 59.8 to 60.5 % |

The stress tool does not print game speed; its fps and missed frames are the figures.
The solo runs are not at equal game speed (the unique pass ran slower there), so their work
figures are the comparison, not the frame rates.

Memory: the Skinning budget peak is 6073 KB on coop (BASE 6320 KB) and 5899 KB on solo (BASE
6227 KB), with the unique arrays counted (about 60 bytes per unique corner, per level).
Bodies re-skinned per frame (a counter in the measurement build, parts written over
frames, two parts a body): about 8.9 on coop, 11.7 on solo, 4.4 on stress.

Per part, the probe on the coop view (COP0 Count ticks, mean of 200 parts, about 89
unique corners a part) before the changes: blend 8.6k, fold 9.5k, box 4.0k, quantize 27.1k,
store 20.2k, bounds 21.4k, finish 5.6k (about 96k). After: blend 8.6k, fold 0, box 4.0k,
quantize about 20k, stores and bounds about 24k, finish 5.7k (about 62k). On the stress view
(81 unique corners a part), the final probe reads blend 7.9k, box 3.6k, quantize 18.3k,
stores and bounds 21.4k, finish 5.2k (about 56k, from 88k). Quantize and the bounds are each
about a third of the part; the blend and the finish are the rest.

Verdicts: the floor holds (coop 23.37 ms <= 24.5, solo 18.8 ms <= 20.1, stress 53.2 fps >=
49.5, Skinning peak <= BASE, six gyms pass, captures: 9 identical, 1 near (crossings-body-2m,
5 pixels, max 1 level), none for review). The target does not: coop is 0.9 ms above the
22.5 ms target, solo 1.3 ms above 17.5 ms, and stress 0.5 fps below 53.7 (the target's own
figure; BASE minus 5 fps is 55.0).

Host check: `tools/verify-skel-unique.cpp` (run by `tools/verify-tyrax2-host.py
--only skel`) over 10 000 poses of every part and LOD of the combatent model:
positions worst 0.503 steps (mean 0.250, rounding only), normals worst 0.500 x127
units (mean 0.250), repeats byte-equal to their source, part boxes equal the unique
extent, every corner covered by the scatter; 200 clamp poses (50 m joint spread)
clamp with the reference's sign and never wrap.

#### Skinned streams with indexed corners

Every skin path above writes each skinned vertex about five times: the compact stream holds
triangle CORNERS (no index buffer), so after the blend the EE quantizes a unique vertex once and
then stores it into every corner that uses it. On the stress view (per part, COP0 ticks) the
unique pass costs blend 8.0k, box 3.8k, quantize 18.1k, **stores + bounds 38.0k**, finish 5.3k:
half of it is the corner duplication. The MMI batch kernel lost for the same reason (its scatter
is ~20k on top of the fold and the final pass), and the EE-only path to the GS spends 37k a part
writing corner packets. The indexed stream moves the duplication to VU1, where it is a load (lower
pipe, hidden) instead of an EE store, and it keeps every byte of the data compact (int16 / int8).
It is proven on the host and in the VU simulator, and wired into the engine behind
`TYRA_SKIN_INDEXED` (compile-time, default 0, `skel_indexed_config.hpp`): see "What was integrated"
below for what it does on the device and what it costs and saves.

**The three parts.** All of it is plain C++ in `renderer/3d/mesh/dynamic/skel_indexed.{hpp,cpp}`:

| part | what | where it lives | rewritten |
|---|---|---|---|
| static block, per package | the program's entry (3 qw), the **corner table** (one V4-16 element a corner: S, T, Q and the unique's address) and the kick | shared by every body of a model part level (about 10 bytes a corner) | never; sent by REF |
| pose block, per package | the package's **unique** vertices: positions V4-16 (8 bytes: int16 x, y, z, pad) and normals V4-8 (4 bytes), each a compact run with its header | one pose buffer per body and part level | every pose, once per unique per package |
| the VU1 program | `stapip_cull_td_indexed_vu1` (and `_fold`): per corner `ilw.w` the address, one `iadd` of TOPS, `lq` the position at it and the normal 75 quadwords on, then the cull TD tail unchanged | micro memory | - |

The package boundaries are the plain stream's (the same `packageSize` corners; the part boxes the
frustum classifier reads are a third of a package, as now). A unique two packages use is in both
(`Level::duplication()`), the corner's local index is static, and the unique list of a package is
in the order its corners first use it. Bind time (`Indexed::build`) needs only the part level's
`skinSource`; per pose the writer (`Indexed::writePose`) gathers the quantized level uniques that
BOTH skins produce (`Unique::quantize`, `Batch::finishPose`: `qPos [unique][4]` s16, `qNrm
[unique][4]` s8) into the package order. The part bounds keep the shipped loop (each unique updates
the parts it falls in, `SkinScatter::rng`): they were never per corner.

**VU1 buffer** (one half of the double buffer, 461 quadwords, TOPS-relative; `n` corners, `u <= n`
uniques in the package). The corner table's address lane is pre-biased by the EE for the package's
size, so VU1 needs no per-package setup:

| quadwords | what | written by |
|---|---|---|
| 0, 1 | scale / count, GIF prim tag | the entry (as now) |
| 2 .. 2 + n | corner table: S, T, Q (int32, 4.12 x the package's scale) and in w the quadword of the corner's unique, `11 + 4n + local` | static block, unpack V4-16 |
| 2 + n .. | the GIF tag block (up to 9) and 3 qw a corner: the program's output | VU1 |
| `11 + 4n` .. + u | unique positions, grid index in xyz, w = 1.0 (COL), offset MODE as every position run | pose block, unpack V4-16 masked |
| `11 + 4n + 75` .. + u | unique normals, int8 x 127 | pose block, unpack V4-8 |

n = 48 (the shipped skin package): corner table 2..49, output 50..202, positions 203..250,
normals 278..325. n = 75 (the cull TD maximum): corner table 2..76, output 77..310, positions
311..385, normals 386..460 - the last quadword of the half. The split (75) is a constant of the
program: both unique arrays are at most 75 long, so one address register serves both loads and a
package of up to 75 corners always fits whatever its `u` (`Indexed::fits`). The plain package
size is therefore the ceiling; a package of more corners needs a program with another split.

**VIF stream, per package: two REFs.** The pose block comes first:
`{STMASK, 0x80808080, STMOD 1, UNPACK V4-16 masked u @ 11+4n}` + the positions,
`{NOP, NOP, STMOD 0, UNPACK V4-8 u @ 11+4n+75}` + the normals; the static block follows:
the entry's 3 quadwords, `{NOP, NOP, STMOD 0, UNPACK V4-16 n @ 2}` + the corner table, and the kick
`{STMOD 0, NOP, FLUSH, MSCAL}`. Every UNPACK is built by hand (signed, TOPS-relative, STCYCL 1/1 is
the bag setup's) and no package leaves VIF1 in offset MODE. A fully visible package is drawn from
these two REFs; a package that crosses the frustum goes to the clip programs, which read plain
corners, so it is **expanded first** (the host verifier's `decodePackage` is that expansion) and
takes today's path.

**What the host proves** (`tools/verify-skel-indexed.cpp`, in `tools/verify-tyrax2-host.py --only
skel`, real combatent model, every part and LOD):

- *Packaging.* The packages cover every corner exactly once, every local index is in range and
  names the corner's own unique, every package fits the buffer half, the gather list names only
  level uniques.
- *Byte-exact decode.* For 2000 poses (every 100th throws the joints 50 m apart so the box clamps),
  package sizes 48 and 75, the model's own triangle order and the locality order: the unique
  vertex path writes the plain corner stream (`Unique::blend` + `writeUniqueStream`); the same
  blend is quantized once, gathered into the pose blocks, and the pose and static blocks are put
  through a model of the VIF (the three UNPACKs the layout uses, STMASK, STMOD, ROW) and of the
  VU1 corner fetch. The position, ST and normal runs that come out, padding included, equal the
  plain stream's byte for byte (816 000 packages), and the part boxes formed from the decoded corners equal the
  plain stream's. Mutating one corner-table address makes it fail (checked by hand, not shipped).
- *The batch kernel.* The lane model of the MMI batch kernel writes the same level-unique arrays;
  its poses gather and decode to the scatter's corners, byte for byte.
- *The VU1 program on the real data.* For sample poses every package of every part level is staged
  twice in VU1 memory through the capture tooling's VIF decoder (`vucap`): the plain package's
  three runs for the shipped cull TD, and the pose and static blocks for the indexed program. Both
  programs run in the VU simulator and every GS word of every kick is identical (the ST register's
  ignored fourth lane aside): 2 040 packages, 1.29 million GS words, none differ, and a moved unique
  is always seen (350 of 350 controls).

**The numbers** (combatent, bytes; "plain" is one side of today's stream, the pose buffer one side
of the indexed one; part boxes, 1.5 bytes a corner, are the same on both and not counted).
Duplication is package-unique entries over level uniques, which is also the stores a unique costs:

| package 48 | corners | level uniques | uniques / package | duplication = stores / unique (plain 5.2-5.5) | plain, one side | pose buffer, one side | static, shared | gather list, shared |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| LOD0, model order | 3348 | 612 | 21.8 | 2.49 | 61 440 | 21 264 (34.6 %) | 32 400 | 3 050 |
| LOD1, model order | 1662 | 322 | 27.9 | 3.12 | 30 672 | 13 536 (44.1 %) | 16 192 | 2 012 |
| LOD2, model order | 834 | 162 | 26.7 | 2.97 | 15 408 | 6 544 (42.5 %) | 8 112 | 962 |
| LOD0, locality order | 3348 | 612 | 16.7 | 1.91 | 61 440 | 16 960 (27.6 %) | 32 400 | 2 340 |
| LOD1, locality order | 1662 | 322 | 15.1 | 1.69 | 30 672 | 8 048 (26.2 %) | 16 192 | 1 088 |
| LOD2, locality order | 834 | 162 | 15.3 | 1.70 | 15 408 | 4 032 (26.2 %) | 8 112 | 552 |
| LOD0, locality, package 75 | 3348 | 612 | 22.6 | 1.67 | 60 016 | 14 160 (23.6 %) | 30 736 | 2 038 |
| LOD2, locality, package 75 | 834 | 162 | 20.4 | 1.51 | 15 024 | 3 456 (23.0 %) | 7 712 | 490 |

The duplication is NOT the 1.1-1.3 one would hope for: a package is 16 triangles (48 corners), and
the vertices on the boundary of any such patch are shared with a neighbouring package. The model's
triangle order (the glTF primitive order) has poor locality; `Indexed::orderTriangles`, a greedy
packer (each next triangle adds the fewest new uniques to the open package), takes it from 2.5-3.1
to 1.7-1.9 and larger packages go down to 1.5. The reorder only changes the draw order inside an
opaque part; the plain stream and the indexed one are both built from the reordered part (the
verifier checks the byte-exact decode on it too), and it is worth doing for the plain stream as
well (tighter part boxes). It belongs in the bake, not in the pose path.

Per body: the plain stream is two sides (ping-pong) of the first column; the pose buffer is the
second. Shared per model: the static blocks (9.7 bytes a corner: the corner table is 8, the rest
the entry and kick) and the gather list (2 bytes a package-unique). Two scenarios (all numbers
bytes; the stress view's 22 fighters at LOD2, and a coop mix of 24: 6 at LOD0, 6 at LOD1, 12 at
LOD2):

| | plain, ping-pong | indexed, model order | indexed, locality order | locality order, package 75 |
|---|---:|---:|---:|---:|
| 22 fighters at LOD2: streams | 677 952 | 287 936 | 177 408 | 152 064 |
| + shared (static + gather) | 0 | 9 074 | 8 664 | 8 202 |
| 24 coop mix: streams | 1 475 136 | 574 656 | 396 864 | 336 768 |
| + shared (static + gather, three LODs) | 0 | 62 728 | 60 684 | 57 290 |

(Each model carries its own static blocks: the two combatent .tskl files are two models.) So the
streams shrink to 39-43 % of today's ping-pong pair in the model's order and 26-27 % in the locality
order (22-23 % with 75-corner packages), and the shared part is less than a tenth of what is saved. **Against the Skinning peak of ~5.8 MB the
saving is 0.8-1.1 MB on the coop mix**: the stream pairs are not the bulk of that peak (the bind
cache and the unique arrays are), so the integration's memory table is the figure that decides.

*Can one pose buffer replace the ping-pong pair?* A body's pose block is read by the DMA chain
until the chain that REFs it has run (up to three packets later with `Vif1Queue`), so the buffer a
re-pose writes into must not be one a chain still reads. The pair is that rule applied per body.
With pose buffers this small a shared pool does it for less: every body keeps ONE buffer and a
re-pose takes a free one from a pool, handing the old one to the retire ring (the span-retire the
streams already use). The pool holds `R x D` extra buffers, R the bodies re-posed in a frame (4.4
on stress, 8.9 on coop, measured) and D the frames a chain can lag (1-3): 22 + 9 buffers instead of
44 on stress at D = 2. A body drawn without a re-pose (far bodies on their turn, the split screen's
second half) keeps drawing its one buffer.

**The VU1 program** (`vugen::descCullTextureDirLightsIndexed`, `Desc::indexedCorners`; emitted to
`stapip_cull_td_indexed_fold_vu1.vclpp` in `.../static/core/programs/cull`, which the engine
assembles and links as `StaPipCullTDIndexedVU1Program` (the image the folded-light default needs),
and `stapip_cull_td_indexed_vu1.vclpp.txt` in `.../static/core/compact`, kept as text only; `--vu-check`
holds both files to the generator).
Assembled with the toolchain's openvcl (`vclpp`, `vcl`, `dvp-as`; Sony's `vcl` was not available):

| image | instructions | words (micro memory) | loop, rows a triangle | a corner |
|---|---:|---:|---:|---:|
| cull TD (shipped, compact) | 214 | 160 | 106 | 35.3 |
| cull TD, indexed | 223 | 170 | 106 | 35.3 |
| cull TD fold (the default image), compact | 202 | 148 | 97 | 32.3 |
| cull TD fold, indexed | 211 | 158 | 97 | 32.3 |

The rows are the scheduled lines of the loop in the assembler's output (issue slots; latencies are
what vcl scheduled around), counted from the `vertexLoop` label to the `ibne` and its delay slot:
**the index fetch costs nothing per corner** - the upper pipe is the bound (102 operations a
triangle in both), the lower pipe takes `ilw`, `iadd` and two `lq` a corner where the plain loop took
three `lq` and four pointer increments. The first version loaded the addresses at the top of the
loop and lost 3 rows a triangle to the `ilw -> iadd -> lq` chain; the addresses of the next
triangle are now fetched at the end of the loop (vcl does not software-pipeline), which closed it.
A package pays about 10 more rows once, for the first triangle's addresses: 0.6 % of a 16-triangle
package. As micro memory: the VU1-clipping set is 1 708 words, so the fold image fits as an extra
program (1 866 of 2 042); with the billboards (1 906) it does not (2 064, 22 over), and that
project drops a class with `setResidentClasses`. It cannot replace the plain image: static props
still draw through it.

`--vu-check` (section "indexed corners vs the compact TD programs") runs the indexed program against the
shipped cull TD (and the fold twins) over random draws with random numbers of uniques and requires
every GS word identical; its control points a corner at another unique and must differ.

**Expected EE saving per part** (from the measured stage costs above; an estimate until the
integration measures it). The stores and bounds (38.0k) split into the bounds (the per-unique
part-range updates, not measured apart: taken as 45 ticks a unique, about 3.6k, unchanged) and the corner stores (about 34k, 82 ticks a destination
corner). The gather is one 8-byte and one 4-byte copy per package-unique with no destination decode,
12 ticks assumed (10-20), 154 package-uniques in the locality order (202 in the model's): about
2k. The part then costs blend 8.0k + box 3.8k + quantize 18.1k + gather 2.0-2.4k + bounds 3.6k +
finish 5.3k = about 41k instead of 75k, **about 34k ticks (0.12 ms) a part**, and with 4.4 / 8.9
bodies of two parts re-posed a frame about **1.0 ms on stress and 2.0 ms on coop**. The batch kernel
gains the same (its 20k scatter becomes the same gather). The added work is two REFs a package
instead of one per bag (an EE chain cost the integration measures), and the expansion of packages
that cross the frustum.

**What was integrated** (`TYRA_SKIN_INDEXED` 1; with 0 nothing below exists in the build).

1. *Bind.* The first `SkelInstance` of a model puts every skinned mesh level in the locality order
   of `Indexed::orderTriangles` (75-corner packages) once, in the loader's own arrays and before
   anything reads them, so the bind data, the streams and the stream check all see one corner
   order. A part level is packaged the first time a body draws it (`SkelBindLod::idx`, an
   `Indexed::Shared`): `Indexed::build` at 75 corners a package (48 as the fallback for a part
   whose packages would not fit the buffer - none of the combatent's part levels needed it),
   the static blocks (`buildStatic`; the ST payload is the plain stream's own ST run, package by
   package), the gather list, and for each unique the stream parts its corners fall in (the part
   boxes' ranges). `local` and `uid` are dropped after the build. Shared by every body of the
   model and charged to the Skinning budget. The game template builds the skinned bags' stream
   layouts with `Indexed::kStreamPackage` (75).
2. *Pose.* A body keeps ONE stream per part level (boxes only, `StaPipCompactStream::
   allocateIndexed`) whose block is a pose buffer from the level's pool. The VU0 unique pass
   blends the uniques, quantizes each once into the level's scratch (bounds of the parts from the
   gathered ranges), `writePose` gathers them into a free pool buffer, the stream takes the buffer
   and the buffer it replaces is retired. A retired buffer is taken again once no recorded chain
   reads it (`Vif1Queue::immutableSpanReady`); every pose buffer is a registered immutable span for
   its whole life (a retire / collect round trip per pose shifts the span table twice and cost
   ~6k ticks a pose before it was dropped). The pool is demand sized: it ends at the bodies plus
   the poses in flight (below). A pass that cannot get scratch or a buffer is refused and the body
   keeps drawing its current buffer; a level the indexed packaging cannot take, or whose layout is
   not the program's (textured, lit, 75 corners), keeps the plain stream pair. The plain stream
   is not made for the clip routes: they decode the package they need (below).
3. *Renderer.* `StaPipCullTextureDirLightsIndexed` is a resident program when the textured lit
   class is and it fits under the draw-finish helper (the fold image: 1866 of 2042 words, see
   micro memory below). A wholly visible bag replays two REFs a package, the body's pose block and
   the model's static block, whose entry (count, GIF tag) and kick (MSCAL for the first package,
   MSCNT after) words are captured into the blocks on the first draw
   (`StaPipQBufferRenderer::replayWholeIndexedBag`); the capture is keyed like the plain stream's
   and a draw with another program or state sends fresh entries. Any other bag goes through the
   ordinary qbuffer route: a fully visible package is `StaPipQBuffer::fillByIndexed` (the entry
   written fresh, two REFs), a package that crosses the frustum - or a bag the indexed program
   cannot draw - is expanded to plain corners in the copy pool (`expandIndexed`; `Compact::copyRun`
   and the view read an indexed stream element by element) and takes the clip path as before. An
   indexed stream is never baked, retained or stamped. The renderer logs, once per 600 frames,
   `[SKIN] indexed packages per drawn frame: sent N, expanded to corners M (max K); pose
   buffers ...`.
4. *Checks.* Host: `python tools/verify-tyrax2-host.py --only skel-indexed` also reads every corner
   of the engine's indexed stream through the view and `copyRun` against the plain stream (46.8
   million corners, 0 differ), and the shared range lists against the scatter's. Device
   (below): the six skin gyms with the stream check reading the indexed stream corner by corner,
   and the render captures.
5. *Package size.* 75 (the program's ceiling) also lowers the duplication to 1.5-1.7 stores a
   unique and the packages a part to 41 (LOD0), 21 and 11.

**Device results** (PCSX2 software renderer, Gunner development build with the flag on, combatent
model). The six skin gyms pass: `combatent-visuals` (stream check: 1 310 passes, 1.65 million
corners, worst 0.502 steps - the rounding of the quantize, as the plain path), `combatent-lods`
(1 162 frames judged, all three levels), `combatent-blend`, `ragdoll-gym`, `compact-pack`,
`skin-refuse` (the Skinning budget refusing 421 requests: the fallback to the held level or the
plain pair works with the flag on). The ten render captures against the plain path:

| view | changed px | max | PSNR dB | SSIM |
|---|---:|---:|---:|---:|
| crossings-batch-edge, -seam, -wall, main-overview | 0 | 0 | inf | 1.0000 |
| crossings-bodies-30m | 0.015 % | 3 | 86.6 | 1.0000 |
| crossings-bodies-70m | 0.001 % | 2 | 95.1 | 1.0000 |
| crossings-body-2m | 0.848 % | 2 | 69.6 | 1.0000 |
| crossings-respawn | 0.009 % | 1 | 89.1 | 1.0000 |
| crossings-split | 0.026 % | 3 | 84.6 | 1.0000 |
| main-menu | 0.193 % | 2 | 75.4 | 1.0000 |

(Six of the ten views differ at all, by at most 0.85 % of the pixels and 3/255: the stream decodes
to the same corners, and what moves is the draw order inside a part, which the locality reorder
changed.) The same ten views with
every package forced through the clip route's expansion (a test build: no whole-bag replay, every
indexed package decoded to plain corners) give the same numbers, so the decode in the copy pool is
the same picture. Packages that cross the frustum are rare in these scenes: the coop gym averages
0.09 expansions a drawn frame (at most 16) against 130-190 packages sent as they are; the stress
view and the captures have none.

**Memory** (bytes; Skinning, BakedStreams and Renderer are the `[MEM]` budgets of the gym run's
end: `used` / `peak`, KB; flag 0 is the branch the flag sits on, with the flag off).

| view | | Skinning used / peak | BakedStreams used / peak | Renderer used / peak |
|---|---|---:|---:|---:|
| crossings-coop | flag 0 | 3 645 / 6 073 | 588 / 854 | 38 / 40 |
| | flag 1 | 3 039 / 6 052 | 588 / 854 | 92 / 92 |
| crossings-solo | flag 0 | 3 009 / 5 899 | 466 / 1 098 | 115 / 138 |
| | flag 1 | 2 421 / 5 879 | 450 / 1 098 | 195 / 195 |
| stress view | flag 0 | 2 834 / 5 899 | 882 / 997 | 38 / 40 |
| | flag 1 | 2 211 / 5 879 | 883 / 997 | 38 / 40 |

The steady state falls by 0.6 MB (coop, solo, stress); the PEAK falls by 20 KB only, because it is
not the streams: it is the moment every body exists and none has drawn yet - each holds the float
mesh arrays its constructor made (48 bytes a corner, 160 KB for a LOD0 body; `releaseFloatMesh`
frees them at the first draw), on top of the bind cache. The peak is where the budget has to be
sized, so a smaller Skinning budget needs those arrays gone (a body that only ever draws a stream
need not make them), not smaller streams. The Renderer budget grows 54-80 KB with the flag on: the
copy pools of the expansions (the plain path drew the same packages in place). Nothing was refused.

Per fighter (both parts, boxes included, from the bind data; the plain pair is two sides of the
shipped 48-corner stream, the indexed figure one pose buffer of 75-corner packages):

| level | plain pair | indexed, one buffer | |
|---|---:|---:|---:|
| LOD0 | 133 488 | 17 616 | 13.2 % |
| LOD1 | 66 768 | 8 744 | 13.1 % |
| LOD2 | 33 648 | 4 392 | 13.1 % |

Shared per model (static blocks + gather list + ranges, all three levels, both parts): 69 KB
(LOD0 28.1 + 8.7 for part 0 and 2.6 + 0.9 for part 1; LOD1 14.2 + 4.4 and 1.1 + 0.4; LOD2 7.3 + 2.3
and 0.4 + 0.1); the two combatent files are two models. The pose pool settles at the bodies plus
the poses in flight: 68 buffers (114 KB) for the stress view's 22 fighters at LOD2 (48 held, 20
spare), 86 buffers (211 KB) for the coop mix of 24 across three levels and 100 (196 KB) for solo.

**Micro memory and billboards.** With the indexed image the resident set is the ten class programs
and the indexed one: 1 866 of 2 042 words. The two billboard programs (206 words) no longer fit
beside it (22 words over), so `setProgramsCache` leaves them to the on-demand set
(`billboards swapped on demand` in the log); a scene that draws a billboard bag pays the two VIF1
drains and the MPG upload of `ensureProgramSet` on every transition, and the first swap is logged
once (`the on-demand billboard set is swapped in`). Gunner's scenes draw none: the line never
appeared in the coop, solo and stress views, the six gyms or the captures. A project with particles
would drop a class with `setResidentClasses`, or leave the flag off.

**A first look at the EE cost** (stress view, COP0 ticks a part, mean of 200 parts of 81 uniques and
417 corners; a temporary probe, reverted; one run, not the paired measurement). The earlier probe of
the shipped path on the same view is the "before" column:

| stage | shipped (scatter) | indexed |
|---|---:|---:|
| pool buffer (acquire) | - | 0.3k |
| blend (VU0 corner loop) | 8.0k | 7.9k |
| box (pose origin, rebox) | 3.8k | 3.8k |
| quantize | 18.1k | 30.1k, with the part bounds |
| stores and part bounds | 38.0k | 4.3k, the gather into the pose |
| finish | 5.3k | 3.4k |
| **part** | **73k (about 75k)** | **50k** |

The pass saves a third (about 24k ticks a part, 0.08 ms; the estimate above was 34k): the gather is
twice the 2k assumed, and the quantize with the part bounds is 8k over its estimate - the ranges
loop (a handful of loads and compares per unique and part) is the hot spot, and the MMI halfword
min / max (PMINH, PMAXH on x, y, z) is the obvious cut. A rough frame figure on the stress view:
52.4 fps, 14.66 ms of work with the flag on (probe included) against 52.8 fps, 14.57 ms off, one
run each: the pass saving is about 0.7 ms a frame and is not visible against the noise yet. Before
the whole-bag replay of captured entries the same view read 45.6 fps / 15.66 ms: the ordinary
route's per-package EE work (qbuffers, a fresh entry each) costs about 1 ms a frame more than
the replay, which is why the replay exists. The paired measurement below separates the EE
saving of the pass from the replay's cost (two REFs and 130-260 packages a frame; the static
blocks' cache lines); the VU1/GS side PCSX2 does not time.

**Paired performance** (flag 0 against flag 1, the same commit; two perf builds, each with the
editor clean-rebuilt for its flag value; PCSX2 software renderer, boosted, one emulator at a
time, idle CPU under 15 % before every run, runs interleaved 0, 1, 0, 1 and the ELF run from a
folder that holds the whole game data). Normal play is the shipped animation LOD (far bodies are
re-posed every 2nd or 4th frame; about 2.2 bodies a frame on coop, 2.8 on solo and 4.4 on the
stress view). Work is the EE time of a frame (frame minus idle); the gym rows are the
`[PERF]` windows summarised, the stress row the 60 s window of the respawn view.

| view | flag | run 1 | run 2 | run 3 | fps | game speed | missed |
|---|---|---:|---:|---:|---:|---:|---:|
| crossings-coop, work ms | 0 | 19.70 | 18.04 | 19.70 | 29.9 | 99.2-99.5 % | 0.3-0.5 % |
| | 1 | 18.82 | 18.82 | 18.81 | 29.9 | 99.8 % | 0.07 % |
| crossings-solo, work ms | 0 | 12.96 | 12.95 | 12.95 | 54.5-56.3 | 90.5-93.4 % | 6.2-9.7 % |
| | 1 | 12.78 | 13.03 | 13.03 | 55.1-56.7 | 91.5-94.1 % | 5.6-8.6 % |
| stress view, work ms | 0 | 14.61 | 14.60 | | 52.7 | 87.8 % | 13.5 % |
| | 1 | 14.60 | 14.60 | | 52.8 | 87.9-88.0 % | 13.4 % |

Update and render ms (mean of the windows): coop 7.88 / 24.23 (flag 0, run 1) against 7.03 /
24.99 (flag 1); solo 3.73 / 13.45 against 3.58 / 13.40 (run 1; runs 2 and 3 within 0.15 ms);
stress 3.76 / 14.06 against 3.76 / 14.04. The coop flag-0 runs fall into two states (19.70 and
18.04 ms), which is the 1.7 ms spread to read the coop row against; the flag-1 runs are one state.
The two views that repeat to 0.03 ms (solo flag 0, stress) are the ones that carry a verdict.

The same pairs with the animation LOD off (every body re-posed every frame, the gym setting; the
case where the pass runs most often):

| view | flag | run 1 | run 2 | fps | game speed |
|---|---|---:|---:|---:|---:|
| crossings-coop, work ms | 0 | 23.46 | 23.46 | 29.7 | 98.8 % |
| | 1 | 22.30 | 22.30 | 29.9 | 99.8 % |
| crossings-solo, work ms | 0 | 18.82 | 18.80 | 36.2 | 60.1-60.3 % |
| | 1 | 17.12 | 17.12 | 39.0 | 65.0 % |

(The stress view is the same build either way: its bodies are re-posed every frame in both.)

Indexed packages per drawn frame (flag 1): coop 130 and 187 sent as they are, 0.09 expanded to
corners (at most 16); solo 173, 227 and 255 sent, 0.14 expanded (at most 14) in the first view
and none in the others; stress 264, none expanded. `[MEM]` (KB, used / peak) at the end of the run,
flag 0 against flag 1: Skinning coop 3 373-3 692 / 6 075 against 3 011 / 6 052, solo
2 952-3 148 / 5 901 against 2 351-2 365 / 5 879, stress 2 838 / 5 901 against 2 211 / 5 879;
Renderer coop 38 / 40 against 92 / 92, solo 115 / 138 against 193-195, stress 38 / 40 against
38 / 40 (the copy pools of the expansions); nothing refused.

**Verdict.** In normal play the indexed stream does not make a frame measurably faster or slower:
the stress view reads 14.60 ms against 14.60 / 14.61 (0.00-0.01 ms, 0.02 ms of render), solo
12.95 ms against 12.78-13.03 (within 0.1 ms), and coop 18.8 ms against a flag-0 spread of 18.0
to 19.7 ms (a mean 0.3 ms in the flag's favour, inside that spread). That is a good deal less than
the roughly 1 ms a frame the pass saving of the probe (50k ticks a part against 75k, 4.4 bodies of
two parts a frame on stress) promises: the rest is consumed on the draw side (two REFs a package
and the replay's captured entries, the pose pool, the clip-path expansions), which this
measurement does not break down. With every body re-posed every frame the saving shows: 1.16 ms
a frame on coop (5 %) and 1.70 ms on solo (9 %, game speed 60 % to 65 %). The flag is therefore
worth turning on for the memory it frees (about 0.6 MB of steady Skinning state and 87 % of every
body's stream) and for scenes that re-pose many bodies a frame, and it costs no speed in the
scenes measured here; it is not a speed-up for the shipped animation LOD.

**The later VU0 split (not built here).** The point of the layout is that its input is what a VU0
micro-mode blend and an EE MMI pack produce. The unique buffer is `qPos [n][4]` s16 (8 bytes a
unique, 8-byte aligned, the fourth halfword unused) and `qNrm [n][4]` s8 (4 bytes, 4-byte aligned),
package runs padded to 16 bytes - exactly what `Unique::quantize` writes. A VU0 microprogram fed by
VIF0 would work on a batch of uniques: VIF0 UNPACKs the int16 bind data (position V4-16, normal
V4-8, joints V4-8, weights V4-8: 4 qw a unique) into VU0 data memory; the palette (the part's used
joints, 4 qw each - `bindExtent` names them) sits above it; the program blends (float, the macro
order of `Unique::blend`), takes the batch's min / max for the pose box, and leaves 2 qw a unique.
VU0 data memory is 4 KB (256 qw) and VIF0 does not double-buffer: with J joints used the room for a
batch is `(256 - 4J) / 6` uniques. The combatent has 37 palette slots and its body part uses 30
of them (the other part 3): J = 30 leaves 22 uniques a batch (11 each with two batches in flight,
input and output halves), so the LOD2 body (151 uniques) is 7 batches single-buffered and 14 with
overlap, each with a sync (VIF0 MSCAL, then the EE polls VPU_STAT) - the palette is what limits
it, and a joint-major batch (one joint's matrix resident, the influence entries streamed through
it, as the MMI kernel does) is the alternative to price. The output is read by the EE from the VU0 data window (VU0 cannot DMA out), where
the MMI pack (CVT.W.S, PPACH / PPACB) quantizes it straight into the pose block; the cost of that
window read per quadword is the open number. To drop the gather, VU0 can blend in PACKAGE order
(bind data duplicated per package-unique, `duplication()` times the bind bytes, blend and quantize
cost times the same factor, results land where the pose block wants them): the host verifier checks that
(a pose blended per package-unique, 50 020 entries, is bit-equal to the gathered one, pose origin
included, because a unique's blend is independent of its neighbours). Sync points per frame: palette upload (once a
body), batch DMA, MSCAL, poll, EE pack, with the next batch's DMA overlapping the pack.

## Flow graph

The **Animation** action plays or stops a clip on its linked object (or self).
Play accepts clip, loop, speed and crossfade time. **On Animation Finished**
fires once at the end of a one-shot and on every wrap of a loop.

```text
On Used -> Animation play "open"
On Animation Finished -> Set Object Visible (hide)
```

Clip names are case-sensitive. An empty name uses the first clip.

## Scripts

`inc/scripts/script.hpp` provides:

```cpp
playAnimation(ctx, objectIndex, "attack", false, 1.5F);
playAnimation(ctx, objectIndex, "idle", true, 1.0F, 0.3F); // crossfade
stopAnimation(ctx, objectIndex);

if (animationFinished(ctx, objectIndex)) {
  // ended or wrapped this frame
}
```

A game can also **drive** a clip by one of its own values instead of
playing it. `seekAnimation` holds a clip at a fraction of its length (0 =
first frame, 1 = last), so a two-key clip - frame 0 standing, the last frame
crouched - sought to the crouch depth every frame is a linear interpolation
between its two poses, with the game owning the parameter. `blendAnimation`
lays a second clip, held at its own fraction, over whatever the object is
playing or holding, by a weight: an offset such as a sideways lean rides on
top of the idle, the walk and the crouch alike, and survives a clip switch
underneath it. Both are re-applied every frame; a blend ends when it is
called with weight 0.

```cpp
seekAnimation(ctx, objectIndex, "crouch", crouchDepth, 0.15F);  // 0..1, crossfade in
blendAnimation(ctx, objectIndex, "lean_left", crouchDepth, leanAmount);
blendAnimation(ctx, objectIndex, "lean_left", 0.0F, 0.0F);  // off
```

A held pose is evaluated once and then shared between instances holding the
same one, exactly like a looping clip; a pose that moves every frame re-skins
every frame, so keep driven clips on the models that need them. A switch
into a held clip crossfades on wall-clock time (the held clip has no playback
of its own to carry the fade), and a held clip that fades out stays where it
was put rather than playing on.

Or the object shows one of its model's **2D blend spaces**
([blend-spaces.md](blend-spaces.md)): up to 3 clips weighted by a direction
(degrees from its facing, + = right) and a speed it is given every frame.
`playAnimation` / `seekAnimation` take it back to clip mode with their fade;
`blendAnimation`'s layer works in both modes.

```cpp
setAnimationMode(ctx, objectIndex, ANIM_MODE_BLEND_SPACE, "locomotion", 0.15F);
setBlendSpaceInput(ctx, objectIndex, moveDirDeg, moveSpeed, strideFraction);  // every frame
int clips[3]; float weights[3];
blendSpaceWeights(ctx, objectIndex, clips, weights);  // what it showed (blend LOD included)
```

These calls are no-ops for objects without animation.

To **carry** something on a bone - a weapon in a hand, a lamp on a helmet -
read the bone's world placement with `boneWorld` and write it into the other
object's data every frame. Author the socket in Blender as a non-deforming
bone parented to the hand, posed where the carried model's origin and axes
should be; its name is all the script needs.

```cpp
float pos[3], rot[3];
if (boneWorld(ctx, bodyIndex, "weapon", pos, rot)) {
  RuntimeObject& gun = ctx.objects[gunIndex];
  for (int i = 0; i < 3; ++i) gun.data.position[i] = pos[i], gun.data.rotation[i] = rot[i];
  gun.dirty = true;
}
```

The pose is the one the body was **last drawn** in (so a far body whose
animation LOD skips frames keeps its weapon on its stale hands, not on a pose
nobody saw); the body's own transform is read from its data as it is now, so
a weapon on a running body does not trail it by a frame of travel. A body
that was never drawn, or that **followed another instance's skin** this frame
(pose sharing: bodies on the same clip at the same time skin once, and a
follower evaluates nothing of its own), has its pose evaluated on demand -
once per pose change - so a crowd of idling soldiers still answers for every
body, not only the one that skinned (`SkelInstance::currentNodeGlobal`). It
returns false - and leaves the outputs alone - for an object that is not
animated or a bone the file does not name: keep a fallback placement. Bones are found by their glTF node name, stored in the `.tskl`'s
optional `NAME` trailer; a `.tskl` baked by an older editor has none, and
the next build re-bakes it. Uniform object scale is assumed.

For **tests** that pin a pose - "the crouch held at 0.5 puts the head top
HERE" - `skinnedVertex` returns where the object's current pose puts one mesh
vertex, in model space (the `.glb`'s own coordinates). The vertex is the
bind-pose one nearest the position you pass (within 5 cm), so a test names
vertices by where they sit in the rest pose, not by an index the importer is
free to reorder. It evaluates the pose on demand, so pose sharing and
animation LOD cannot hand it a stale skin; it also scans the mesh, so keep it
in tests and tools, not per-frame game logic.

```cpp
const float headTopBind[3] = {0.0F, 1.72F, 0.12F};
float now[3];
if (skinnedVertex(ctx, bodyIndex, headTopBind, now)) { /* compare */ }
```

To test **mesh LOD**, `animLod` reports what one part of an object last DREW:
the level (after the clamp to that part's chain), the vertex count, the
texture path and whether the draw really carries it, and the model-space box
of the drawn vertices - so a test can put a body at a distance and check that
it switched level, texture and all, without leaving its shape.

```cpp
AnimLodInfo info;
if (animLod(ctx, bodyIndex, 0, &info) && info.level == 2 && info.textureBound) { /* ok */ }
```

To **drive the pose from the game** - a ragdoll, a procedural pose - read
the nodes the body was last drawn with and hand the instance your own
matrices. `ctx.animNodeFind(obj, name)` gives a node's index, and
`ctx.animNodeModel(obj, node, m)` gives its MODEL-space matrix (column-major
`float[16]`, the frame `boneWorld` multiplies by the object's transform), as
of the last pose - evaluated on demand, like `boneWorld`, for a body that
followed another's skin or was never drawn (before this, a ragdoll could not
start on any body but the one of a lockstep crowd that skinned). `ctx.animPose(obj, matrices, count)` then sets an
**override**: `count` model-space matrices, one per node in the model's node
order (`count` must be the model's node count - read nodes from 0 until
`animNodeModel` returns false). While it is set:

- the clips are ignored - playback keeps its bookkeeping, but the palette is
  built from your matrices;
- the instance shares its pose with nobody (pose sharing compares clips);
- it is re-skinned **only when the matrices change**, so a body at rest
  costs no skin at all.

`ctx.animPose(obj, nullptr, 0)` clears it: back to the clips on the next
pose. `boneWorld` and `skinnedVertex` read the overridden pose like any
other. The engine side is `SkelInstance::setPoseOverride` /
`clearPoseOverride` / `poseOverridden`.

```cpp
float m[16 * 48];
int n = 0;
while (n < 48 && ctx.animNodeModel(body, n, m + 16 * n)) ++n;  // the death pose
// ... simulate, write new model-space node matrices into m ...
ctx.animPose(body, m, n);        // drive it
ctx.animPose(body, nullptr, 0);  // later: back to the clips
```

To drive a ragdoll TOWARD an animation (a **motorized** ragdoll - a game's
(see Ragdolls), keep playing clips on the overridden body as usual
(`playAnimation` / `seekAnimation` keep their bookkeeping under the override)
and read what they WOULD show with `ctx.animClipModel(obj, node, m)`: a node's
MODEL-space matrix from the clips - crossfade and held layer included - while
the drawn pose stays yours. It is evaluated on demand, once per clip change,
into its own buffer (`SkelInstance::clipNodeGlobal`); without an override it
equals `animNodeModel`.

A project that owns its `src/terrain_game.cpp` (removed the `// Generated by
TyraX` marker) does not get the template's wiring: add
`scriptCtx.boneWorld = &animBoneWorldThunk;`,
`scriptCtx.vertexModel = &animVertexModelThunk;`,
`scriptCtx.animLod = &animLodThunk;`,
`scriptCtx.animNodeFind = &animNodeFindThunk;`,
`scriptCtx.animNodeModel = &animNodeModelThunk;`,
`scriptCtx.animPose = &animPoseThunk;` and
`scriptCtx.animClipModel = &animClipModelThunk;` next to `resolveClip`, or the helpers
return false. Its `loadAnimModelAsset` should also end with
`ensureAnimLodTextures(i);` so authored LOD textures load with the model -
without it they load on the first frame that draws the model instead.
## The `.tskl` file (version 3)

The build writes the skeleton, the keyframe tracks and the mesh of each
animated model as a `.tskl`. Version 3 stores the mesh **compact**, the way a
static model's `.tmdl` does (`docs/model-pipeline.md`, "The compact `.tmdl`"):

- **One box per model.** Every part and LOD tier is quantized in the same
  isotropic box, a power-of-two step with an origin on its grid. A position is
  stored as `round(v / step) - round(origin / step)`. The step is the smallest
  power of two that holds the model in 65 534 steps: the combatent's is
  2^-15 m (0.03 mm).
- **Normals** are `int8` x 127 with a zero fourth component. The engine
  renormalises after decoding, so a normal is within one int8 step (1/127)
  of its authored value (0.0047 per component measured on the combatent).
- **UVs have a grid of their own per mesh**, chosen as in the `.tmdl` (the
  largest fraction from 12 down to 4 that keeps the range inside int16).
  Untextured parts carry no UVs.
- **Joints and weights** are unchanged: one byte each, four a vertex.

The engine decodes the mesh once, when a model is first instanced: the bind
cache and each instance's bind pose are still floats, so the compact form saves
the model's own memory (22 bytes a vertex against 40) and not yet the skinning
work. Parts that share a texture and a colour still merge at load; a merged
part whose UVs were on two grids is re-gridded onto one, which costs a
rounding of its UVs by at most one step of the coarser grid.

Versions 1 and 2 are the float layout and still load: the engine boxes and
quantizes them at load by the same rules, so a v2 file and its v3 twin convert to
the same bits. The layout itself is the comment above `writeTskl` in
`src/glbparser.cpp`; the engine's parse is `TsklLoader::parse`
(`vendor/tyra/engine/src/loaders/3d/tskl_loader/tskl_parse.cpp`).
`tools/verify-tskl-compact.cpp` checks both sides against each other, the v2
conversion, truncation, and the combatent's box.

## Build output

Builds write:

- `res/models/<name>.tskl` — mesh (and its LOD variants, when mesh LOD is on),
  skeleton (with its node names, for `boneWorld`), weights and compressed
  tracks;
- extracted model textures beside it, then quantized like other project images.

Sources are never modified and the output is deterministic.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Import says `unusable` | Re-export; the message names the malformed part |
| Invisible in game | Read `[anim bake]` in Output and check the `.tskl` exists |
| Flat-colour texture | Check extracted PNGs and power-of-two dimensions |
| Clip does not switch | Check the exact, case-sensitive clip name |
| Clip switch pops | Add a 0.2–0.4 s crossfade |
| Walk moves the object away | Enable **In place** on that clip |
| Palette-slot error | Simplify the rig below 256 bones/nodes |
| First assignment pauses | The scene preview now bakes in the background (a box placeholder shows for a moment); the Material Editor's own preview still bakes inline. Several seconds of bake usually means an FBX needs ufbx's slower posing path because it uses parent-scale inheritance or dual-quaternion skinning |

Animated models receive the scene's directional and ambient light. Point-light
bakes and the usable-object highlight rim currently apply only to static meshes.
