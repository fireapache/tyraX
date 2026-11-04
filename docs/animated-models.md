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

On the EE (`TYRA_SKIN_FIXED_MMI`, on by default) the kernel's dot products run
as PMADDH with the operands in 16-byte lane vectors the fold writes (`aV`, `rV`,
`bW`); the shifts, rounds and weighted sums are scalar integer ops, since the
EE's variable word shift (PSRAVW) moves words 0 and 2 only. The first pass per
process runs the kernel and the scalar blend corner by corner over the part and
logs `SKINMMI self-check: byte-equal` (or the first differing corner's inputs).
Measured on the six Gunner gyms: all pass, combatent-lods judges 1162 frames.

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
