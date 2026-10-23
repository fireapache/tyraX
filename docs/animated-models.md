# Animated models

TyraX imports skeletal `.glb` and `.fbx` models, previews their clips and bakes
them into the PS2-native `.tskl` format.

![The Animation Editor showing the model preview, clip list and non-destructive trim controls.](img/animation-editor.png)

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

Open **Tools > Animation Editor**. Select a model and clip (the list has a
name filter), then preview, pause, scrub or switch to wireframe.

Clip edits are non-destructive:

- **Name in game** gives a short runtime name.
- **Time scale** fixes one clip that plays too quickly or slowly.
- **Trim start/end** cuts unwanted frames.
- **In place** removes root translation for locomotion.
- **Loop by default** sets the initial object behaviour.

Project Preferences also has a global animation FPS for exports whose authored
rate was interpreted incorrectly. Fix a single clip with Time scale; use the
project setting only when every clip is wrong by the same factor.

The same window's **Imported clips** section borrows clips out of a *different*
rigged file — a Mixamo download, another export from an animator — and puts them
on this model, matching bones by name and keeping this model's own proportions.
An imported clip then behaves as one of this model's clips everywhere, including
every field above. See [animation-import.md](animation-import.md).

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

A project that owns its `src/terrain_game.cpp` (removed the `// Generated by
TyraX` marker) does not get the template's wiring: add
`scriptCtx.boneWorld = &animBoneWorldThunk;`,
`scriptCtx.vertexModel = &animVertexModelThunk;`,
`scriptCtx.animLod = &animLodThunk;`,
`scriptCtx.animNodeFind = &animNodeFindThunk;`,
`scriptCtx.animNodeModel = &animNodeModelThunk;` and
`scriptCtx.animPose = &animPoseThunk;` next to `resolveClip`, or the helpers
return false. Its `loadAnimModelAsset` should also end with
`ensureAnimLodTextures(i);` so authored LOD textures load with the model -
without it they load on the first frame that draws the model instead.

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
