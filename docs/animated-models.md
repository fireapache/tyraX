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
a weapon on a running body does not trail it by a frame of travel. It returns
false - and leaves the outputs alone - for an object that is not animated,
a bone the file does not name, or a body never drawn yet: keep a fallback
placement. Bones are found by their glTF node name, stored in the `.tskl`'s
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

A project that owns its `src/terrain_game.cpp` (removed the `// Generated by
TyraX` marker) does not get the template's wiring: add
`scriptCtx.boneWorld = &animBoneWorldThunk;`,
`scriptCtx.vertexModel = &animVertexModelThunk;` and
`scriptCtx.animLod = &animLodThunk;` next to `resolveClip`, or the helpers
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
