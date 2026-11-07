# Render slots

Render slots reserve the memory of skinned vertices once, at level load and per level of detail, and
give it only to the bodies a view sees: a skinned body that no view looks at owns no skinned vertex
memory at all, and no per-body float mesh arrays either.

Without slots every skinned body makes its own streams (or pose buffer) the first time it is drawn
and keeps them for as long as it lives - a match that fields twenty-four fighters pays for twenty-four
bodies' worth of vertices whether the camera sees two of them or all of them - and each body is made
with the float mesh arrays of its bind pose (48 bytes a corner, 160 KB for a LOD0 body) that it frees
only when it first draws. The peak of the `Skinning` budget was the moment every body existed and none
had drawn yet. With slots the budget is a fixed pool, the peak is the pool, and what the bodies cost
is the bodies a view sees.

Slots are opt-in: a project that sets `Settings::renderSlotBytes` in its memory registry gets them,
one that does not draws exactly as before (the per-body streams of
[animated-models.md](animated-models.md), "Skinning into compact streams" and "Skinned streams with
indexed corners"). They need the memory manager (`TYRA_MEMORY_MANAGER`) and the game template's
animated-model pass.

## Tiers

A tier is a level of detail of the skinned mesh: tier 0 the full mesh, tier 1 and 2 the decimated
levels the model carries (the template's mesh LOD distance picks the level a body asks for: tier 0
within it, 1 beyond, 2 beyond twice). Each tier owns its own slots, sized for the largest model at
that tier:

- `slotSideBytes(model, tier)` is the bytes of ONE side of a slot: every part of the model at the
  level the tier names, in the format the part is placed in (below), boxes included;
- a slot is two sides - a pass writes the side no frame reads, then flips it to be the drawn one;
- `Settings::renderSlotBytes[tier]` is the bytes of the `Skinning` budget reserved for the tier; the
  layout carves `floor(bytes / slotBytes)` slots out of it. A carve the budget refuses ends the tier
  there: the tier gets the slots it has.

The layout is made once per scene, by the first animated object created in it (the models are
loaded by then), under the `Skinning` budget scope; a log line says what was carved:

    [SLOTS] tier 0: 8 x 35232 B, tier 1: 10 x 17504 B, tier 2: 26 x 8800 B, reserved 685696 B

If nothing could be carved, every body of the scene keeps its own streams (the line says so).

## Who takes a slot

Each view pass asks for a slot for each body whose box touches its frustum, nearest first, with the
tier its distance names (`SkelRenderSlots::acquire(body, tier, frame)`):

- **A body keeps the slot it holds** when that serves it - the same tier, or a better one it needed
  within the last 30 frames (a body one view of a split screen sees near and the other sees far
  keeps the near tier instead of swapping slots twice a frame).
- **A full tier hands out the next cheaper tier** (LOD0 full: LOD1; that full: LOD2). `fellBack`
  counts it. A body holding a cheaper slot than it asks for takes the better one the moment one is
  free.
- **Only past the cheapest tier is a body refused** (`refused` counts it): it holds no memory, so it
  is not drawn that frame. The cheapest tier is sized so that never happens (below).
- **A slot is held for `kSlotHoldFrames` (30) frames** after the last pass that touched it, then
  goes back to the pool at the end of a frame (`frameEnd`). A body that leaves a view and returns
  within the hold finds its slot - and its pose - as it left it; a body that returns after it has
  lost it takes a slot that is new to it.
- **A slot whose memory a frame still reads is not given to a new body.** The two sides of a slot
  are registered as immutable spans, so the frames that draw them borrow them in place; a free slot
  is handed out only when no recorded frame names either side (`Vif1Queue::immutableSpanReady`),
  and a pass writes the spare side only when no frame names it.

A body that is drawn is skinned (`SkelInstance::skinIntoSlot`) in the pass that first gives it a
slot, whatever its animation cadence says, and afterwards when its pose is due: the cadence of far
bodies (`TYRA_SKIN_CADENCE`, `animLod`) is unchanged. A slot written this frame is not written again
in the split screen's second pass.

Bodies that share a pose (`poseEquals`) share the owner's slot: a follower takes none.

## The split screen

The second view's pass runs the same loop. A body only that view sees is acquired and skinned in
that pass - the stale second view of bodies that were never posed because the first pass did not
see them is gone - and a body both views see is skinned once, in the first pass.

## What a slot holds

The payload of a slot is a small interface (`SkelInstance::slotSideBytes`, `bindSlot`,
`skinIntoSlot`), not a format. A part is placed as either of the engine's skinned streams:

- the **indexed pose buffer** (`TYRA_SKIN_INDEXED`, [animated-models.md](animated-models.md),
  "Skinned streams with indexed corners") when the part is the indexed program's class: the
  unique vertices once per package, int16 and int8, a few KB a part; the model's static blocks are
  shared by every body and live outside the slots;
- the **plain compact stream** (`StaPipCompactStream::allocateIn`, the stream placed in the slot's
  memory, boxes included) for every other part, and for every part when the flag is off.

`bindSlot` places the parts once for a slot and a model (the ST run of a plain stream is written
then; it never changes between poses); a body of another model re-places them. A slot's bytes are
carved from the budget at layout, never allocated per body.

## What a body without a slot owns

Nothing in skinned memory. `SkelInstance::create(model, floatMesh = false)` - what the template
asks for when slots are on - makes no float mesh arrays: the mesh keeps its bounding boxes (made
from the two corners of the bind pose's box) and two-vertex stand-ins for the arrays, `ensurePose`
and `lodArrays` have nothing to skin or return, and the bags are made without vertices, normals or
texture coordinates. `SkelInstance::skinnedBytes()` reports what a body owns: 0 for a body on a
slot, against 17 616 bytes at LOD0 (8 744 at LOD1, 4 392 at LOD2) for a body with an indexed buffer
of its own and 160 KB more for a body that still has the float arrays.

## Sizing

Per tier, bytes = slots x slotBytes of the largest model at the tier. The census a project runs
(the `[SLOTS]` lines every five seconds in development builds) gives the peaks: slots in use per
tier, `fellBack`, `refused`.

- **The cheapest tier** is sized for the most bodies the hold keeps at once: every body drawn within
  30 frames. For a match that is the number of fighters - a body cannot hold two slots - so a
  fighter count is a ceiling that never refuses. `refused` must stay 0.
- **The better tiers** are sized for quality, not safety: the bodies near enough to be worth the
  triangles. A body that finds its tier full is drawn at the next cheaper one.

The combatent's slot is 35 232 / 17 504 / 8 800 bytes at tiers 0 / 1 / 2 with the indexed pose
buffer.

## The registry setting

`Tyra::Memory::Settings::renderSlotBytes[4]` (`registry.hpp`), set in `registerGameBudgets` beside
the budgets' regions. Index = tier; the fourth is unused. All zero (the default): no slots. The bytes
come out of the `Skinning` budget, so its region must hold them with the bind data and the models'
shared blocks.

## Verifying

`python tools/verify-tyrax2-host.py --only slots` runs `tools/verify-skel-render-slots.cpp` against
the engine's own code: the hold (back at 29 frames keeps the slot, at 31 has lost it), the fall back
LOD0 -> LOD1 -> LOD2, 100 000 random frames of 40 bodies in two views with the cheapest tier never
refusing (and an undersized one refusing), a body only the second view sees, the carve arithmetic and
a refused carve, and a stream placed in a slot's memory allocating nothing on the heap.
