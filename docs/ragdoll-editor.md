# Ragdoll Editor — skeletons, ragdolls and motorized ragdolls

*Tools > Ragdoll Editor* is where a skinned character's **ragdoll** is made and
tuned: which parts of the body are physics bodies, how far each joint may bend,
and how a **motorized** ragdoll follows an animation. Its preview simulates the
asset while you edit it — let the body fall, shoot it, or play a clip through a
motor profile and watch it stagger, tumble and come to rest.

It is TyraX's counterpart of the Unreal Engine **Physics Asset Editor**:

| Unreal Engine | TyraX |
|---|---|
| Skeleton asset | `<model>.skeleton` — written by the import for every skinned model |
| Physics Asset: bodies | `.ragdoll` **bodies** (a segment pivot → end with a radius at each end) |
| Physics Asset: constraints (swing 1 / swing 2 / twist limits) | a body's **pitch / yaw / roll** limits, degrees from the bind pose, with a **hinge** option |
| "Generate All Bodies" (min bone size, primitive fit) | *New from model...* (min bone size, total mass) |
| Simulate / Simulate with physical animation | *Simulate*: **Ragdoll** / **Motorized** |
| Physical Animation Profiles (strength, local vs world) | **motor profiles**: per body *relative* (local) or *absolute* (world), and a 0..1 ratio |
| Ctrl+click poke / grab | Ctrl+click **punches** a body where you click; Ctrl+drag grabs it there and pulls it; *Shoot* |

## The assets

Both are JSON files beside the model in `res/models/`. They are editor-only:
the build never ships them (a game receives a ragdoll as its generated header,
below).

**`<model>.skeleton`** — the skin's bones (the palette joints, each with its
parent and its model-space bind matrix) and the model they come from. The model
import writes it for every skinned `.glb`/`.fbx` (and rewrites it when a
re-exported model's bones change). A ragdoll names its skeleton, not its model:
that is what will let another model with *exactly* the same bone structure share
the ragdoll and the animations later (see the backlog).

**`<model>.ragdoll`** — a Verlet *particle* ragdoll on that skeleton:

| Part | What it is |
|---|---|
| **particles** | points riding a bone (an offset in its bind frame), each with a radius (what touches the ground) and a mass |
| **bodies** | one per simulated joint: the segment *pivot → end*, hung on its *parent* body at the pivot. A body with a **side pair** (the pelvis on its hips, the chest on its shoulders) builds its own frame from it — the root needs one. Its **limits**: + pitch swings the far end toward *pitch dir*, + yaw toward *yaw dir*, roll twists it about its own axis; a **hinge** bends in pitch only (an elbow, a knee). *Fade once down* marks what carries the weight (legs, pelvis, chest) |
| **clusters** | particle groups held rigid (every pair linked): the pelvis with its hips, the chest with its shoulders. A child body whose pivot is not its parent's end must have it in the parent's cluster |
| **profiles** | motor profiles for a motorized ragdoll (below) |
| **solver** | the preview's simulation settings |
| **header / namespace** | where the build writes the ragdoll's C++ data header, and in which namespace |

The solver is the Gunner game's (`../gunner`, `.plan/35`, `.plan/39`): integrate
with gravity and damping, the motors, the ground contacts and their Coulomb
friction, the joints' frames and damping, the torso twist limit, then passes of
links, swing limits and contacts, then rolling resistance and sleep. The Solver
tab's defaults are that game's tuning, so the preview falls the way its bodies
do; a game keeps its own copy of these numbers in its code. A Solver value you
changed shows two arrows at the end of its row: back to the value last **saved**
(only while it differs from it) and, in yellow, back to the **default** (only
while it differs from that) - the tooltip names the value each one restores.

## Motorized ragdolls

A motorized ragdoll runs the animation and the physics at once: every step each
body's particles are pushed toward where the clip puts them, as hard as the
active profile says.

- **Absolute** — toward the clip's pose in the world (translation and rotation).
  A zombie: absolute everywhere, the legs at 1, the upper body at 0.5 — it walks,
  with a slack torso.
- **Relative** — toward the clip's pose *against the parent body as it is now*.
  It is an internal pair (the parent takes the reaction), so it shapes how a body
  falls and never lifts it. A death: relative at 0.75, the root at 0, playing a
  death clip — the body falls and tumbles (down a staircase, given one) in the
  clip's shape.
- The **ratio** (0..1) is the strength: 1 closes *motor gain* of the gap a step.
- Once the pelvis is down, the motors of the bodies marked *Fade once down*
  weaken to the solver's *When down* (a fallen body stops fighting the ground —
  without it a standing clip thrashes a body around on its back).

## Using the editor

The window has a toolbar and three panels: the asset tree on the left (its
*Section* dropdown picks Skeleton / Bodies / Particles / Profiles / Solver), the
preview in the middle and the selection's details on the right. Drag the
vertical dividers between the panels to resize them. The toolbar's rows are the
document (*Asset*, *Save*, *Revert*, *Undo* / *Redo*, *New from model...*), the
simulation (the *Animation* / *Ragdoll* / *Motorized* segments, *Clip*, *Motor
profile*, *Pause* / *Step* / *Reset*, the speed bar) and the push and view
(*Push* speed and direction, *Shoot*, *Gravity*, *Stairs*, and the *Show*
dropdown - mesh, wireframe, bodies, particles, links, bones, limits, angle axes,
motor targets - and *View* - reset the camera, the light).

Every control explains itself: hover it for a tooltip, or the **(?)** after a
label or a section title for what it means. The details read as an inspector -
the label on the left, the value filling the row - grouped under section titles
(*Body*, *Joint limits*, *Motors*; *Particle*; *Motor profile*; *Bone*; with
nothing selected, the asset's *Ragdoll*, *Contents* and *Generated header*). In
the preview the bodies are outlined capsules in the theme's accent, the selection
in its warning colour, and the body or particle under the mouse lights up with
its name beside the cursor; a pill at the bottom left says what is running
(mode, clip or clock, awake / asleep, the motors), and the **?** at the top right
lists the preview's mouse controls. A motor profile's bodies are a table of a
*Relative* / *Absolute* toggle and a strength bar each. The toolbar's *Motor
profile* and the *Profiles* list are ONE choice: picking a profile in either
marks it in the list, shows it in the details, makes it the one the preview
drives and starts the mode over (New profile and Duplicate pick the new one).

1. **Open or make a ragdoll.** The *Asset* combo lists every `.ragdoll`; the
   Asset Browser opens one on double-click (a `.skeleton` or an animated model's
   *Ragdoll Editor...* button opens its ragdoll, or offers to make one).
   *New from model...* generates one from a skinned model: a body on every bone
   whose segment is at least *min bone size* of the model's height (shorter
   bones ride their parent's body), radii and masses from the skinned vertices,
   side pairs and a bracing particle where a body has two lateral children,
   hinges and limits from the bone names (knee, elbow, spine, neck, ...), and two
   starter profiles. It simulates as generated; tuning it is your job.
2. **Edit.** Click a body or particle in the preview, the *Bodies* /
   *Particles* lists or the *Skeleton* tree; its details are on the right. A bone
   with no body offers *Add a body on this bone*; *Reset from the bone name*
   resets a body's joint (hinge, limits, directions). The selected body's swing limit is drawn as a
   yellow cone of allowed directions from its pivot, in its parent's frame, and
   its angle axes in the same colours as their limits in the details: **+pitch
   red**, **+yaw green** (none on a hinge), **roll blue** (the ring around the
   body's axis). On the root they are its frame's axes.
   *Undo* / *Redo* cover the asset; *Save* (Ctrl+S) writes it, *Revert* reloads.
3. **Simulate.** *Animation* plays the clip (the bind pose without one).
   *Ragdoll* starts the body from the clip's current pose and lets it go —
   *Shoot* pushes it at the chest, like a hit. Ctrl + left button on
   a body is one input with two outcomes: released without moving, a **punch**
   (the shot push at that point, into the screen along the mouse ray, at the
   push speed); moved past the drag threshold, a **grab** that pinches the body
   at that point and pulls it after the mouse (a spring, drawn as a magenta line)
   until the button is released. Either one in *Animation* mode switches to
   *Ragdoll* first. *Motorized* plays the clip through the chosen *Motor
   profile*: the green crosses are the motors' targets. Clicking the active mode
   again, or picking a motor profile (in the toolbar or the *Profiles* list),
   restarts it (the simulation from the pose it started at, the clip from 0).
   *Stairs* stands the body on a landing above ten 15 cm steps (in every mode);
   *Step* advances one fixed step; *Speed* slows it down. In the preview, RMB
   orbits (drag down to raise the camera over the body, up to look from below -
   the floor is one-sided, so from under it the body stays in sight), MMB pans,
   the wheel zooms (the **?** in the preview's corner lists them).

## In the game

A ragdoll whose *Header* is set (`inc/ragdoll/<name>.gen.hpp` by default) is
written into the project as a plain C++ data header on every build — the
`kParticles`, `kBodies` (with the derived `q`: the parent's end farther from
the pivot), `kBodyGroundFade`, `kLinks`, `kBones` (which body every skin joint
follows, and its bind matrix) and `kProfiles` tables. Gunner's
`inc/gunner/combatent_ragdoll.hpp` includes `ragdoll/combatent.gen.hpp`, and its
solver reads nothing else. *Write header now* writes it without a build.

## Command line

```
tyrax-editor --ragdoll <projectDir> skeleton <model>
tyrax-editor --ragdoll <projectDir> generate <model> [--force] [--min-bone 0.05] [--mass 75]
tyrax-editor --ragdoll <projectDir> header <asset.ragdoll> [--out path|-]
tyrax-editor --ragdoll <projectDir> sim <asset.ragdoll> [--seconds 4] [--push x,y,z]
             [--from clip[@t]] [--clip name --profile name] [--stairs]
tyrax-editor --import-model <projectDir> <model> --animated [--ragdoll] ...
```

`sim` is the preview, headless: it prints where the pelvis is every half second,
how far a joint went past its limit and whether the body came to rest — the
check a script can run after editing an asset. `--import-model --animated`
writes the `.skeleton` of a skinned model; `--ragdoll` also generates its
`.ragdoll` when there is none (never over one you tuned).

## Limits of this first version

- Bodies are particle segments (a sphere at each particle), not arbitrary
  capsules, boxes or convex hulls; collisions are with the floor and the preview's
  boxes, and between bodies of one ragdoll there are none.
- One skeleton per model: the import dialog that matches a skinned mesh to an
  existing skeleton (so models share animations and ragdolls) is queued.
- The engine itself has no ragdoll runtime: a game simulates the generated data
  (Gunner's core does), so the preview and the game agree only while the game
  keeps the same solver and settings.
