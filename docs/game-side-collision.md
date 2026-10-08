# Game-side collision

The engine moves the player with its **walker**: read the pad, integrate, resolve
against the scene, publish the camera. A project that wants *different* movement
— a tanky shooter controller, a character with momentum rules of its own, a
jetpack — does not have to keep that walker. Gunner (the FPS prototype in
`examples/`) replaces it wholesale: its own fixed-step simulation owns position,
velocity and stance, and the engine only renders the result.

What the engine will not hand over is the answer to "**what is solid here?**".
That answer lives in the walker's collision pass, and for a long time it was
`private:` in the generated `terrain_game.hpp`, so a game-side simulation could
only reimplement it. Reimplementation is the trap: the rules are not one box
test, and each of them is load-bearing somewhere else in the engine.

`ScriptContext::collideActor` is the seam. It applies **the walker's own pass**
to an attempted step, for a caller that keeps its own integration:

```cpp
// ctx is a ScriptContext&, inside a global Script::update
float nextX = self.x + vx * dt;
float nextZ = self.z + vz * dt;

// SEED the out params the way the walker's call site does.
float ground = terrainHeightAtScene(ctx.scene, nextX, nextZ);
float ceiling = 1e30F;

// Ask the engine: box mode + the 0.5 step-up, mesh mode ground ray + wall
// push-out, portal pass-through, the carried-object skip, lowest overhead
// surface. nextX/nextZ come back resolved.
ctx.collideActor(self.x, self.z, &nextX, &nextZ, feetY, eyeHeight, &ground,
                 &ceiling);

self.x = nextX;
self.z = nextZ;
if (self.y < ground) self.y = ground;                 // stand on things
const float maxY = ceiling - eyeHeight - 0.2F;        // eye clearance
if (self.y > maxY && maxY >= ground) self.y = maxY;   // head under things
```

## The contract

| Parameter | Meaning |
|---|---|
| `prevX`, `prevZ` | where the actor was **before** the step. The mesh push-out needs it: an actor that ends up past a wall's plane is ejected to the side it came from, not the side it was heading for. Pass the real previous position, not the same point twice. |
| `nextX`, `nextZ` | in: the attempted step. out: the resolved step. Either pointer may be null if the caller only wants the surfaces. |
| `feetY` | the actor's feet height **this frame**, before you apply the returned `ground`. |
| `eyeHeight` | feet → eye. Drives the waist probe, the mesh push-out height and the step span. |
| `ground` | **seed it** with the terrain height under the actor; the call only ever *raises* it (standing on a box, a model's top face, a procedural block). Seed `-1e30F` when a linked floor portal swallows the actor — that is what the walker does, and it is what makes a floor portal a hole to fall through instead of a wall. |
| `ceiling` | **seed it** with `1e30F`; the call only ever lowers it, to the lowest overhead surface (box undersides, an upward mesh hit, a procedural block above). |

`ctx.collideActor` is null before the first scene is built. A guard is not
optional in a script that can run before then: the null path leaves the position
untouched and reports "no surfaces" (`-1e30F` / `1e30F`) rather than inventing a
floor at `y = 0`.

## Why the rules are not yours to write

- **Markers, lights, decals, areas, volumes, belts** block nothing — one skip
  list (`objectCollides`), shared with the camera sweep and the physics pass.
  A game's own AABB loop over `ctx.objects` forgets this on the first decal.
- **0.5 step-up** is what makes a kerb walkable and a wall not. Step height is
  also the navmesh's (`docs/navigation-ai.md`) — change one without the other and
  bots climb what players cannot.
- **Portal pass-through**: while the walker stands in a linked portal's opening,
  the wall the opening is cut into stops colliding, and a *floor* portal opens
  the whole object. This state is per-frame and belongs to the engine; a
  game-side loop cannot see it.
- **Ceiling** is what keeps the camera out of geometry from below — the jump
  clamp the walker does every frame.

## Cost

One call is one pass over the scene objects (the same pass the walker makes, so
a project that replaces the walker pays what the walker paid — it does not pay
twice) plus, for mesh-mode objects, a downward ray and an upward ray. Call it
**once per fixed step of the actor that needs it**; it is not a per-frame
budget for hundreds of actors, so a game with a crowd should resolve bodies from
the same pass (one call, many actors' worth of bookkeeping) or fall back to the
coarse block field (`docs/physics.md`, Blocks Fill).

There is deliberately **no** force/shove here: shoving physics bodies out of a
walker's path (`pushPhysicsBodies`) stays with the walker. A game-side actor
that must push crates owns that behaviour explicitly.

## What is NOT here

- **Vehicle collision** (a car's swept box), which is the vehicle system's own
  pass.
- **Mesh-accurate tracing.** A `collision: mesh` object traces as its mesh
  **bounds** — the same box the walker would stand on. That is exact for the
  box-mode objects it matches, and coarse for a model whose silhouette matters
  to a bullet (a railing stops shots through the gaps). Reusing the walker's own
  `CollisionMesh` in the trace is the fix when a project needs it; it was left
  out because a bullet that stops on a railing's bounding box is a much smaller
  bug than a tracer that costs a triangle walk per shot.

## Tracing instead of stepping

For "does this shot reach that target", use `tyra::helper::traceLine` from the
same physics library (`physics/ray.hpp`):

```cpp
// Any origin, any direction - a muzzle, a bot's eye, a muzzle offset.
const tyra::helper::TraceHit hit =
    tyra::helper::traceLine(muzzle, muzzle + dir * 120.0F);
if (hit.hit && hit.objectIndex == targetIndex) { /* connected */ }
```

It traces the same world this page is about: the engine registers the current
scene's collidables and terrain at scene load, so a ray and a walker cannot
disagree. It is NOT the flow-graph `Raycast` node, which is eye-origin,
view-direction and bounding spheres only. `tyra::helper` is the home for the
helpers that ship — the traces and sweeps still to come. See
[backlog](backlog.md) for the coarse variant bots want, and
[debug-draw.md](debug-draw.md) to draw a trace.

## Where it lives

| Layer | Code |
|---|---|
| Public method | `TerrainGame::collideActor` (`src/templates.cpp`) |
| Rules it forwards to | `TerrainGame::collidePlayer` (the walkers' shared pass) |
| Script-facing thunk | `collideActorThunk` + `scriptCtx.collideActor` in the generated runtime (`src/game_templates.inc`) |
| Ray-trace providers | `TerrainGame::rayTraceCount/rayTraceBoxAt/rayTraceTerrainAt` + `rayTrace*Thunk` |
| Trace itself | `tyra::helper::traceLine` (`vendor/tyra/engine/src/physics/ray.cpp`) |
| Debug drawing of a trace | `tyra::debug::drawLine` ([debug-draw.md](debug-draw.md)) |
| Skip list / box builder | `objectCollides`, `objectCollisionBox` ([collision-boxes.md](collision-boxes.md)) |

Changes to the rules therefore change the walker, the camera sweep, every
game-side caller AND every tracer at once — which is the entire point of the
seam.

## The wiring trap

A project that owns `src/terrain_game.cpp` (delete the `// Generated by
TyraX` marker to take ownership) gets the **declarations and thunks** from the
templates but has to write the **assignments** itself: `scriptCtx.collideActor
= &collideActorThunk;` and `tyra::helper::setTraceWorld(...)` next to
`g_animGame = this;`. Without them everything compiles, runs, and collides with
nothing — which is exactly how Gunner's `object-collision` gym found it.