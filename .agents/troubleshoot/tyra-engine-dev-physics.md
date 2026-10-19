# troubleshoot/tyra-engine-dev-physics.md

The XZ-grid triangle collider, the contact broad phase and their queries. Symptom first.

## Collision queries paid for every border triangle

**Cause:** `physics/CollisionMesh` is an XZ-grid triangle collider. The grid clamps an
outside query onto its edge cells, so a query that starts outside the mesh would otherwise
test every border triangle.
**Fix:** both queries test the mesh's own box FIRST — keep that early-out exact: the
sphere's reach is radius plus the **0.6** crossing band.

## The rigid-body solver needs the hit triangle's normal

**Cause:** the per-corner contacts need a mesh-local normal.
**Fix:** `raycast(..., outNormal)` also returns the hit triangle's mesh-local normal
(docs/physics.md). `Ray::intersectTriangle` is the triangle primitive underneath it.

## A collision, sweep, physics or trace query misses an object it used to hit

**Cause:** every one of them takes its candidates from `physics/BroadPhase`
(`broad_phase.{hpp,cpp}`, docs/collision-boxes.md "The broad phase": a height-aware
XZ grid or a loose octree behind one interface), and a candidate set that drops an
object is a wall walked through - silently. Its contract is a SUPERSET (never fewer),
ascending (the walker's pushes depend on order).
**Fix:** turn on `kBroadPhaseVerify` in the generated `game_runtime.gen.hpp` template:
it re-runs every walker call over all objects and checks every query against the
objects' current boxes, logging `[BROADPHASE] ...`. Before changing the structure,
fuzz it against brute force from a scratch host harness - it compiles on the host
(only `<vector>`).
