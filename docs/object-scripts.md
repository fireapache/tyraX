# Object scripts (Unity-style components)

![The Scripts section on a selected object](img/object-scripts-section.png)

Custom game logic is written in C++ and compiled into the ELF by the normal
build - there is no interpreter on the PS2, so scripts run at native speed.
An **object script** is a class you write once and *attach* to any number of
scene objects, the way components work in Unity: every attachment becomes
its own instance at scene load, with its own member variables, its own
[parameters](#parameters) and a `self` pointer to the object it hangs on.
Attach `Spinner` to five crates and five independent spinners run; detach it
and the class costs nothing.

```
Properties > Scripts (attach "Spinner" to box-1, box-2; give box-2 a "speed")
   │  stored per object in <name>.tyra
   │  ("scripts": ["Spinner", { "name": "Spinner", "params": { "speed": 2.0 } }])
   ▼
src/gen/object_scripts.gen.cpp        ← regenerated on every build:
   (scene, object, class-name, params) table  +  a driver that owns the instances
   ▼
scene load: driver looks each name up in the TYRA_OBJECT_SCRIPT registry,
constructs one instance per attachment in the class's static storage and
calls onStart; every frame it refreshes `self` and calls onUpdate (and onUsed
when the player USEs the object); on the next scene load it calls onDestroy
```

`src/scripts/` is **exclusively yours** - engine-generated sources live in
`src/gen/`, so the editor's Scripts list shows only your files. Subfolders
are fine (`src/scripts/ai/guard.cpp` compiles and scans like any other; the
**New script...** dialog accepts `ai/guard`). Your `.cpp` files are **never
regenerated or parsed for compilation** - the editor only scans them
read-only for `TYRA_OBJECT_SCRIPT(...)` registrations to fill the attach
list. (Projects from an older editor still carry generated copies inside
`src/scripts/`; the first build with this version deletes them
automatically.)

## TyraX2 and custom rendering

The experimental [TyraX2 frame pipeline](tyrax2.md) records ordinary engine
render operations and supplies their ordering fences automatically. Custom
logic using those APIs needs no unconditional fence per node or script call.
Raw DMA sends, GS register writes and CPU GPU readbacks bypass that contract;
they are not automatically captured by enabling the project preference.

Change `RendererCore::setFramePipeline(bool)` between frames, before
`beginFrame()`. `getFramePipeline()` reports the **requested** setting, not
whether overlap is active in the current compatibility mode. Before a
between-frame CPU GPU readback, call `renderer.core.synchronizeFrame()` to
complete the pending presentation; the generated debugger capture already
uses it. This is not a mid-recording readback helper.

For custom packets, use the existing adapters in
`renderer/core/paths/path1/frame_submission.hpp` (`Tyra::frameSendPacket` and
its ordering helpers), or explicitly obey the queue/channel completion and
source-ownership contracts. `Tyra::Vif1Queue::drain()` is required before a
foreign direct VIF1 send. An idle DMA channel does not prove the GS has stopped
reading a texture. During recording, `sync.align3D()` / `sync.align2D()` can
record barriers instead of waiting on the CPU: do not use them as proof that
readback or resource destruction is safe. Keep referenced bytes and VRAM
alive until their actual readers finish.

## Quick start

1. Select an object and click **New script...** in **Properties > Scripts**
   (also on the Project panel's *Scripts* page, the `</>` icon on the left rail). The editor writes
   `src/scripts/<name>.cpp` from an attachable stub and attaches the class
   to the selected object.
2. Click **Open in VS Code** (Project panel > Scripts) - the generated
   `.vscode/c_cpp_properties.json` gives working IntelliSense against the
   engine and PS2SDK headers.
3. Edit the class:

```cpp
// bobber.cpp - created by TyraX. This file is yours.
#include "scripts/script.hpp"

namespace Mygame {

class Bobber : public ObjectScript {
 public:
  void onStart(ScriptContext& ctx) override {
    baseY = self->data.position[1];      // remember where we were placed
    height = param<float>("height", 0.5F);  // set per object in Properties > Scripts
  }

  void onUpdate(ScriptContext& ctx) override {
    t += g_frameDt;                      // wall-clock seconds, PAL == NTSC
    self->data.position[1] = baseY + height * sinf(t * 2.0F);
    self->dirty = true;                  // geometry changed - rebuild
  }

 private:
  float baseY = 0.0F;
  float height = 0.5F;
  float t = 0.0F;
};

TYRA_OBJECT_SCRIPT(Bobber);

}  // namespace Mygame
```

4. Attach the class to more objects with the **Attach script...** combo -
   each gets an independent instance. **Build & Run** (`F5`).

## The API

Everything lives in the generated `inc/scripts/script.hpp` (regenerated
while its ownership-marker line is intact, so the API tracks the editor).

### Lifecycle

| Override | Called |
|---|---|
| `onStart(ScriptContext&)` | Once per scene (re)load, before the first `onUpdate`. `self` is already valid and the [parameters](#parameters) are readable. |
| `onUpdate(ScriptContext&)` | Every frame while the owning scene is active - except while the object is **inactive** (its streaming layer is out), and never for a [start-only](#markers-start-only-scripts) class. Paused while a pausing menu is open, like all scripts. |
| `onUsed(ScriptContext&)` | The frame the player pressed USE on `self` (objects with **Usable** checked - same dispatch as the flow graph's *On Used* trigger). Not for a start-only class. |
| `onDestroy(ScriptContext&)` | Once per instance when the scene it belongs to is left or reloaded, before the destructor. Every instance of the scene gets its `onDestroy` first, then every destructor runs, so an instance may still talk to the others from `onDestroy`. `self` is `nullptr` there: the next scene's objects are already in place, so use what `onStart` captured. |

The order of a scene's life, per instance: constructor, `onStart`, `onUpdate`
every frame, `onDestroy`, destructor. Instances start, and are destroyed, in
the order described under [Order](#order).

Instances are constructed at scene load and destroyed when the scene is left
- member variables reset on every scene switch or reload. State that must
survive scenes or power-off goes in `ctx.saveValues` (see *Save data* in
the Project panel).

**No heap.** The code generator knows every attachment, so it reserves
**static storage** for each class: as many instances as the busiest scene
attaches, constructed in place with placement `new` at scene load and
destroyed in place at the next one. Nothing is allocated per attachment or
per scene load, and a class only needs a default constructor. A scene switch
reuses the same memory.

### `self` - the attached object

`self` is a `RuntimeObject*` for the object this instance is attached to
(`selfIndex` is its index in `ctx.objects`), refreshed by the driver every
frame. Mutate the object through it, then set `self->dirty = true` so the
geometry rebuilds:

| Field | Meaning |
|---|---|
| `self->data.position/rotation/scale` | The transform (rotation in degrees, X→Y→Z order). |
| `self->data.color` | RGB 0..1 - mesh tint on solids, particle tint on emitters, free parameter on empties. |
| `self->visible` | Show/hide (also mutes sound emitters, disables particle emitters). |
| `self->velocityX/Y/Z` | Rigid-body velocity (per-frame displacements) used by object physics. |
| `self->spin` | Angular velocity, degrees/frame (X/Y/Z) - tumbling. |
| `self->restFrames` | Physics sleep counter; write `0` after changing velocities to wake a sleeping body (`PHYS_SLEEP_FRAMES` = asleep). |
| `self->dirty` | Set true after changing any of the above so the frame's rebuild picks it up. |
| `self->active` | False while the object's streaming layer is not resident (the game manages it; scripts should use the Load/Unload Layer nodes). An inactive object's script is not ticked. |
| `self->anim*` | Animated-model playback state - prefer the `playAnimation`/`stopAnimation`/`animationFinished` helpers (see [animated-models.md](animated-models.md)). |

### Registration

`TYRA_OBJECT_SCRIPT(ClassName);` goes at file scope **inside your project
namespace** (the stub places it correctly). The stringized class name is
what the attach list shows and what the project file stores - rename the
class and existing attachments show a red *not found* until you re-attach
or rename back. One file may register several classes. Registration reserves
the class's static storage and reads two optional class constants, both
`public static constexpr` members:

| Constant | Meaning |
|---|---|
| `kScriptOrder` (`int`, default 0) | Where the class starts and ticks among the other classes: lower first. See [Order](#order). |
| `kStartOnly` (`bool`, default false) | The class is constructed, gets `onStart` and `onDestroy`, and is never ticked. See [Markers](#markers-start-only-scripts). |

## Parameters

An attachment can carry named values, set per object in **Properties >
Scripts** and read by the class - Unity's `[SerializeField]`, Godot's
`@export`. Two spawn points are the same class with different numbers:

```cpp
class SpawnPoint : public MarkerScript {
 public:
  void onStart(ScriptContext&) override {
    const int team = param<int>("team");
    const int sector = param<int>("sector", 1);        // 1 when the object sets none
    const bool hq = param<bool>("hq");
    const float radius = param<float>("radius", 4.0F);
    const char* tag = param<const char*>("tag");        // "" when unset
    mySpawnTable().add(self->data.position, team, sector, hq, radius, tag);
  }
};
TYRA_OBJECT_SCRIPT(SpawnPoint);
```

In the editor, each attached script has a **+ param** button. A parameter is
a name, a kind (`int`, `float`, `bool` or `string`) and a value. The editor
cannot read your C++, so it does not know which names a class asks for: type
the name the class passes to `param<T>()`. A red name is ignored by the build
(empty, or a repeat - the first of a repeated name wins).

In the project file an attachment with parameters is an object next to the
plain names, and a script without parameters stays the bare name it always was:

```json
"scripts": ["Spinner", { "name": "SpawnPoint", "params": { "team": 1, "sector": 2, "radius": 4.5, "hq": true, "tag": "north" } }]
```

The JSON token carries the kind: `2` is an `int`, `2.0` a `float`, `true` a
`bool`, `"x"` a string. Projects that use no parameter are written exactly as
before.

| Accessor | Reads |
|---|---|
| `param<T>(key, fallback = T())` | `T` is `int`, `float`, `bool` or `const char*` (any integer type reads as `int`, any floating type as `float`) |
| `paramInt`, `paramFloat`, `paramBool`, `paramString` | the same, spelled out |
| `hasParam(key)` | true when this attachment set that name |

A number converts to the other number kinds (a `float` read as `int`
truncates) and to `bool` (nonzero is true); a string is only a string, and a
missing name answers the fallback. The tables are `constexpr` data in
`src/gen/object_scripts.gen.cpp`, one per attachment that has parameters;
a lookup is a `strcmp` over that attachment's few entries, so read parameters
in `onStart` and keep the values, not in `onUpdate`. A string stays valid for
the whole run. There is no allocation and no `std::function` involved.

## Order

Who runs first is defined, not left to the link order.

**Each frame**, the game runs the global scripts (flow graphs, your `Script`
classes, the engine's own directors) in ascending `Script::scriptOrder`, and
the object-script driver is one of them:

| `scriptOrder` | Runs |
|---|---|
| below 0 | before everything left at the default |
| 0 (default) | the flow graphs, the engine directors and your `TYRA_SCRIPT` classes, in registration order |
| 1000 (`SCRIPT_ORDER_OBJECT_SCRIPTS`) | **the object-script driver**: scene (re)load handling, then every `onUpdate` and `onUsed` |
| above 1000 | after the object scripts |

So by the time any `onUpdate` or `onStart` runs, every default-order global
script has run for the frame. Scripts that share a `scriptOrder` run in the
order they registered - within a file that is declaration order, across files
the link order, which nothing promises. When one global script must precede
another, set `scriptOrder` in its constructor:

```cpp
class Spawner : public Script {
 public:
  Spawner() { scriptOrder = -10; }   // before the flow graphs and the object scripts
  void update(ScriptContext& ctx) override { /* ... */ }
};
TYRA_SCRIPT(Mygame::Spawner);
```

**Inside the driver**, instances start in ascending `kScriptOrder` of their
class (declare it in the class: `static constexpr int kScriptOrder = -100;`),
and instances of equal order start in *attachment order*: the scene's objects
in their scene order, then the object's scripts in the order of its list. The
same order is the order of `onUpdate` every frame, and of `onDestroy` and the
destructors at the next scene load. A class without `kScriptOrder` has order 0,
so projects that never declare one start and tick exactly as they did in
attachment order.

A scene (re)load is noticed by the driver on the first frame after it: that
frame it runs the old instances' `onDestroy`, destroys them, constructs the
new scene's, and runs their `onStart` - all inside its one `update`, after the
default-order global scripts.

## Markers (start-only scripts)

Many scripts exist to *describe* the scene: a spawn point, a capture point, a
waypoint. They read their parameters once, register with the game's own
systems and have nothing to do afterwards. Make such a class **start-only**
and it costs nothing per frame:

```cpp
class CapturePoint : public MarkerScript {     // MarkerScript sets kStartOnly = true
 public:
  static constexpr int kScriptOrder = -100;    // register before the scripts that read the table
  void onStart(ScriptContext& ctx) override {
    capturePoints().add(self->data.position, param<float>("radius", 6.0F));
  }
  void onDestroy(ScriptContext&) override { capturePoints().clear(); }
};
TYRA_OBJECT_SCRIPT(CapturePoint);
```

A start-only instance is constructed, gets `onStart` once and `onDestroy` at
scene unload, and lives in the class's static storage in between - it is just
never visited by the per-frame loops: no `onUpdate`, no `onUsed`, no virtual
call, no `self` refresh. Derive from `MarkerScript` (its `onUpdate` and
`onUsed` are `final`, so a marker that overrides one fails to compile instead
of silently never running) or set `static constexpr bool kStartOnly = true;`
in any `ObjectScript`.

## `ScriptContext` - everything a script can see

The same context global scripts receive; the fields you'll actually use:

| Field | Meaning |
|---|---|
| `ctx.engine` | The Tyra engine: `ctx.engine->pad` for buttons/sticks, renderer, audio. |
| `ctx.playerPosition` | Camera/player position this frame. |
| `ctx.objects`, `ctx.objectCount` | All runtime objects of the active scene (your `self` is `ctx.objects[selfIndex]`). |
| `ctx.skyColor` | Write to retint the sky/clear color. |
| `ctx.teleport*` | Set `teleport = true` + `teleportPos`/`teleportYaw` to move the player. |
| `ctx.usedObject` | Index of the object USEd this frame (-1 none) - `onUsed` is the ergonomic form. |
| `ctx.saveValues`, `ctx.saveTexts` | Named values persisted in memory-card saves. |
| `ctx.openMenu`, `ctx.menuEvent` | Open a game menu / react to a menu entry's flow event. |
| `ctx.rumble*` | Pad vibration request — prefer the `padVibrate(ctx, big, small, seconds)` helper (big = heavy motor 0..1, small = the buzz motor, seconds > 0 auto-stops, 0 = until the next call). |
| `ctx.scene`, `ctx.requestScene` | Active scene index; write an index to switch scenes after this frame. |
| `ctx.sceneGeneration` | Bumps on every scene (re)load - object scripts rarely need it (their lifetime already tracks it). |

Handy globals from `scene_data.hpp` (included via `script.hpp`):
`g_frameDt` (seconds per frame - multiply all speeds by it so PAL and NTSC
play identically), `g_frameScale`, the `SCENE_*` accessor macros.

## Empty objects

**+ Add object > Empty** inserts a pure transform: a small sphere marker in
the viewport, *nothing* in the game - no geometry, no collision, no USE
target. It exists so scripts and flow graphs have something to hang on:

- an anchor for a script that manages a spot in the world (spawner, trigger
  zone via distance check, cutscene camera target),
- a waypoint: flow-graph *Move Object* nodes and scripts can read/write its
  position like any object,
- its editable **color** never renders in-game, so it's a free per-object
  parameter (`self->data.color`) - e.g. one "SkyTint" script attached to
  differently-colored empties per scene. For anything beyond three numbers,
  use [parameters](#parameters).

Rotation, scale and *Save state* work like on any object; collision is
always off.

## Global scripts

The pre-existing flavor still works and is the right tool for logic that
isn't about one object: derive from `Script`, override `init`/`update`,
register with `TYRA_SCRIPT(MyScript);` (at file scope, *outside* the
namespace or inside it with a qualified name - see the FPP example script).
One instance for the whole game, `update` runs every frame in every scene, at
the point `scriptOrder` gives it (see [Order](#order)).
Existing projects keep compiling unchanged; the flow-graph compiler and the
object-script driver are themselves global scripts under the hood.

If you own `inc/scripts/script.hpp` (you deleted its first line) the editor no
longer regenerates it: copy the new definitions from a project created by this
editor, or delete the file and let it be regenerated.

## Performance

- Scripts are compiled MIPS code, not interpreted - an empty `onUpdate` is
  one virtual call per attached instance per frame (nanoseconds on the EE).
- A [start-only](#markers-start-only-scripts) class costs no per-frame work at
  all, and an inactive object's script is skipped before its virtual call.
- Class-name lookup happens **only at scene load**; the per-frame path
  touches no strings. Scene load allocates nothing.
- Scenes with no attachments cost an empty loop; unattached classes cost
  only their code size. A class's static storage is `sizeof(class)` times the
  most instances any one scene attaches.
- The real costs are what the script *does*: `self->dirty = true` rebuilds
  that object's vertex buffers this frame (fine for a handful of moving
  objects - the flow graph's Move/Recolor nodes do the same), and per-frame
  math over `ctx.objects` scales with scene size.

## Verifying

`tools/verify-object-scripts.cpp` pins all of the above on the host: how the
project file reads and writes both spellings of an attachment, the rows and
parameter tables the generator emits, and - by compiling the generated
`script.hpp` and `object_scripts.gen.cpp` with a stub engine and running them -
start order, class order, markers, inactive objects, `onDestroy`, storage
reuse, that nothing allocates, and where the driver sits among the global
scripts. The build line is in its header
(`./verify-object-scripts` prints `ALL n CHECKS PASSED`).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Class missing from the **Attach script...** combo | No `TYRA_OBJECT_SCRIPT(Name);` in a `src/scripts/*.cpp` file (the scan is textual and per-file - check spelling and that the file is saved). |
| Attached entry shows red *"not found"* | The registration disappeared (file deleted, class renamed). The game skips it with a log line; remove the entry or restore the class. |
| `Object script not registered: X` in the game log (Debug panel, `bin/log.txt`) | Same as above, seen at runtime - the scene loads fine, that attachment is skipped. |
| `Object script has no storage left: X` in the game log | An attachment the generator did not count, so its class's storage is full (the storage is sized from the project at generation time). Build again from the editor; this never happens from a matching build. |
| Script does nothing | Is it attached to an object *in the active scene*? Attachments are per object per scene. |
| `onUpdate` stopped for an object | The object is inactive (its streaming layer is out of memory); the script resumes with the layer. |
| A marker's `onUpdate` does nothing | A start-only class is never ticked - that is its point. Use a plain `ObjectScript` if it needs a tick. |
| `param<T>()` returns the fallback | The attachment sets no parameter by that exact (case-sensitive) name, or the value's kind cannot convert (a string read as a number). A red parameter name in Properties is ignored by the build. |
| `self` is null in `onDestroy` | By design: the next scene's objects are already in place. Keep what you need from `onStart`. |
| State resets when the scene changes | By design - instances are recreated per scene load. Persist through `ctx.saveValues` or a `static` member (statics survive scene switches but not power-off). |
| Movement speed differs between PAL and NTSC | Multiply per-frame deltas by `g_frameDt` (seconds) or `g_frameScale` (50 Hz-relative) - never hardcode per-frame steps. |
| `onUsed` never fires | The object needs **Usable** checked (Properties), the player must be close and press USE; menus swallow input while open. Markers (spawn, player, lights, empties) cannot be usable, and a start-only class is never told. |
| Nothing moves while a menu is open | Menus with *Pause gameplay* checked pause all scripts - that is the pause menu working as intended. |
| Editing the `.cpp` does nothing in the running game | Scripts compile at build time - **Build & Run** again (no hot reload on the console). |
