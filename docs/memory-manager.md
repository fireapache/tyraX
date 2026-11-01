# The memory manager

The memory manager gives a game strict, visible memory budgets: every region of
memory has a name, a size and a counter, and when a request cannot be served the
engine says which budget ran out and who asked for it, instead of the heap
quietly running dry and the console going to a black screen.

The module is being built one piece at a time and is not yet wired into the
engine: nothing in the engine or in a generated game calls it, and a project
builds and runs exactly as before. This page describes what exists so far - the
region heap, the failure path, the budget registry, the per-frame scratch and
the host verifiers that test them. The sources
are in `vendor/tyra/engine/src/memory/` and `vendor/tyra/engine/inc/memory/`;
everything there outside `src/memory/ps2/` is portable C++ that includes no PS2
header and assumes nothing about the size of a pointer, so the same code runs on
the EE (32-bit pointers) and under the host verifiers (64-bit).

## The region heap

`Tyra::Memory::RegionHeap` (`inc/memory/region_heap.hpp`) is a non-moving
allocator over one byte range you give it with `init(base, bytes)` - `base`
16-aligned, `bytes` a multiple of 16. It never grows, never asks the system for
memory and keeps no state outside the object and the range, so one `RegionHeap`
per region is the whole idea.

It is a two-level segregated-fit allocator (TLSF). Free blocks sit in lists by
size class - a power of two, split into 8 steps - and two small bitmaps say which
lists are not empty, so `release` takes constant time and so does `allocate`,
except for one walk of a single size-class list when only that class can hold
the request. A released block is merged with a free neighbour on either side at
once, so two free blocks are never adjacent. An ordinary request (alignment 16)
is served whenever some free block is big enough, not only when a block of a
"safe" class exists.

Every block starts with a 16-byte header: its size and two flags (free,
over-aligned), the size of the block before it, a 16-bit **budget** id and a
32-bit **owner** id. The payload follows the header and is always 16-aligned.
The budget and owner of any pointer the heap handed out are one read away
(`budgetOf`, `ownerOf`), and `setOwner` changes the owner later.

| call | what it does |
|---|---|
| `allocate(size, align, budget, owner)` | a payload of at least `size` bytes aligned to `align` (a power of two; below 16 means 16). `nullptr` when nothing fits - and then nothing has changed |
| `release(p)` | gives the block back and merges it with its free neighbours; `nullptr` is ignored |
| `usable(p)` | the payload bytes the block really has |
| `contains(p)` | whether `p` lies inside the range given to `init` (a range test only) |
| `owns(p)` | whether `p` looks like the payload of a live block of this heap: inside the range, 16-aligned, with a header that is not free and agrees with its neighbours' sizes. A best-effort sanity check, not a proof - it cannot see a stale pointer into a block that was released and reused |
| `slidable(p)` / `slideDown(p)` | see below |
| `used()` `peak()` `freeBytes()` `capacity()` | totals in bytes of whole blocks, headers and slack included: `used() + freeBytes() == capacity()`; `peak()` is the most `used()` has been |
| `largestFree()` | the largest allocation (alignment 16 or less) that would succeed now |
| `forEachBlock(f)` | calls `f(BlockInfo)` for every block in address order, free ones included; the heap must not change meanwhile |
| `validate()` | walks every block and every free list and returns whether they agree with the totals and the bitmaps |

An alignment above 16 makes the allocation reserve up to that much room more. The
gap in front of the payload becomes a free block of its own when it is at least
32 bytes; a gap of 16 bytes joins the allocated block before it (and at the very
start of the range, where there is none, the payload moves on to the next
boundary). Such a block is flagged over-aligned and is never moved.

**Sliding.** The heap never moves a block by itself. `slideDown(p)` is the one
operation that does, and only when asked: if the block in front of `p`'s is free,
`p`'s block (header and payload) is copied to the start of that free block, the
room it vacated becomes one free block (merged with the free block behind it, if
there is one) and the new payload address is returned; otherwise it returns
`nullptr` and changes nothing. Whoever owns the pointer must fix it up, and a
block allocated with an alignment above 16 (`slidable(p)` is false) is refused.
Repeating it over the blocks in address order packs the used blocks toward the
start of the range.

**Checks.** `TYRA_MEMORY_CHECKS` (default 0, set it with `-D`) turns on the
integrity checks: `release` of a pointer that is not a live block of this heap
(outside the range, misplaced, released twice - detected on a best-effort basis),
an alignment that is not a power of two and a bad range in `init` are reported
through `fail` (below) instead of corrupting the heap or returning quietly. With
the checks off, those calls are the caller's responsibility; an alignment that
is not a power of two simply returns `nullptr`.

## Reporting a failure

A failure of the memory manager is reported one way, through
`Tyra::Memory::fail(const Failure&)` (`inc/memory/failure.hpp`); today the region
heap's checks are its only callers. A `Failure` is four fields: `what` (the kind, e.g. `"budget full"`), `name` (the
budget, pool or region concerned), `size` (the request in bytes, 0 if it has
none) and `caller` (the return address of the code that asked, or 0). `fail`:

1. formats one line into a buffer on the stack - nothing is allocated, which
   matters because the failure is often an allocation that could not be served:

   ```
   [MEM] FAIL <what> name=<name> size=<n> caller=0x<hex>
   [MEM] FAIL budget full name=Renderer size=65536 caller=0x1c2a40
   ```

   (`size` is decimal, `caller` lowercase hex; a missing text prints as `?` or
   `-`, a control character as a space, and a very long `what` or `name` is cut
   so the numbers at the end always fit);
2. prints it - to `stderr` unless `setFailureOutput` installed another function;
3. calls the game's handler, if `setFailureHandler` set one (draw a message,
   flush a log). A failure raised from inside the handler skips the handler, so
   it cannot recurse;
4. halts - `abort()` unless `setFailureOutput` installed another function. A halt
   function does not return.

`setFailureOutput(print, halt)` takes the two functions together; a null argument
keeps that default. `print` receives the line without a newline. The line is the
same on every platform, so a log or a tool reading it needs one parser.

## The budget registry

`Tyra::Memory::Registry` (`inc/memory/registry.hpp`) is where a game says what
memory it needs: a list of **budgets**, a list of **pools** and a few
**settings**. It allocates nothing. From its rows it works out how much fixed
memory the game needs, where each piece of that memory goes, and whether it all
fits the RAM there is - the boot-time **fit check**. The engine registers its own
rows when the registry is made; a game overrides those it wants to size and adds
its own.

### Budgets

A budget is a named share of memory. A row (`BudgetDesc`) has a `name`, a `kind`
(`System`, `Logic`, `Render`, `Asset` or `Scratch`) and two sizes:

- `regionBytes` - the budget's own region (a scratch's size). `0` means the
  budget has **no region of its own**: it is to be served from the `System`
  budget's region, charged to itself and not capped. That is the **measure mode**
  every engine row starts in, so the engine's default is one heap, as without a
  manager, while the budget's use is still counted by name. The registry
  reserves nothing for such a budget. A region is handed to `RegionHeap::init`,
  so give it a multiple of 16;
- `extensionBytes` - how much of the movable heap the budget's pools may take as
  extension buckets. The heap is the sum of these over all budgets.

A budget is named by a `BudgetId`. The engine's ids, registered by `Registry()`
with both sizes 0:

| id | name | kind |
|---|---|---|
| `Budget::System` (0) | `System` | System |
| `Budget::FrameScratch` (1) | `FrameScratch` | Scratch |
| `Budget::LoadScratch` (2) | `LoadScratch` | Scratch |
| `Budget::Renderer` (3) | `Renderer` | Render |
| `Budget::Skinning` (4) | `Skinning` | Render |
| `Budget::BakedStreams` (5) | `BakedStreams` | Render |
| `Budget::Audio` (6) | `Audio` | Asset |
| `Budget::Level` (7) | `Level` | Asset |

A game's budgets start at `Budget::FirstGame` (32); ids go up to, not including,
`Budget::Max` (64), and 8 to 31 are left free for the engine to grow into.

`budget(id, name, kind)` returns the row to override, adding it when the id is
not registered yet. Giving a `name` (re)declares the row's name and kind; leaving
it out returns an existing row untouched (and names a new one `unnamed`):

```cpp
using namespace Tyra::Memory;

Registry r;
r.budget(Budget::Renderer).regionBytes = 2u << 20;  // size an engine row
r.budget(Budget::FirstGame, "Actors", Kind::Logic).extensionBytes = 64 * 1024;
```

`findBudget(id)` reads a row (null when it is not registered).

### Pools

A pool holds the entries of one component type: a first bucket that is part of
the fixed memory, and extension buckets that come from the movable heap. A row
(`PoolDesc`) is `{name, budget, typeSize, align, firstCount, extensionCount,
maxExtensions, policy}`. `pool(desc)` adds it and returns its `PoolId`; ids are
dense from 0 and at most `kMaxPools` (64) pools fit. `poolCount()` and
`poolDesc(id)` read the rows back. The policy - `Refuse`, `RecycleOldest` or
`Halt` - says what the pool does when it is full and its budget cannot take
another extension; the registry only needs it for sizing.

`bucketBytes(id, count)` is the size of one bucket of `count` entries:

| part | bytes |
|---|---|
| entries | `roundUp(count * stride, 16)`, where `stride = roundUp(max(typeSize, 4), max(align, 4))` |
| live bitmap | `roundUp((count + 7) / 8, 16)` |
| recycle stamps, only for `RecycleOldest` | `roundUp(count * 4, 16)` |
| generations, only with `TYRA_MEMORY_CHECKS` | `roundUp(count * 2, 16)` |

`poolFixedBytes(id)` is `bucketBytes(id, firstCount)`. Both are 0 for an id the
registry never issued.

### Settings

`registry.settings` holds `ptrTableCapacity` (4096, the entries of the pointer
table), `defragBytesPerFrame` (64 KB), `newlibReserveBytes` (256 KB, what the C
library keeps for itself after the block) and `systemBytes` (0: the system
region takes everything that is left).

### The layout and the fit check

`fits(availableBytes, layout)` places everything fixed in **one block** and
checks that the block, plus the newlib reserve, is not more than
`availableBytes`. The pieces follow each other in this order, each starting on a
multiple of 64 bytes (`Layout` holds the offsets from the start of the block):

1. every pool's first bucket, in pool order (`poolOffset[id]`) - a pool whose type
   asks for more than 64 bytes of alignment starts on a multiple of that;
2. the region of every budget that has one, in id order (`regionOffset[id]`);
3. the pointer table (`ptrOffset`, `ptrBytes`: two words an entry, twice over, so
   a 32-bit pointer halves it);
4. the movable heap (`heapOffset`, `heapBytes`: the sum of the budgets'
   `extensionBytes`, rounded up to 64);
5. the system region (`systemOffset`, `systemBytes`).

`fixedBytes` is everything before the system region, plus the system region when
`settings.systemBytes` gave it a size; with `systemBytes` 0 the system region
takes `availableBytes - fixedBytes - newlibReserveBytes`. `blockBytes` is the
whole block (`systemOffset + systemBytes`) and `blockAlign` the alignment its
start must have: 64, or the largest alignment a first bucket asks for. All sizes
are summed in 64 bits and cannot wrap around into "fits".

`fits` answers `false`, and `error()` says why, when:

- a pool names a budget that is not registered;
- a pool's `align` is not a power of two, or is above 128 for a pool that takes
  extensions from the movable heap (`extensionCount` and `maxExtensions` both
  non-zero);
- `pool` or `budget` was called with more than `kMaxPools` pools or with an id
  from `Budget::Max` up;
- `fixedBytes + newlibReserveBytes` is more than `availableBytes`
  (`registry does not fit: ...`).

`fits` never calls `fail`: it reports and the code that starts the manager
decides what to do. It fills `layout` even when it answers `false`, so the table
can be printed for a registry that does not fit, and `error()` is empty after an
answer of `true`.

### The table

`printTable(print, layout)` calls `print` once per line, without a newline: a
line per budget (by id), a line per pool, then the totals. Sizes are in KB,
rounded up; a budget in measure mode prints `region=0`.

```
[MEM] budget System kind=System region=0 heap=0
[MEM] budget FrameScratch kind=Scratch region=256 heap=0
[MEM] budget Renderer kind=Render region=2048 heap=0
[MEM] budget Actors kind=Logic region=0 heap=64
[MEM] pool actor budget=Actors first=32 ext=8x4 policy=Refuse fixed=4
[MEM] pool decal budget=Actors first=64 ext=0x0 policy=RecycleOldest fixed=4
[MEM] total fixed=2439 system=30074 reserve=256
```

(32 MB of RAM and 4-byte pointers, as on the EE. The listing leaves out the
engine budgets that print all zeros - `LoadScratch`, `Skinning`, `BakedStreams`,
`Audio` and `Level` - where the real table prints every budget. Rounding up
means the three totals can add up to a KB or two more than the RAM.) `ext=8x4`
is `extensionCount` x `maxExtensions`; the last line is `fixedBytes`,
`systemBytes` and the newlib reserve of the layout.

## The per-frame scratch

Most of what a frame builds - a list of the actors in range, a string for a HUD
line, a map from id to result - dies before the frame ends. The scratch is a
region those things live in for free: allocating moves a cursor forward, and the
next frame **starts the cursor over from the beginning of the region. Nothing is
erased**; the bytes stay where they were until something is written over them.
Nothing is freed one by one, so there is no fragmentation and no per-object cost.

`Tyra::Scratch` (`inc/memory/scratch.hpp`) has two regions and the containers on
them. Swap `std::vector` for `Tyra::Scratch::vector` and nothing else changes:

```cpp
#include "memory/scratch.hpp"

Tyra::Scratch::vector<Actor*> inRange;        // was std::vector<Actor*>
inRange.reserve(visible);                     // the size is known: say so
Tyra::Scratch::string line = "score ";        // was std::string
Tyra::Scratch::map<int, int> hits;            // was std::map<int, int>
```

| name | the std container |
|---|---|
| `Scratch::vector<T>` | `std::vector<T>` |
| `Scratch::string` | `std::string` |
| `Scratch::deque<T>` | `std::deque<T>` |
| `Scratch::map<K, V, C>` | `std::map<K, V, C>` |
| `Scratch::unordered_map<K, V, H, E>` | `std::unordered_map<K, V, H, E>` |
| `Scratch::stringstream` | `std::stringstream` |
| `Scratch::Load::vector<T>`, `Scratch::Load::string` | the same on the load scratch (below) |

They are the std containers with a different allocator, `Scratch::Allocator<T>`
(`Scratch::LoadAllocator<T>` for the load scratch), so every member and algorithm
works as it does on the heap, and so does the alignment: like `std::allocator`,
a scratch container's buffer is aligned to at least 16 bytes (more for a type
that asks for more), even for a `Scratch::vector<uint8_t>` allocated right after
a block of odd size, so a byte buffer can be cast to a struct as one from the
heap can. (`Region::allocate` itself gives exactly the alignment it is asked
for.) A `Scratch::string` is not the same type as a
`std::string`: build one from the other with `Scratch::string(s.c_str())`, or
append a `std::string` to it with `+=`.

**The two regions.** `Scratch::frame()` is the one to reset at the start of every
frame; it is for anything that does not outlive the frame. `Scratch::load()` is
the one to reset when a level starts to load; it is for the temporary data of
loading (a parsed file header, a table built to place objects) that is thrown
away once the level is up. Each is a `Scratch::Region`, meant to be started on
the `regionBytes` of the `FrameScratch` and `LoadScratch` budgets in the
registry. Nothing starts or resets them yet - the memory manager that will is
not wired - so a region has no room until `init` is called on it (a verifier does
that itself): an allocation from a region nobody started is a full scratch.

| call | what it does |
|---|---|
| `init(base, bytes)` | takes the range, starts the cursor at 0, forgets the peak, new epoch |
| `allocate(size, align)` | moves the cursor to the next address aligned to `align`, takes `size` bytes; the padding counts as used. `nullptr` when the rest of the range is too small - and then nothing has changed |
| `deallocate(p, size)` | moves the cursor back, over the block and its padding, **only if `p` is the block allocated last** (and `size` is its size); any other block is not reclaimed |
| `reset()` | the cursor to 0 and the epoch up by one; the bytes are left as they are |
| `used()` `peak()` `capacity()` `epoch()` | the cursor; the most `used()` has been since `init`; the size of the range; how many times it was started over |

**Reserve when you know the size.** A block you free is only reclaimed when it is
the last one allocated, so a `Scratch::vector` that grows by doubling leaves every
smaller buffer behind it until the reset. `reserve(n)` before filling costs one
block; a vector that grew from 1 to 1024 elements has used about twice what it
holds. The same goes for a `Scratch::string` built by many `+=`. A scratch
container that is created, filled and destroyed before anything else allocates
(the usual shape) gives all of it back when it dies.

**A full scratch** is a failure like any other: `fail` writes
`[MEM] FAIL scratch full name=frame size=<bytes asked> caller=0x<hex>` (`name=load`
for the load scratch) and halts. A container never sees a null from its allocator.
Size the `FrameScratch` budget to the worst frame; `peak()` says how much it
used.

**Keep it to the frame.** A scratch container must not outlive the reset of its
region: its memory is reused by whatever the next frame allocates, so keeping a
`Scratch::vector` in a member and reading it next frame reads someone else's
bytes. With `TYRA_MEMORY_CHECKS` an allocator remembers the epoch it was made in,
and every `allocate` or `deallocate` that happens in another epoch - the first
growth of a kept vector, or just destroying it - reports
`[MEM] FAIL scratch kept past its frame name=frame size=<bytes> caller=0x<hex>`
and halts. A container made in a frame is made for that frame: build it there,
not in a member that lives longer. Without checks the allocator is an empty class
(a `Scratch::vector` is the size of a `std::vector`), all allocators compare equal
and nothing is checked - the same mistake silently corrupts, so a build that
tests game code should have the checks on. With checks, allocators made in the same
epoch compare equal and those of different epochs do not, which is what lets the
standard containers move and swap only what belongs to the same frame.

**One thread.** The regions do no locking: only the game thread - the one that
runs the frame - may use them or reset them. Another thread keeps to the heap.

## Host verifiers

The memory manager is tested on the host, without an emulator. Each
`tools/verify-memory-<name>.cpp` is a small program that exercises one part and
prints `ALL <n> CHECKS PASSED` (exit code 0) or says which expectation failed.
`tools/verify-memory-check.hpp` is what they share:

- `CHECK(cond)` counts, prints the failing expression and line, and keeps going;
  `finish(name)` prints the result and returns the exit code;
- `installThrowingFailure()` makes `fail` testable: the line is captured
  (`capturedFailureLines()`) and the halt throws `ThrownFailure`, which
  `throwsFailure(f, "what text")` catches;
- `allBytes(p, n, value)` checks a filled range.

`tools/verify-memory.py` builds and runs all of them:

```
python tools/verify-memory.py                  # every verifier, both modes
python tools/verify-memory.py --only heap      # tools/verify-memory-heap.cpp
python tools/verify-memory.py --compiler g++   # pick the host compiler
python tools/verify-memory.py --sanitize       # + -fsanitize=address,undefined
```

It picks up every `tools/verify-memory-*.cpp`, compiles each with all of
`vendor/tyra/engine/src/memory/*.cpp` (not the `ps2/` folder) with
`-std=gnu++17 -O1 -Wall -Wextra -Werror`, once with `-DTYRA_MEMORY_CHECKS=0` and
once with `=1`, and runs the programs. The builds live in a temporary folder,
never in the repository. It exits non-zero on any failure and ends with
`PASS memory verifiers (<n> builds)` when everything passed. `--sanitize` adds
AddressSanitizer and UndefinedBehaviorSanitizer (undefined behaviour aborts); run
it where the sanitizers exist - Linux, or WSL on Windows.

Today there are four verifiers:

- `heap` - alignment, coalescing in every release order, exhaustion and exact
  reuse of a hole, the over-aligned cases, `slideDown`, the accounting, the
  checks, and a 100 000-operation seeded stress run (sizes 1 to 65 536, alignments
  16/64/128, every payload filled with its own byte and checked at release,
  blocks compared for overlap, the heap validated as it goes);
- `failure` - the exact line, the order print / handler / halt, a missing
  handler, the texts cut to fit, and a failure raised inside the handler;
- `registry` - the engine rows and the budget and pool calls, the bucket sizes for
  every part of the formula, the order, alignment and non-overlap of the layout,
  the fit check to the byte at its edge and each reason it answers `false` (and
  that it never calls `fail`), sizes too big for a word, and the exact table
  lines;
- `scratch` - the region (alignment of the address, padding, the last-block
  roll-back, exactly full, sizes that would wrap, `reset` erasing nothing, `init`
  starting over), the two instances, every container really living in the region
  and giving the right answers, byte containers 16-aligned after any odd-sized
  block, over-aligned types, 200 frames of the same work
  needing the same room, the `scratch full` line to the byte, and with checks the
  `scratch kept past its frame` failure on `allocate` and `deallocate`, for both
  regions, with allocators of one epoch equal and of two not; without checks an
  allocator that is an empty class.
