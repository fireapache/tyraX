# The memory manager

The memory manager gives a game strict, visible memory budgets: every region of
memory has a name, a size and a counter, and when a request cannot be served the
engine says which budget ran out and who asked for it, instead of the heap
quietly running dry and the console going to a black screen. Not every full
budget stops the game: render data is a request, and a render budget that has
no room refuses - the renderer keeps what it has, draws something cheaper or
nothing this frame, and asks again the next ([When a budget is
full](#when-a-budget-is-full)).

A project opts in at build time with one make variable,
`TYRA_MEMORY_MANAGER := 1` ([Turning it on](#turning-it-on)); without it nothing
in the engine calls the manager and a project builds and runs exactly as before.
This page describes the pieces - the region heap, the failure path, the budget
registry, the per-frame scratch, the pools, `Ptr<T>`, the movable heap with its
defragmentation, the manager that puts them together (routing, budget scopes,
the start of a frame, `Pool<T>`, the stats) - then [the PS2
binding](#on-the-ps2) that wires it into the engine (the switch, the allocation
hooks, the engine's budget scopes, the `[MEM]` lines, the telemetry block) and
the host verifiers that test it. The sources are in
`vendor/tyra/engine/src/memory/` and `vendor/tyra/engine/inc/memory/`;
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
| `freeBlocks()` | how many free blocks the range is in: 1 for an empty heap, 0 for a full one |
| `attachMap(map)` | gives the heap storage for an occupancy map (`RegionHeap::Map`, 324 bytes) to keep, filled from the blocks it holds now; null takes it away |
| `mapCells()` `cellBytes()` | the map's cells - 64, fewer only for a range under 1 KB - and the bytes of each but the last (the range / 64, rounded up to 16; the last has what is left) |
| `cellUsed(c)` | the bytes of used blocks (headers included) in cell `c`, exact |
| `occupancy(out)` | every cell's used fraction: 0 empty, 255 full; any used byte reads at least 1 and only a full cell 255. `copyMap(m)` + `occupancy(m, out)` do the same in two halves, for a caller that holds a lock only for the copy |
| `forEachBlock(f)` | calls `f(BlockInfo)` for every block in address order, free ones included; the heap must not change meanwhile |
| `validate()` | walks every block and every free list and returns whether they agree with the totals, the bitmaps, the free-block count and the map |

**How fragmented it is**, without a walk. `freeBytes()`, `largestFree()` and
`freeBlocks()` say how much is free and in how many pieces: a region with 1.9
MB free whose largest block is 140 KB cannot serve a 150 KB request, however
much is free. The heap keeps the count as it goes (every free block is in one
list), and `largestFree()` reads the top non-empty size class. The **occupancy
map** says where the used blocks are: `kMapCells` (64) cells over the range,
each the bytes of used blocks in it. `allocate`, `release` and `slideDown` add
or take a block's bytes there as they change `used()`: the bytes go into the
cell the block starts in and the cell it ends in, and a block that covers whole
cells in between marks where it starts and stops covering them (a difference
array, summed when the map is read), so a block costs four stores at most
whatever its size, and the cell is found with a multiply by the reciprocal of the
cell size, not a divide. The map's storage is the caller's (`attachMap`): a
manager keeps a `RegionHeap` for every budget id and gives maps only to the
regions it shows.

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
`Tyra::Memory::fail(const Failure&)` (`inc/memory/failure.hpp`); every piece on
this page reports through it. A `Failure` is four fields: `what` (the kind, e.g. `"budget full"`), `name` (the
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

and `onFull`, what a request it has no room for does: `OnFull::Halt` (a failure,
the game stops - for memory whose running out is a bug) or `OnFull::Refuse` (the
request answers null and its caller does without - [When a budget is
full](#when-a-budget-is-full)). `defaultOnFull(kind)` is `Refuse` for `Render`
and `Halt` for every other kind - `Asset` included: a level whose data does not
fit is an authoring error that should stop loudly, and loaders write through
what they are given; `budget()` sets it from the kind whenever it names a row,
and a game may set it otherwise after.

A budget is named by a `BudgetId`. The engine's ids, registered by `Registry()`
with both sizes 0:

| id | name | kind | when full |
|---|---|---|---|
| `Budget::System` (0) | `System` | System | halts |
| `Budget::FrameScratch` (1) | `FrameScratch` | Scratch | halts |
| `Budget::LoadScratch` (2) | `LoadScratch` | Scratch | halts |
| `Budget::Renderer` (3) | `Renderer` | Render | refuses |
| `Budget::Skinning` (4) | `Skinning` | Render | refuses |
| `Budget::BakedStreams` (5) | `BakedStreams` | Render | refuses |
| `Budget::Audio` (6) | `Audio` | Asset | halts |
| `Budget::Level` (7) | `Level` | Asset | halts |

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

`bucketParts(id, count)` is the layout of one bucket of `count` entries - the one
formula for it. Each part is rounded up to 16 so the next stays aligned:

| part | bytes |
|---|---|
| entries | `roundUp(count * stride, 16)`, where `stride = roundUp(max(typeSize, 4), max(align, 4))` |
| live bitmap | `roundUp((count + 7) / 8, 16)` |
| recycle stamps, only for `RecycleOldest` | `roundUp(count * 4, 16)` |
| generations, only with `TYRA_MEMORY_CHECKS` | `roundUp(count * 2, 16)` |

It answers a `BucketParts { entries, bitmap, stamps, generations, total, stride }`:
the byte offset of each part from the bucket's base (`entries` is 0; an absent
part is zero-sized but its offset is filled in), the bucket's size in `total` and
the `stride` in bytes. `bucketBytes(id, count)` is `bucketParts(id, count).total`
and `poolFixedBytes(id)` is `bucketBytes(id, firstCount)`. All of it is 0 for an
id the registry never issued. How a pool uses the layout is under
[Pools](#pools).

### Settings

`registry.settings` holds `ptrTableCapacity` (4096, the entries of the pointer
table - a power of two, or 0; see [Ptr](#ptr)), `defragBytesPerFrame` (64 KB), `newlibReserveBytes` (256 KB, what the C
library keeps for itself after the block), `systemBytes` (0: the system
region takes everything that is left), `refuseHeadroomBytes` (512 KB: what a
refusing budget in measure mode leaves free in System's region - [When a budget
is full](#when-a-budget-is-full)), `refusalReportFrames` (60: the frames
between two lines about one refusing call site, a second at 60 frames a
second) and `refusalLog` (true: whether refusals and fallbacks write their
lines at all - [When a budget is full](#when-a-budget-is-full)).

### The layout and the fit check

`fits(availableBytes, layout)` places everything fixed in **one block** and
checks that the block, plus the newlib reserve, is not more than
`availableBytes`. The pieces follow each other in this order, each starting on a
multiple of 64 bytes (`Layout` holds the offsets from the start of the block):

1. every pool's first bucket, in pool order (`poolOffset[id]`) - a pool whose type
   asks for more than 64 bytes of alignment starts on a multiple of that;
2. the region of every budget that has one, in id order (`regionOffset[id]`);
3. the pointer table (`ptrOffset`, `ptrBytes`: `PtrTable::bytesFor` of the
   capacity - two words an entry, twice over, so a 32-bit pointer halves it);
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
- `ptrTableCapacity` is neither 0 nor a power of two;
- `fixedBytes + newlibReserveBytes` is more than `availableBytes`
  (`registry does not fit: ...`).

`fits` never calls `fail`: it reports and the code that starts the manager
decides what to do. It fills `layout` even when it answers `false`, so the table
can be printed for a registry that does not fit, and `error()` is empty after an
answer of `true`.

### The table

`printTable(print, layout)` calls `print` once per line, without a newline: a
line per budget (by id), a line per pool, then the totals. Sizes are in KB,
rounded up; a budget in measure mode prints `region=0`; `full=` is its `onFull`.

```
[MEM] budget System kind=System region=0 heap=0 full=Halt
[MEM] budget FrameScratch kind=Scratch region=256 heap=0 full=Halt
[MEM] budget Renderer kind=Render region=2048 heap=0 full=Refuse
[MEM] budget Actors kind=Logic region=0 heap=64 full=Halt
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
registry: the manager's `start` does that, its `frameStart` resets the frame
scratch and its `loadStarted` the load scratch (see [The manager](#the-manager)).
Until something calls `init` on a region it has no room (a verifier calls it
itself): an allocation from a region nobody started is a full scratch.

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

## Pools

A pool holds the entries of **one component type** - one `typeSize`, one
alignment - and is a chain of **buckets**: the first bucket is part of the fixed
memory (the registry's layout places it), and when it is full the pool takes
**extension buckets** from a `BucketSource`, up to the cap its registry row gives
it. A **budget** groups pools (it is the cap on how much extension memory they
may take together); a pool belongs to one budget, and two different types never
share a pool. `Tyra::Memory::PoolCore` (`inc/memory/pool.hpp`) is the type-erased
core: it hands out and takes back raw entry memory, and constructs and destroys
nothing.

```cpp
using namespace Tyra::Memory;

Registry r;
r.budget(Budget::FirstGame, "Actors", Kind::Logic).extensionBytes = 64 * 1024;
PoolDesc d{"actor", Budget::FirstGame, sizeof(Actor), alignof(Actor),
           32 /*first*/, 8 /*per extension*/, 4 /*extensions at most*/,
           Policy::Refuse};
PoolId id = r.pool(d);

PoolCore pool;
pool.init(id, d, r, firstBucketMemory, &source);  // source: a BucketSource

void* raw = pool.acquire();        // nullptr when the policy refuses
Actor* a = new (raw) Actor();      // the caller constructs...
a->~Actor();                       // ...and destroys
pool.release(a);
```

A game uses the typed `Pool<T>` on the manager's pools instead, which constructs,
destroys and hands out `Ptr<T>` (see [Typed pools](#typed-pools)).

### What is inside a bucket

A bucket is one block of `bucketBytes(id, count)` bytes, laid out as
`Registry::bucketParts(id, count)` says (the pool reads every offset from there,
so the layout has one definition):

| part | offset | what |
|---|---|---|
| entries | `entries` (0) | `count` entries, `stride` bytes apart |
| live bitmap | `bitmap` | a bit per entry, set while the entry is live |
| stamps | `stamps` | `RecycleOldest` pools only: when each entry was acquired |
| generations | `generations` | `TYRA_MEMORY_CHECKS` only: 16 bits per entry |

`total` is the bucket's size and every offset is a multiple of 16. A part a pool
does not have is zero-sized but its offset is still filled in (`stamps` equals
`generations` in a pool without stamps). Because **everything is inside the
bucket**, copying a bucket's bytes elsewhere copies its bookkeeping too - that
is what lets a bucket source move a bucket (see `bucketMoved`).

**The free list.** The free entries of a bucket form a list through the first
4 bytes of each (an entry index, `0xFFFFFFFF` ends the list), so an entry's first
4 bytes belong to the pool while the entry is free and to the caller while it is
live - destroy the object before `release`. A bucket is formatted when it is
made (the first at `init`, an extension when it is taken), whatever the memory
held. A fresh bucket hands its entries out in address order. A write to an entry
after its release can change its link; what `acquire` can and cannot tell is
under [Damage to the free list](#damage-to-the-free-list).

### The calls

| call | what it does |
|---|---|
| `init(id, desc, registry, firstBucket, source)` | makes this the pool `id` on the memory of its first bucket (`poolFixedBytes(id)` bytes, aligned to 16 or to the type's alignment if more); forgets what the pool held. `source` may be null: the pool never grows |
| `acquire()` | raw memory for one entry, or `nullptr` (below). Takes the entry from the **lowest-numbered bucket that has a free one**; when every bucket is full it takes an extension from the source; when it cannot, the policy decides |
| `release(entry)` | gives the entry back |
| `setRecycle(hook, ctx)` | the function `RecycleOldest` calls (below) |
| `releaseEmptyExtensions()` | every extension with no live entry goes back to the source except one spare, the lowest-numbered of them; the first bucket is never given back |
| `bucketMoved(bucket, newBase)` | the source moved extension `bucket` to `newBase` (a copy of its bytes): the pool finds it there from now on |
| `live()` `capacity()` | entries in use; entries in the buckets held |
| `bucketCount()` | one past the highest bucket number held (below) |
| `entryAt(b, i)` `isLive(b, i)` `countIn(b)` | walk the pool: entry `i` of bucket `b` (null when there is none), whether it is live, and how many entries bucket `b` has (0 when it is not held) |
| `exhaustions()` `name()` | how many times `acquire` found the pool full with nothing to grow into; the row's name |

**Bucket numbers.** The first bucket is 0; an extension takes the lowest number
that is free, from 1 up to `maxExtensions`, so a pool has at most
`1 + maxExtensions <= 64` buckets (`init` fails for more). A bucket keeps its
number for as long as it exists, because ids and a bucket source's tokens
(`bucketToken(pool, bucket)`, never 0) name it. So when `releaseEmptyExtensions`
gives back a bucket from the middle, it leaves a **gap** that keeps its number
and a later extension takes. `bucketCount()` is one past the highest number held,
so `for (b = 0; b < bucketCount(); ++b)` visits every bucket; a gap answers
`countIn(b) == 0`, `entryAt(b, i) == nullptr` and `isLive(b, i) == false`.

**Extensions.** An extension is requested from the source with
`allocateBucket(bytes, align, pinned, owner, budget)`: `bytes` is
`bucketBytes(id, extensionCount)`, `align` is the type's alignment (at least
16), `owner` is `bucketToken(pool, bucket)` and `budget` the pool's. The source
answers null when its budget's cap or its memory is used up. The bucket is
**pinned** - the source must never move it - when the budget's kind is `Render`
or the type is aligned to more than 16. A pool whose `extensionCount` is 0 never
grows, whatever its `maxExtensions` says.

### When a pool is full

`acquire` first uses a free entry of any bucket, then tries an extension, then -
the pool is **exhausted** - counts it (`exhaustions()` goes up, whatever the
policy does about it) and obeys the row's `Policy`:

- `Refuse` - answers `nullptr`; the caller handles it (a grenade is not thrown);
- `RecycleOldest` - reuses the live entry that was **acquired longest ago**
  (anywhere in the pool, not only the first bucket): the recycle hook is called
  as `hook(entry, ctx)` while the entry still holds the old object - destroy it
  there - and the same memory is returned as a new entry. The hook must not call
  `acquire` or `release` on the pool. With no hook the entry is handed back as it
  is. A pool with no live entry at all answers `nullptr`. Age counts
  acquisitions in a 32-bit clock that is compared by distance, so it stays right
  across a wrap;
- `Halt` - `fail({"pool exhausted", name, typeSize, caller})`, which halts in
  every build; the pool is untouched when it does. For a pool whose exhaustion
  is a bug.

### Ids and generations (TYRA_MEMORY_CHECKS)

With `TYRA_MEMORY_CHECKS`, `struct Id { uint16_t pool, bucket, index, generation; }`
names one entry, and a pool answers two more calls:

- `idOf(entry)` - the id of an entry of this pool, carrying the generation the
  entry is at now (it fails for a pointer that is not at the start of one);
- `valid(id, raw)` - whether `id` names a **live** entry of this pool, at that
  generation, and `raw` is that entry's memory.

An entry's **generation goes up every time it is released or recycled**, so the
id of an entry that was released never validates again, even when the same
memory is handed out afresh. A bucket's generations are seeded past everything
the bucket in the same number reached before, so ids of a bucket that was given
back stay dead when the number is used again. An id survives `bucketMoved`: it
names bucket and index, and `raw` is the new address. Generations are 16 bits and
wrap, so an id older than 65536 releases of its own entry is not caught; an
`Id` numbers entries in 16 bits too, so with checks a bucket may have at most
65536 entries (`init` fails for more). Without checks `Id`, `idOf` and `valid`
do not exist.

The checks also fail (through `fail`, with the pool's name) for: `release` of a
pointer that is not an entry of the pool (`pool release outside the pool`), not
at the start of one (`pool release of a misplaced pointer`) or not live (`pool
released twice`); `bucketMoved` of the first bucket, of a bucket that is not
held or to an address that is not aligned (`pool bucket moved wrongly`); a
source that answers a misaligned bucket; a damaged free list (below); and, in
`init`, a `desc` that is not the registry's row for `id`, a misaligned first
bucket or a bucket too big for ids. Without checks `release` and `bucketMoved`
ignore a call they can tell is wrong, and the free-list tests below still run;
nothing else is verified.

### Damage to the free list

The free list is intrusive, so a write to an entry after its `release` (a use
after release) can change the entry's link. Each time `acquire` takes a free
entry it tests, in every build, two things: that the entry it takes is free (its
live bit is clear) and that the link behind it is the end of the list or the
index of an entry of the same bucket that is free too. A link that is out of
range or names a live entry is damage:

- with `TYRA_MEMORY_CHECKS` it is a failure,
  `[MEM] FAIL pool free list corrupt name=<pool> size=<typeSize> caller=0x<hex>`,
  raised before anything is handed out, so the pool is as it was;
- without checks the pool hands out the entry it was taking (it is free, and
  good) and then cuts the list off there: the entries behind it are lost to the
  pool, until the bucket is made again, but the pool never hands out a live entry
  twice and never reads past the bitmap. `acquire` goes on to the next bucket,
  an extension or the policy.

The damage is found by the `acquire` that takes the damaged entry; a link that
names the entry itself looks fine until that entry has been handed out, and is
found by the `acquire` after it. **Not detected**, in either build: a link
overwritten with the index of another free entry. It cannot be told from a good
one; nothing live is handed out, but the entries it skips are lost to the free
list. A write that does not touch the first 4 bytes of a free entry cannot be
seen at all.

`init` fails in every build for a pool the registry never issued, more than 63
extensions, and entries without a first bucket.

One thread only: nothing in a pool locks.

## Ptr

`Tyra::Memory::Ptr<T>` (`inc/memory/ptr.hpp`) is a pointer to a `T` that the
manager can move. Code keeps using pointers - `->`, `*`, `get()` - and when the
movable heap moves a bucket, one **resolve** fixes every `Ptr` that points into
it, so nothing has to ask where the data is now.

```cpp
using namespace Tyra::Memory;

Ptr<Actor> target;                  // empty
target = Ptr<Actor>(rawActor);      // points at an actor
if (target) target->takeDamage(5);  // used like a pointer
Actor* raw = target.get();          // good until the end of the frame
target.reset();
```

### What a Ptr carries

The raw pointer, always. With `TYRA_MEMORY_CHECKS` it also carries the `Id` of
the pool entry it points at (see [Ids and generations](#ids-and-generations-tyra_memory_checks)).
Without the checks the `Ptr` is the raw pointer alone - `sizeof(Ptr<T>) ==
sizeof(T*)`, one word, and the ids and the checks compile out - so the engine and
the game must be built with the same `TYRA_MEMORY_CHECKS`. Nothing else differs.

| call | what it does |
|---|---|
| `Ptr()`, `Ptr(nullptr)` | an empty `Ptr` |
| `explicit Ptr(T* raw)` | points at `raw` (with the checks: asks which pool entry `raw` is, below) |
| copy, move, `=` | as for a pointer; a `Ptr` that was moved from is empty |
| `get()`, `->`, `*` | the raw pointer. With the checks, a `Ptr` that has an id first verifies that its entry is still live |
| `explicit operator bool` | whether it points at anything. Not a liveness check |
| `reset()` | makes it empty |
| `id()` | with the checks: the `Id`, or `kEmptyId` (`pool` is `kNoPool`; `isEmptyId` tests it) |

A `Ptr` must be a real C++ object: constructed, and destroyed before its memory
is reused. The table knows it by the address it lives at, so copying its bytes
with `memcpy` leaves the copy unknown to the table. The one exception is the move
of a whole bucket by the heap, which `resolve` accounts for. A `Ptr` is for the
game thread only; nothing here locks.

### What is registered

Only a `Ptr` whose target lies in the **movable heap** is registered; a `Ptr` to
fixed, routed or static memory never needs fixing and costs one range test (two
comparisons with a global) when it is made, copied, moved, assigned or destroyed.
A registered `Ptr` is a row `{address the Ptr lives at, token of the bucket it
points into}` in a `PtrTable`, which lives outside the `Ptr`. The token is the
bucket's owner token (`bucketToken`); a place in the movable heap that is in no
bucket has token 0.

- constructing or copying a `Ptr` adds its row, and so does assigning one;
  assigning over a `Ptr` that is registered replaces its token, which needs no
  new row;
- a **move** re-keys the row from the old `Ptr` to the new one. It needs no room,
  so a move cannot fail for a full table;
- `reset()`, destruction, and assigning a target outside the movable heap remove
  the row;
- when the table is full, a `Ptr` that would need a new row fails with
  `[MEM] FAIL ptr table full name=ptr table size=0 caller=0x<hex>` in every
  build, before anything about the `Ptr` has changed. Size
  `Settings::ptrTableCapacity` for the most `Ptr`s into the movable heap that
  exist at once.

### The table

`PtrTable` holds `capacity` rows (`Settings::ptrTableCapacity`, a power of two;
`fits` reports any other, and 0 is a table that holds nothing) in two equal
arrays of `{uintptr_t address; uint32_t token;}`, so it needs
`PtrTable::bytesFor(capacity)` bytes - two words a row, twice over, which is what
the registry's layout reserves (`Layout::ptrBytes`). It is an open-addressing hash
set keyed by the address of the `Ptr`, with linear probing and a backward shift
on erase (no tombstones, so a table that has seen many erasures is no slower for
it). A lookup probes further the fuller the table is, so leave headroom: it holds
`capacity` rows, but is fastest well below that.

| call | what it does |
|---|---|
| `init(storage, capacity)` | makes the table in `bytesFor(capacity)` bytes aligned for a pointer. Fails (every build) for a capacity that is not a power of two or storage that is missing or misaligned |
| `insert(ptrAddress, token)` | adds a row, or replaces the token of the row it has. False - and nothing changes - when the table is full or the address is null |
| `erase(ptrAddress)` | removes the row; an address that is not registered is ignored |
| `rekey(from, to)` | the `Ptr` registered at `from` now lives at `to`; same token. Cannot fail for a full table; a `from` that is not registered is ignored |
| `resolve(moves, count)` | fixes the table and the `Ptr`s after buckets moved (below) |
| `dropBucket(token)` | removes every row with that token (0 is ignored) |
| `size()` `capacity()` | the rows in use; the most it holds |
| `forEach(f)` | calls `f(void* ptrAddress, uint32_t token)` for every row, in no particular order |

`PtrRegistry` (`init(storage, capacity, lo, hi, tokenAt, ctx)`) is the table plus
what registering needs: the range `[lo, hi)` of the movable heap and `tokenAt`,
the function that answers the token of the bucket that holds an address (0 when
none). It does not install itself. A `Ptr` finds the registry through
`detail::g_ptrs` and the range through `detail::g_movableLo` and
`g_movableHi` (an empty range by default: with none set, no `Ptr` is registered
and every `Ptr` costs only the range test); whoever owns the registry sets all
three.

### Resolve

Buckets of the movable heap move (the heap copies their bytes, and the old and
new places may overlap), and then one call fixes everything:

```cpp
const Move moves[] = {{oldBase, bytes, newBase, token}, /* ... */};
table.resolve(moves, count);
```

A `Move` is `{uintptr_t oldBase; size_t bytes; uintptr_t newBase; uint32_t
token;}` - the bucket's old place, its size, its new place and its owner token.
The moves of one call are of distinct buckets and their old ranges do not
overlap. For every row `(A, K)`, in this order:

1. if `A` lies in some move's `[oldBase, oldBase + bytes)`, the `Ptr` itself was
   in a bucket that moved: `A` becomes `A + newBase - oldBase`;
2. if `K` is some move's token, the `Ptr` points into a bucket that moved: the
   raw pointer the `Ptr` holds - read at the new `A`, where its bytes are now -
   gets `newBase - oldBase` added. A pointer into the middle of an entry moves
   with it.

The rows go into the idle array and the arrays swap, so a row whose key just
changed is never looked at twice - two buckets may swap places in one call. A
`Ptr` that lives in a bucket and points into it is fixed by both rules; one that
points into a bucket that did not move, or lives outside the heap, is left
alone. Every registered `Ptr` must be inside a live object, because resolve reads
and writes its first word. After `resolve` the table has the same rows at their
new addresses and `size()` is unchanged.

**A bucket that is given back** takes `dropBucket(token)` with it. The `Ptr`s that
still pointed into it dangle; if their rows stayed, a later bucket that gets the
same token would patch them. A dropped `Ptr` is harmless: destroying it finds no
row and does nothing.

**A raw pointer is good until the frame ends.** `get()`, `->` and `*` return a
raw pointer, and `resolve` does not know about it. Buckets are meant to move
only at the start of a frame, never in the middle of one, so a raw pointer taken
during a frame is valid until the frame ends. Do not keep one across frames; keep the `Ptr`.

### Pin

`Pin<T>` keeps the bucket that holds a `Ptr`'s target where it is while the `Pin`
lives:

```cpp
{
  Pin<Actor> hold(target);  // the bucket will not move
  startSomethingThatReadsTheActorLater(target.get());
}                           // let go
```

A `Pin` of an empty `Ptr`, or of one that points outside the movable heap, does
nothing. A `Pin` can be neither copied nor moved. The pinning itself is done by a hook, `detail::PinHook {pin, unpin, ctx}` in `detail::g_pinHook`,
that the code which moves buckets installs: each `Pin` calls it once when it is made
and once when it is destroyed. With no hook, `Pin` does nothing.

### Ids (TYRA_MEMORY_CHECKS)

With the checks, a `Ptr` made from a raw pointer asks `detail::idOf(raw)` which
pool entry it is and keeps the answer; copies carry it, a move moves it and
`reset()` clears it. `get()`, `->` and `*` then ask `detail::checkLive(raw, id)`
and fail with `[MEM] FAIL ptr to a dead object name=ptr size=0 caller=0x<hex>`
when the entry has been released or recycled. A `Ptr` with no id - an empty one,
or one made from memory that is not a pool entry - is not checked.

Both calls go to `detail::IdHook {checkLive, idOf, ctx}` in `detail::g_idHook`,
which the owner of the pools installs (the manager, at `start`). With no hook `checkLive` answers true and
`idOf` answers `kEmptyId`, so a `Ptr` costs no more than a pointer and a range
test. `idOf` should answer `kEmptyId` for memory that is not the start of a pool
entry.

## The movable heap and defragmentation

`Tyra::Memory::MovableHeap` (`inc/memory/movable_heap.hpp`) is the heap the
extension buckets of the pools live in - it is their `BucketSource` - and the
one place the manager moves memory. At the start of a frame it slides buckets
down into the holes in front of them, so the free space gathers into one block
instead of lying in pieces between buckets, and then one `PtrTable::resolve`
makes every registered `Ptr` right again. Nothing else in the engine moves
memory, and nothing moves it in the middle of a frame.

```cpp
MovableHeap heap;
heap.init(base, bytes, extensionCaps, &ptrRegistry);  // caps by BudgetId
heap.installPinHook();                                // Pin<T> pins through it
heap.setMoveListener(&tellThePool, ctx);              // the pool's bucketMoved
// ptrRegistry.init(storage, n, lo, hi, &MovableHeap::tokenAtCallback, &heap);

size_t moved = heap.step(64 * 1024);                  // at the start of a frame
```

`step` is meant to run once, first thing in a frame, with
`Settings::defragBytesPerFrame` (64 KB by default) as its budget; the manager's
`frameStart` does that, and its `start` wires the pools, the `Ptr` registry and
this heap together as above (see [The manager](#the-manager)).

### Buckets

The heap is a `RegionHeap` that only hands out whole buckets, and it keeps a
list of them in address order. A bucket's **token** - the `owner` the pool passes
to `allocateBucket`, `bucketToken(pool, bucket)` - sits in the block header, with
the budget, so `tokenAt(p)` (a binary search over the list) names the bucket
that holds any address, and 0 for free space, a header, an address outside the
heap or null. The list is a fixed table of `MovableHeap::kMaxHeapBuckets` (512)
rows: the heap allocates nothing of its own, and with that many buckets alive
`allocateBucket` answers null like any other refusal, so the pool's policy
(`Refuse`, `RecycleOldest` or `Halt`) decides what happens.
(`Tyra::Memory::kMaxBuckets` is another number: the buckets of one pool.)

`allocateBucket(bytes, align, pinned, owner, budget)` answers null - and changes
nothing - when the budget has no room for it, the heap has no block that fits or
the list of buckets is full. `init` takes the caps by `BudgetId` (the registry's `extensionBytes`; a cap of 0,
or no array, means the budget may take no bucket). A cap is tested against the
block the request needs: a 16-byte header and the payload rounded up to 16, at
least 32 bytes in all. `usedBy(budget)` counts the block bytes a budget really
holds; a block that came out up to 16 bytes bigger (the heap does not split a
remainder that small) is charged in full, and so are the bytes an over-aligned
block's gap leaves with the block in front of it.

`releaseBucket(base)` gives a bucket back by the address it has now, forgets its
pins and calls `PtrTable::dropBucket` for its token, so the `Ptr`s that still
pointed into it are no longer patched when another bucket gets the same token.
`freeBytes()` and `largestFree()` are the heap's: `largestFree()` is the largest
allocation that would succeed, which is what defragmentation raises.
`region()` is the `RegionHeap` itself - its free-block count and its occupancy
map, which the movable heap always keeps. `largestRequest()` is the biggest
bucket asked for since `resetLargestRequest()`, served or not.
`validate()` checks the heap, the list against the heap's blocks, the budgets'
counts against a walk and the pins.

### The step

`step(budgetBytes)` walks the buckets in address order. A bucket moves when it
is not pinned, is not aligned above 16 and has a free block directly in front of
it. It is slid into that block (`RegionHeap::slideDown`); the free block it
leaves behind merges with the free block behind it, so the bucket after it has a
hole in front of it now and slides too, in the same step. For every move the
move listener is called - `onMove(token, newBase, ctx)`, which is the pool's
`bucketMoved` - after the bytes are in their new place; when the walk is done,
one `PtrTable::resolve` takes the batch as a list of `Move`s (the payload that
moved, its old and new address and the token). The Ptrs are right again when
`step` returns. It returns the block bytes it moved (also `movedLastStep()`);
`movesLastStep()` is the buckets it moved and `oversizeLastStep()` how many of
them were bigger than its budget (0 or 1: such a bucket moves alone).

- **The budget counts block bytes**, header and payload, of the buckets moved.
  The walk stops at the first bucket that would take the total over
  `budgetBytes`. The first bucket a step moves is let through whatever its size,
  so a bucket bigger than the whole budget moves alone, in a step of its own,
  and cannot wait for ever. A budget of 0 moves nothing.
- **A step moves at most `kMaxMovesPerStep` (32) buckets**, because resolve looks
  at every move for every registered `Ptr`; it ends there as if the budget were
  spent.
- **A bucket that cannot move does not end the walk**: the buckets behind it that
  have a hole in front of them still move.
- The listener runs while the Ptrs are not resolved yet: it must not touch a
  `Ptr` or call the heap.

### What never moves

- A bucket **allocated pinned**. A pool asks for it when its budget is of the
  `Render` kind and when its type is aligned to more than 16 bytes (see
  [Pools](#pools)), so memory the renderer may be reading, or that needs a
  bigger alignment, stays where it is.
- A bucket **aligned above 16 bytes**: a slide could not keep the alignment, so
  `RegionHeap::slidable` is false.
- A bucket under a **`Pin<T>`**: after `installPinHook()` each `Pin` calls
  `pin(address)` and, when it ends, `unpin(address)`. The heap counts the pins
  per bucket in a table keyed by token (as many rows as there are buckets, so it
  cannot fill up), and pins nest. A pin on an address in no bucket does nothing.
- Everything that is not in the movable heap: the pools' first buckets (fixed
  memory), the scratch regions and the routed allocations are never moved at all.

A raw pointer taken out of a `Ptr` is good until the frame ends, because that is
when the heap moves things (see [Ptr](#ptr)).

### What is left to do

`health(budgetBytes)` answers, from one walk of the bucket list (at most
`kMaxHeapBuckets` rows), how far the heap is from compact, with the step's
budget (`Settings::defragBytesPerFrame`). A bucket is **fixed** when it cannot
move now: allocated pinned, aligned above 16, or under a `Pin`.

| field | what it is |
|---|---|
| `debt` | the free bytes below the end of the highest bucket: what defragmentation has not closed yet |
| `blocked` | of the debt, the free bytes below the end of the highest fixed bucket. A bucket slides only into the hole right in front of it, never past a fixed one, so steps can gather these bytes in front of the fixed buckets but never close them; `debt - blocked` is what steps can still close |
| `pinned` | the block bytes of the fixed buckets |
| `toMove`, `moveBytes` | the movable buckets that still have to move - those with free bytes between them and the nearest fixed bucket (or the heap's start) below them - and their block bytes |
| `oversize`, `oversizeBytes` | of those, the ones bigger than the budget, and their bytes: a step lets such a bucket through only as its first move. A bucket bigger than the budget that has nothing to close below it is not counted: it is no work |
| `framesToCompact` | the steps compaction takes from here, if nothing else changes: 0 with nothing to move, `MovableHeap::kNever` (0xFFFFFFFF) with something to move and a budget of 0 |
| `buckets` | `bucketCount()` |

The work left is measured the way a step spends its budget: in the bytes of the
buckets it **moves**, not the bytes of hole it closes. A hole stops being debt
only when every movable bucket above it, up to the next fixed one, has slid down
- so 16 bytes of hole under 1 MB of buckets is 1 MB of moves, 16 frames at
64 KB. Each of those buckets moves exactly once and in address order (a step
walks in address order and ends at the first bucket that does not fit), so
`health` packs them into steps exactly as `step` does - its budget, a first move
let through whatever its size, `kMaxMovesPerStep` - and counts the steps.

Only this heap is defragmented. The routed regions - System's and the budgets'
own - never move a block, so their fragmentation is not something defrag can
pay off: it is answered by giving the budgets that fragment them regions of
their own, and the occupancy maps show where it is.

### Failures

With `TYRA_MEMORY_CHECKS`, `allocateBucket` with a
budget past `Budget::Max`, a token of 0 or a token a live bucket already has, and
`releaseBucket` of an address that is not the start of a bucket go to `fail`
(and allocate answers null); without the checks the same calls answer null or are
ignored.

## The manager

`Tyra::Memory::Manager` (`inc/memory/manager.hpp`) is the piece that owns the
others. At start it takes **one block** from the C library, lays the registry
out in it - the pools' first buckets, the budgets' regions, the `Ptr` table, the
movable heap and the system region - and from then on every allocation the
platform routes to it (`malloc`, `new`, ...) is served from a region and charged
to a budget. `inc/memory/memory.hpp` is the header a game includes: the manager,
the registry, `Pool<T>`, `Ptr<T>`, the scratch and `fail`.

### The platform

The manager calls the machine through a `Platform`, a struct of functions it is
given with `setPlatform` before anything else - before the first allocation that
reaches it:

| function | what it is |
|---|---|
| `rawAlloc(align, size)`, `rawFree(p)`, `rawUsable(p)` | the C library's allocator under the routing: `memalign`, `free`, the usable size of one of its blocks |
| `available()` | the bytes the C library can still hand out - what the fit check gets |
| `threadId()` | the running thread |
| `lock()`, `unlock(state)` | excludes every other thread (interrupts off, on the EE) and restores what was; the manager never takes it twice |
| `print(line)`, `halt()` | one line without a newline, not allocating; a halt that does not return. `setPlatform` also makes them `fail`'s output (`setFailureOutput`) |

None of them may allocate through the manager. The manager itself allocates
nothing through the system allocator either: everything it keeps is in the
`Manager` object or in the block, and its lines are formatted into stack
buffers. `Manager::instance()` is the one the routing uses; it is built on first
use (that may be from inside a `malloc`) and never destroyed, so the static
destructors that free memory at exit still find it. A verifier makes `Manager`s
of its own.

### Start

```cpp
using namespace Tyra::Memory;

Manager& m = Manager::instance();
m.setPlatform(platform);
Registry r;
r.budget(Budget::FirstGame, "Actors", Kind::Logic).regionBytes = 512 * 1024;
r.budget(Budget::FrameScratch).regionBytes = 256 * 1024;
m.start(r);  // the registry is copied: r may go away
```

`start(r)` lays `r` out with `fits(available(), layout)` and takes the block
with `rawAlloc(layout.blockAlign, layout.blockBytes)`. In the block:

- **System's region** - a `RegionHeap` of `layout.systemBytes` - and a region
  heap for every other budget with `regionBytes` (each size rounded down to a
  multiple of 16);
- the `FrameScratch` and `LoadScratch` regions become `Scratch::frame()` and
  `Scratch::load()` (a budget of 0 bytes leaves its scratch with no room);
- the movable heap, with every budget's `extensionBytes` as its cap; its move
  listener is the owning pool's `bucketMoved` (the bucket's token,
  `bucketToken(pool, bucket)`, says which), and it becomes the `Pin` hook;
- the `Ptr` registry over exactly the movable heap's range, and the globals
  every `Ptr` reads (`detail::g_ptrs`, `g_movableLo`, `g_movableHi`) set from
  that registry's own range in one place;
- every pool, on its first bucket, with the movable heap as its bucket source;
  with `TYRA_MEMORY_CHECKS`, the id hook that answers which pool entry a
  pointer is and whether an id is still live.

The thread that calls `start` is the **game thread**. `start` answers `true`;
everything that could be refused is refused before the block is taken, with the
registry's table printed first and then a failure:

| `[MEM] FAIL ...` | when |
|---|---|
| `registry does not fit` | `fits` answers false (its `error()` is printed as a line of its own), or the C library refuses the block (`name=the C library refused the block`) |
| `system budget has a regionBytes: size it with settings.systemBytes` | `Budget::System` has `regionBytes`: System's region is sized by `settings.systemBytes` alone (0: everything left) |
| `budget region too small` | a region, System's included, is below `RegionHeap`'s smallest block (32 bytes) after rounding |
| `pool has too many extensions` | a pool with extensions has `maxExtensions` 64 or more |
| `pools take more extensions than the movable heap has buckets` | the pools' `maxExtensions` add up to more than `MovableHeap::kMaxHeapBuckets` (512) |
| `manager started twice`, `manager has no platform (setPlatform)` | as it says |

### Routing

Every allocation goes to one place, by these rules:

1. **Before `start`** it goes to the C library (`rawAlloc`), as without a
   manager. `Totals::newlibBytes` counts those blocks until they come back.
2. **`release` of an address outside the block** goes to `rawFree`: a block
   from before `start`, or one the C library made for itself. The C library's
   own allocations (stdio's buffers, `strdup`, ...) never reach the manager; they
   come out of what it keeps after the block - `settings.newlibReserveBytes`.
3. **Another thread** is charged to `System`, whatever scope the game thread has
   open.
4. **The game thread with no scope open** is charged to `System`.
5. **Otherwise** the innermost scope's budget: served from its own region when it
   has one, and from System's region - **charged to itself**, with no cap - when
   its `regionBytes` is 0 (measure mode). A budget's use is counted either way.
6. **No room** in the region is what the budget's `onFull` says: a failure,
   `[MEM] FAIL budget full name=<budget> size=<bytes> caller=0x<hex>`, for a
   budget that halts; a null for one that refuses ([When a budget is
   full](#when-a-budget-is-full)). Never a fallback to another region.

A block's budget is written in its header, so `release` and `reallocate` read it
there - a block in System's region charged to a measured budget is that budget's,
wherever it is freed from. **`reallocate` stays in the block's budget** whoever
calls it and whatever scope is open: growing a block of a full budget fails with
`budget full` (or, in a refusing budget, answers null) even when System has
room, and the block is left as it was. A
shrink keeps the block; a size of 0 frees it and answers null (as newlib does);
null is an `allocate`. A block from before `start` is copied into the budget an
`allocate` would use now (its size from `rawUsable`) and given back to the C
library. A grown block is aligned to 16, as the C library's `realloc` gives.

An alignment below 16 is 16; one that is not a power of two answers null (no
failure: `aligned_alloc` and `posix_memalign` report it themselves). With
`TYRA_MEMORY_CHECKS`, freeing memory of the block that is not a live routed
block - a pool entry, a scratch block, a block freed twice - is a failure
(`free of manager memory that is not a routed block`,
`free of a block that is not live`). Without them, a free of memory that is
not in a routed region (a pool entry, a scratch block) is ignored, but a block
of a routed region freed twice is not caught: the second free goes to the
region as if the block were live, which corrupts the region's free lists and
its budget's count - undefined behaviour, as a double `free` is in newlib.

**What a budget is charged** is what the region's `used()` grew by: the block,
header and payload, and for an over-aligned block whose 16-byte gap joined the
block in front of it, those 16 bytes too. The header keeps that amount, and a
release takes back exactly it, so a budget's count never drifts whichever blocks
share a region. Every region operation runs under the platform's lock; the
scopes, the scratch, the pools and the `Ptr`s are the game thread's.

### Budget scopes

```cpp
{
  MemScope scope(Budget::Level);  // every allocation of the game thread
  loadTheLevel();                 // in here is charged to Level
}
```

`MemScope` opens a scope for its lifetime (`Manager::pushScope` / `popScope`,
on `Manager::instance()`); scopes nest and the innermost wins. Opening one on a
budget that is not registered (`scope on an unknown budget`), on `FrameScratch`
or `LoadScratch` (`scope on a scratch budget`) or deeper than
`Manager::kMaxScopeDepth` (16, `budget scope too deep`), and closing one that is
not open (`budget scope underflow`), are failures. On any thread but the game
thread a scope does nothing: those threads go to `System`. A scope on `System`
is allowed - it marks a call site as deliberately the system's. Before `start`
the game's ids are not known yet: a scope opened then is checked by `start`.

**Unscoped callers.** With `TYRA_MEMORY_CHECKS` the manager remembers every
call site (the `caller` the routing passes, the return address of the code that
allocated) that allocates on the game thread with no scope open after `start`,
and prints it the first time: `[MEM] unscoped caller=0x<hex>`. The set holds
`Manager::kMaxUnscopedSites` (256) sites, kept in the manager, not allocated; when
it is full, `[MEM] unscoped callers: the set is full (256)` says so once and later
sites are not reported. `Totals::unscopedSites` counts them.

### When a budget is full

A budget that **halts** (`OnFull::Halt`: `System`, `Logic`, `Scratch` and
`Asset` - memory whose running out is a bug; for a level, data that does not fit
is an authoring error) turns a request it has no room for into a failure, which
stops the game with the `[MEM] FAIL` line. A budget that **refuses**
(`OnFull::Refuse`: `Render` unless a game says otherwise) answers the request
null instead. Render data is a request, not a need: its caller keeps
what it already has, draws a cheaper level of detail, or skips that object this
frame - and asks again the next, when memory may be less fragmented or freed.
Each refusal:

- is counted in its budget (`BudgetStats::refused`, the table's `refused=`,
  the telemetry rows);
- prints `[MEM] refused name=<budget> size=<bytes> caller=0x<hex> frame=<f>`
  through the platform's `print`, once per call site per
  `settings.refusalReportFrames` frames at most (the manager remembers
  `Manager::kMaxReportKeys` (64) sites; past that a newcomer takes a remembered
  one's place, so it still gets its line). `frame` is `Manager::frame()`, the
  frames started so far.

**Quiet refusals.** With `settings.refusalLog` false, neither a refusal nor a
fallback (below) writes a line - no `[MEM] refused`, no `[MEM] fallback`, no
`suppressed` count - while the counts go on as before: `refused=` in the table,
`BudgetStats::refused`, the telemetry rows. A game whose render budgets refuse
by design (a crowd that does not fit, drawn cheaper) can turn it off in a
shipping build, where every line is a write to the console - on the console an
IOP round trip - about twenty a second in a busy scene. Failures still print
their line and the table: nothing silences a `[MEM] FAIL`.

Which calls can be refused is decided by the call, not only by the budget:

| call | in a refusing budget with no room |
|---|---|
| `allocate` (`malloc`, `calloc`, `memalign`, `aligned_alloc`, `posix_memalign`, the nothrow `operator new` / `new[]`) | null |
| `reallocate` (`realloc`) | null; the block is left as it was |
| `allocateOrFail` (the throwing `operator new` / `new[]`) | `[MEM] FAIL throwing new in a refusing budget name=<budget> ...` - the call site asks for render memory it cannot do without: a bug to convert to `new (std::nothrow)` and a fallback |

**The headroom in measure mode.** A refusing budget with no region of its own
is served from System's region, and System's other users halt when it runs out.
So such a budget also refuses a request that would leave System's region with
less than `settings.refuseHeadroomBytes` (512 KB) free: render data can never
starve the game's logic. A refusing budget with a region of its own is held to
its region only.

**The fallback's own line.** A render site that did without says what it
dropped with `noteFallback(what, asset, lod, key)`:
`[MEM] fallback <what> asset=<asset> lod=<lod> frame=<f>`, once per `key` (any
address that stays the same for the asset) per `refusalReportFrames` frames,
through the same `print` - nothing allocated, so it is safe at the moment memory
is short. Every key together gets at most `Manager::kMaxFallbackLinesPerWindow`
(16) lines a `refusalReportFrames` window: a shortage that hits hundreds of
assets at once - more than the 64 keys the manager remembers, so each newcomer
would get its line every frame - drops the rest, and `frameStart` prints how
many when the window ends: `[MEM] fallback lines suppressed=<n> frames=<a>-<b>`.

### The start of a frame

`frameStart()` runs first thing in a frame on the game thread, and in this order:

1. counts the frame (`frame()`) and resets the frame scratch
   (`Scratch::frame().reset()`);
2. gives every pool's empty extensions back (`releaseEmptyExtensions`, one spare
   kept per pool);
3. runs one defragmentation step, `step(settings.defragBytesPerFrame)` - the
   pools follow their moved buckets and the `Ptr`s are resolved before it
   returns - and writes what it did into the defrag ring (three stores: see
   [Fragmentation](#fragmentation));
4. rolls the per-frame counters: `Totals::allocsLastFrame` (the `allocate` and
   `reallocate` calls since the last frame start, from every thread) and
   `movedLastFrame`;
5. prints a line for every pool that was exhausted since the last frame start:
   `[MEM] pool <name> exhausted +<n> policy=<policy>` (every policy, `Refuse`
   and `RecycleOldest` included);
6. with `TYRA_MEMORY_CHECKS`, sweeps the `Ptr` table: every registered `Ptr`
   that carries an id must still name a live entry. The first one that does not
   is printed as `[MEM] dead ptr at 0x<where the Ptr lives> target=0x<raw>` and
   fails with `ptr to a dead object` (`name=ptr sweep`).

`loadStarted()` resets the load scratch; call it when a level begins to load.

### Typed pools

`Pool<T>` (`inc/memory/pool.hpp`) is the typed face of a pool the manager
started: it constructs and destroys the objects and hands them out as `Ptr<T>`.

```cpp
Pool<Bullet> bullets(bulletPool);  // the PoolId the registry gave
Ptr<Bullet> b = bullets.acquire(origin, velocity);
if (!b) return;                    // Refuse: the pool is full
b->step();
bullets.release(b);                // destroyed; b is empty now
bullets.forEach([](Bullet& x) { x.step(); });
```

| call | what it does |
|---|---|
| `Pool<T>(id)` | binds to the manager's pool `id`. Fails when the manager has not started, when the id is not a pool (`pool is not in the registry`) and when the row is not for `T` - its `typeSize` is not `sizeof(T)`, or `alignof(T)` is above 16 and above the row's `align` (`pool type does not match its row`) |
| `acquire(args...)` | constructs a `T` from `args` in a free entry and returns a `Ptr<T>` to it; an empty `Ptr` when the policy refuses |
| `release(p)` | destroys the object, gives the entry back and empties `p`; an empty `p` does nothing |
| `forEach(f)` | `f(T&)` for every live object; `f` must not acquire or release on the pool |
| `live()` | the objects alive |

A `Pool<T>` is a handle: copies share the manager's pool. It installs the
recycle hook, so a `RecycleOldest` pool destroys the oldest object before it
constructs the new one in its entry. With `TYRA_MEMORY_CHECKS` the `Ptr<T>` that
`acquire` returns carries the entry's id, and a copy of it that outlives a
`release` or a recycle fails on its next use. An object in an extension bucket
may move between frames - the `Ptr`s to it are fixed, a raw pointer is not.
`Manager::pool(id)` gives the type-erased `PoolCore`.

### Stats and the table

`budgetCount()` and `budgetStats(i)` walk the registered budgets by id; a
`BudgetStats` is `{name, id, kind, cap, used, peak, exhaustions, refused,
onFull}`: `cap` is the region's bytes (0 for a budget in measure mode, which has
no cap), `used` and `peak` what its blocks were charged now and at most (a
scratch budget reports its region's cursor and peak), `exhaustions` the sum over
its pools, `refused` the requests it answered null and `onFull` what it does when
full.
`totals()` gives `{fixedBytes, systemFree, heapFree, heapLargest, movedLastFrame,
newlibBytes, allocsLastFrame, ptrs, unscopedSites}`. `owns(p)` says whether `p`
lies in the block and `regionOf(p)` which budget's region holds it (-1 outside a
region: the pools' first buckets, the `Ptr` table and the movable heap are not
regions).

`printTable()` prints the registry's table (see [The table](#the-table)), then a
line per budget, then the C library and the movable heap, through the platform's
`print`:

```
[MEM] budget System cap=29560 used=1488 peak=1630 exhausted=0 refused=0
[MEM] budget FrameScratch cap=256 used=0 peak=97 exhausted=0 refused=0
[MEM] budget Renderer cap=0 used=1830 peak=1902 exhausted=0 refused=14
[MEM] budget Actors cap=512 used=73 peak=140 exhausted=2 refused=0
[MEM] newlib boot=41 free=212 reserve=256
[MEM] heap cap=64 free=60 largest=60 moved=0 ptrs=12
```

Sizes are KB, rounded up. `cap=0` is a budget with no region - served from
System's region, counted and uncapped - and System's own `used` is only what is
charged to System. `newlib boot` is the blocks the C library served before
`start` that have not come back, `free` what it can still hand out (the reserve
left), `reserve` the registry's `newlibReserveBytes`. The heap line is its size,
free bytes, largest free block, the bytes the last frame's step moved and the
registered `Ptr`s.

### Fragmentation

For a tool that shows how memory is doing while the game runs (the telemetry
block below), the manager keeps the answers as it goes: nothing here walks a
routed region, and only `heapHealth()` walks anything (the movable heap's
buckets).

| call | what it gives |
|---|---|
| `defragRing()` | the last `kDefragRingFrames` (64) frame starts' steps: frame `f` (`f > frame() - 64`) is entry `f % 64`, a `DefragFrame {bytes, buckets, oversize}` - the block bytes the step moved, the buckets, and how many of them were bigger than the budget (moved alone). `frameStart` writes it: three stores |
| `regionCount()`, `regionStats(i)` | the regions a tool shows, in this order: System's, each budget's own region by id (not the scratch regions, which are not heaps), then the movable heap when it has bytes - at most `kMaxRegionRows` (16); a budget past that is left out and its region keeps no map. A `RegionStats` is `{id, capacity, freeBytes, largestFree, freeBlocks, largestRequest, cells, map[64]}`: the region's budget (`kMovableHeapRegion`, 0xFFFF, for the heap), its totals, the pieces its free bytes are in, the largest request it got since the window began, and its occupancy map (`RegionHeap::occupancy`). The numbers of a routed region and a copy of its map are taken under the lock; the map is read after it |
| `largestRequest()` | the largest `allocate` / `reallocate` request since the window began, any region, served or refused: the request most likely to fail next. Each region keeps its own (a request is the region's that serves its budget, so a measured budget's go to System's row) |
| `resetLargestRequests()` | starts a new window, every region's and the movable heap's (its biggest bucket asked for) |
| `heapHealth()` | the movable heap's `health(settings.defragBytesPerFrame)`: debt, blocked, fixed bytes, the buckets still to move, oversize ones, the frames to compact ([What is left to do](#what-is-left-to-do)) |

What it costs: one compare per `allocate` (the largest request, under the lock it
takes anyway); a block's bytes in its two cells and at most two marks per
`allocate`, `release` and slide in a region that keeps a map; a free-block count;
three stores per frame. The maps take 324 bytes a region shown (16 in the
manager, one in the movable heap).

## On the PS2

Everything above is portable and tested on the host. This part is the EE's: the
build switch, the allocation hooks and the `Platform` (`src/memory/ps2/hooks.cpp`),
the engine's budget scopes, the `[MEM]` lines and the telemetry block
(`src/memory/ps2/telemetry.cpp`).

### Turning it on

`vendor/tyra/Makefile.base` reads two make variables:

| variable | what it does |
|---|---|
| `TYRA_MEMORY_MANAGER := 1` | compiles the binding and the engine's calls in (`-DTYRA_MEMORY_MANAGER=1`) and links with the linker's `--wrap` for `malloc`, `free`, `calloc`, `realloc`, `memalign`, `aligned_alloc`, `posix_memalign`, `operator new` / `new[]` (`_Znwj`, `_Znaj`), their nothrow forms (`_ZnwjRKSt9nothrow_t`, `_ZnajRKSt9nothrow_t`) and the aligned forms of all four (`_ZnwjSt11align_val_t`, `_ZnajSt11align_val_t`, `_ZnwjSt11align_val_tRKSt9nothrow_t`, `_ZnajSt11align_val_tRKSt9nothrow_t`): every call of those in the link - the engine, the game, libstdc++, ps2sdk - goes to the manager |
| `TYRA_MEMORY_CHECKS := 1` | `-DTYRA_MEMORY_CHECKS=1`: the ids, generations and integrity checks this page describes. Off unless asked for: a development build wants them, a shipping build does not |

A project sets them in its `project.mk`, the make fragment `Makefile.base` reads
before them ([native-toolchain.md](native-toolchain.md#a-projects-own-make-fragment-projectmk)):

```make
TYRA_MEMORY_MANAGER := 1
TYRA_MEMORY_CHECKS  := 1
```

Any other value, or none, is off. Without `TYRA_MEMORY_MANAGER` nothing in the
engine calls the manager, `src/memory/ps2/` compiles to nothing and the link has
no `--wrap`: the build is exactly the one without a manager.

**The engine library is built with the same values.** Both compile into
`libtyra.a` as well as into the game - the engine's start, frame start and scopes
are `#if TYRA_MEMORY_MANAGER`, and `TYRA_MEMORY_CHECKS` changes the layout of a
`Ptr` and of a pool's buckets - but the library is made by its own make, in its
own directory (`vendor/tyra/engine/Makefile`), which reads nothing of the game's.
So the editor's build first asks the game's make for them: `make
tyra-engine-vars` in the project prints `TYRA_MEMORY_MANAGER=<0|1>
TYRA_MEMORY_CHECKS=<0|1>`, and those words go on the engine make's command line
(`tools/toolchain/native-build.sh`; the Docker backend does the same in
`src/runner.cpp`). By hand: `make -C vendor/tyra/engine TYRA_MEMORY_MANAGER=1`.

make does not track a value from its command line, so `Makefile.base` keeps the
two in a stamp, `obj/.tyra-memory-switch`, rewritten only when they change, and
every C++ object depends on it. Changing the switch therefore rebuilds the game's
C++ and the engine's C++ once (not the VU microprograms). The engine's build cache
is shared by every project an editor builds, so building a project with the other
setting rebuilds the engine's C++ too. A game and an engine built with different
values do not work together - without the hooks in the library the link fails on
`undefined reference to __wrap_malloc`; with different checks the two disagree
about the layout of a `Ptr` - which is what the forwarding prevents.

### The game's registry

The game describes its budgets and pools in one function, which the engine calls
at boot on a registry that already holds the engine's rows:

```cpp
#include "memory/memory.hpp"

namespace Tyra {
namespace Memory {

void registerGameBudgets(Registry& r) {
  r.budget(Budget::FrameScratch).regionBytes = 64 * 1024;
  r.budget(Budget::FirstGame, "Actors", Kind::Logic).extensionBytes = 64 * 1024;
  // pools: r.pool(PoolDesc{...})
}

}  // namespace Memory
}  // namespace Tyra
```

`registerGameBudgets` is declared in `inc/memory/manager.hpp`; the engine's own
definition is weak and empty, so a game without one runs on the engine's rows as
they are: one heap - every budget in measure mode, charged and counted in
System's region - and no scratch.

### At boot and every frame

With the switch on, the engine (`src/engine.cpp`):

1. **first thing in its constructor** - after `EngineOptions::writeLogsToFile` is
   applied, so the lines go where the log goes - calls `Memory::install()`,
   fills a `Registry` (the engine's rows, then `registerGameBudgets`), starts
   `Manager::instance()` with it and, when it started, prints the table
   (`printTable()`). A registry that does not fit fails right there, with the
   table printed before the `[MEM] FAIL` line;
2. **every frame**, first thing after `HardwareTrace::beginFrame()`, calls
   `frameStart()` and `publishTelemetry()`, inside a hardware trace span named
   `Memory`.

What is allocated before `install()` - static constructors, libstdc++'s start-up,
the engine's member objects (built before its constructor's body runs) - is the
C library's and the manager never sees it. From `install()` to `start` the hooks
send allocations to the C library and count them on the table's `newlib boot`.

### The engine's budgets and their scopes

| budget | what is charged to it | the scope |
|---|---|---|
| `System` | every allocation with no scope open, and every allocation from another thread (audio, streaming) | - |
| `FrameScratch`, `LoadScratch` | nothing routed: they are the scratch regions (`Scratch::frame()`, `Scratch::load()`), 0 bytes until a game sizes them | - |
| `Renderer` | what the renderer allocates when it starts, the retained command cache's entries, the bbox cache's (a bag's package boxes), in the generated game a model part's far tier, and the static pipeline's copy pool of compact runs (grown the first time a package needs it; refused, that package draws nothing this frame, the pool asks again the next, and `expandIndexed` - the clip route's decode of an indexed package into it - logs a `[MEM] fallback` line) | `Engine::initAll`, around `renderer.init`; `StaPipRetainedCommands::acquire`, around a new entry's blocks and a resized entry's; `StapipBagBBoxesCacher::getBBoxes`, around a new entry or a new split; the generated `applyGeoLod`, around a tier's first build; `StaPipQBuffer::pack`, around a pool block |
| `Skinning` | a skinned instance's mesh, its skin output and its LODs' buffers | the `SkelInstance` constructor and `SkelInstance::create`'s probe of the room it will take (the model's bind data is built first, in stages that each probe their exact size: `buildBindCache`), `ensureOutput`, which gives a LOD its skin output the first time it is used, the indexed packaging (`Indexed::Shared::build`, every table probed at its size) and `SkelRenderSlots::layout` (the slot objects, lists and table) - a stage a full budget refuses makes nothing and the body keeps the cheaper thing it has (`docs/animated-models.md`, "Memory that is refused") |
| `BakedStreams` | the baked VIF stream cache: a new entry (the table of entries grows before the scope opens, in the caller's budget), and every compact stream it packs - a float bag's, its spare, a strip expansion's (since the compact vertex format, docs/compact-vertex-format.md "Routes") | the tail of `StaPipBakedStreams::acquire` that makes an entry; `StaPipBakedStreams::pack` (a stream or its spare), and `StaPipQBufferRenderer::fillCachedStrip` (an expansion's stream) |
| `Audio` | what the audio system allocates when it starts | `Engine::initAll`, around `audio.init` |
| `Level` | the game's level loading | none in the engine: a game opens it where it loads |

**What System serves of the renderer's.** Only what a scope above covers is
charged to a render budget. Every other allocation the renderer makes while a
game plays is System's, in System's region, and halts when System finds no
room: the two static caches' tables of entries growing (the `storage.reserve`
in `StaPipRetainedCommands::acquire` and `StaPipBakedStreams::acquire`, a
throwing vector growth, taken before the entry's scope opens), the
`SkelInstance` object itself (`create`'s `new`, before its constructor's scope
opens), the strip-subrange cache's one object, a texture upload's descriptors,
the dynamic and legacy pipelines' per-mesh bags and packets, and what
libstdc++ and ps2sdk allocate on their behalf. Most of it is small or
short-lived, but it is render memory in System's region; only the scoped sites
can be refused.

`Renderer`, `Skinning` and `BakedStreams` are `Render` budgets, so they refuse
when full: their call sites take memory with `new (std::nothrow)` / `memalign`
and do without when it is refused. `Audio` and `Level` are `Asset` budgets and
halt: a level that does not fit while it loads stops with
`[MEM] FAIL budget full name=Level` and the table. The render sites that run
while a game plays each have a fallback, and say so with a `[MEM] fallback`
line (`Manager::noteFallback`):

| site | refused | the line |
|---|---|---|
| a moved payload's compact stream, repacked into the entry's spare (`StaPipBakedStreams::pack`; the allocation under the BakedStreams scope) | the bag is drawn from the copy pool that frame, and the repack is asked again the next | `compact stream: packed into the copy pool` |
| a new baked entry (`acquire`) | no entry: the bag is packed into the copy pool, asked again next frame | `baked entry: packed into the copy pool` |
| a strip expansion's stream (`fillCachedStrip`) | the ordinary expansion: its package is packed into the copy pool | `expansion: packed into the copy pool` |
| a retained entry's blocks, new or resized | no entry, or the old storage kept: the ordinary route | `retained entry: ordinary route`, `retained resize: ordinary route` |
| a bag's package boxes (`StapipBagBBoxesCacher`, the first time a vertex buffer is drawn) | the bag is not drawn this frame - drawn without culling it could wrap the GS window - and asks again the next | `bbox cache: bag not drawn` |
| a model part's far tier (the generated game's `applyGeoLod`, the first time the part is that far away) | the part keeps the tier it shows - tier 0, or one it holds - and asks again the next frame it wants the tier | `model tier: kept the tier shown`, `asset=tier-<n>-vertices` and the wanted tier |
| a LOD level's skin output (`SkelInstance`) | the part skins and draws the cheapest level it holds; its pose keeps updating | `skin output: drew lod <n>`, the part's name and the wanted level |
| a whole skinned instance (`SkelInstance::create`) | no instance: the object plays on without a body - nothing drawn, its bones read false - and the generated game asks again every frame | `skinned instance: no body yet`, the model's first part's name |

A cache's `asset=` is `bag-<n>-vertices` (the bags carry no name) and its
`lod=` -1; a line is once a second per kind of fallback and per mesh (its vertex
count), and a skin output's per model, part and level - not per instance, so a
crowd short of memory writes a handful of lines, not one per fighter.
`SkelInstance`'s constructor still takes its memory with a throwing `new` (a
body's full-mesh output, its mesh, its pose scratch), and an instance is made
while a game plays - a fighter spawned or joining, an object streamed back in -
so nothing calls it straight: `SkelInstance::create` first takes, in the
`Skinning` scope, one block of what the constructor will take
(`SkelInstance::bytesFor`) plus a margin, gives it back, and answers null when
it is refused - the constructor runs only where its requests fit ([animated
models](animated-models.md)). The bbox cache and the generated game's model
tiers ask first the same way. A probe is a CALL to the nothrow
`operator new[]`, never a `new (std::nothrow)` expression whose block goes
unused: GCC may drop such an allocation - its null check folded to "not null" -
and at -O3 it did, so the probe never refused. A scope opens only where memory
is taken: at start-up, and on a path that runs every frame (a new cache entry,
an arena, a skin output, a probe) only around the allocation itself, so a frame
that allocates nothing opens none - a scope asks the thread's id twice. An
engine allocation with no scope around it is System's, render data or not. A
game opens its own the same way
(`MemScope scope(Budget::Level);`).

### The hooks and the EE platform

Each `__wrap_<name>` calls the manager with the caller's return address (the code
that called `malloc` or said `new`, which is what a failure line names):

| call | goes to |
|---|---|
| `malloc` | `allocate(size, 16)`: null when a refusing budget has no room |
| `operator new` / `new[]` | `allocateOrFail(size, 16)`; 0 bytes asks for 1, and it never answers null - a budget with no room is a failure, which halts, in a refusing budget too (`throwing new in a refusing budget`) |
| `new (std::nothrow)`, `new (std::nothrow) T[]` | `allocate(size, 16)`; 0 bytes asks for 1; null when a refusing budget has no room. Wrapped itself: libstdc++'s calls the throwing `new` inside a `try`, which would halt |
| the aligned `new` of all four (`new T` for a `T` aligned to more than the EE's 8 bytes - a `Vec4`, a matrix) | as their unaligned twins, at the type's alignment. Wrapped too: libstdc++'s aligned `new` turns a null from `aligned_alloc` into a `bad_alloc` nothing catches |
| `calloc` | `allocate`, then zeroed; null with `errno = ENOMEM` when count x size overflows |
| `memalign`, `aligned_alloc` | `allocate(size, align)` |
| `posix_memalign` | `EINVAL` unless the alignment is a power of two and a multiple of `sizeof(void*)`, else `allocate` (newlib has no `posix_memalign` of its own: before `install()` it is `memalign`) |
| `realloc` | `reallocate` |
| `free` | `release` |

Until `install()` has run, every hook passes the call straight to the C library
(`__real_<name>`). The test is a plain global that `install()` sets last: the
first call of `Manager::instance()` (its static's guard) may itself allocate, and
it happens inside `install()`, before the switch.

The EE `Platform` that `install()` gives the manager:

| function | on the EE |
|---|---|
| `rawAlloc`, `rawFree`, `rawUsable` | the C library's `memalign` and `free` (`__real_`), `malloc_usable_size` |
| `available` | `mallinfo().fordblks` plus what `sbrk` can still give below `EndOfHeap()` - the two reads `Info::getFreeRAMSize` makes; neither allocates |
| `threadId` | `GetThreadId()` |
| `lock`, `unlock` | `DIntr()`, which answers whether interrupts were on, and `EIntr()` only when they were |
| `print` | one line appended to the log file with `fopen` / `fputs` / `fclose` when `Info::writeLogsToFile`, else to stdout, flushed. The path is the one `FileUtils::fromCwd("log.txt")` gives, computed once by `install()`; on a read-only medium (`cdrom...`) there is none and the line goes to stdout - the rule `TyraDebug::writeInLogFile` follows |
| `halt` | `__builtin_trap()` |

ps2sdk's file layer allocates through `malloc` when it opens a file, so while
`print` runs, the printing thread's allocations go to the C library: a line never
re-enters the manager, not even the one that says `System` is full.

### The newlib reserve

newlib's own allocations - stdio's buffers, `strdup`, the internals of `printf`
- call `_malloc_r`, not `malloc`, so `--wrap` never sees them. They are served by
what the C library has left after the manager's block: the fit check keeps
`settings.newlibReserveBytes` (256 KB unless the game changes it) out of the
block. A block newlib made and the program frees with `free` goes back to newlib
through the manager (`release` of an address outside the block). The table's
`newlib` line shows what is left (`free=`) against the reserve.

`Info::getAvailableRAM()` - the debug HUD's `MEM` - reads the manager once it
has started: System's free bytes plus the movable heap's (`Totals::systemFree +
heapFree`), the game's free memory. Before `start`, and without the manager, it
reads the C library as it always did. Its `TYRA_MEM_VERIFY` probe (malloc until
it fails) runs only then: with the manager it would fill System, which halts.

### The `[MEM]` lines

Every line goes through the platform's `print`, so into `log.txt` beside the ELF
when the engine writes a log:

| line | when |
|---|---|
| `[MEM] budget <name> kind=<kind> region=<KB> heap=<KB> full=<Halt\|Refuse>`, `[MEM] pool ...`, `[MEM] total fixed=... system=... reserve=...` | at boot: the registry ([The table](#the-table)) |
| `[MEM] budget <name> cap=<KB> used=<KB> peak=<KB> exhausted=<n> refused=<n>`, `[MEM] newlib boot=... free=... reserve=...`, `[MEM] heap cap=... free=... largest=... moved=... ptrs=...` | at boot, right after: the manager's stats ([Stats and the table](#stats-and-the-table)) |
| `[MEM] FAIL <what> name=<name> size=<n> caller=0x<hex>` | any failure ([Reporting a failure](#reporting-a-failure)); the game's handler runs, then the game halts |
| `[MEM] refused name=<budget> size=<n> caller=0x<hex> frame=<f>` | a refusing budget answered a request null ([When a budget is full](#when-a-budget-is-full)); once per call site per `refusalReportFrames` frames; never with `refusalLog` off |
| `[MEM] fallback <what> asset=<asset> lod=<lod> frame=<f>` | a render site did without (`noteFallback`); once per asset per `refusalReportFrames` frames, 16 lines a window at most; never with `refusalLog` off |
| `[MEM] fallback lines suppressed=<n> frames=<a>-<b>` | the fallback lines dropped in that window past the 16 |
| `[MEM] unscoped caller=0x<hex>` | with the checks: a game-thread call site that allocates with no scope open, once per site |
| `[MEM] pool <name> exhausted +<n> policy=<policy>` | at a frame start, for a pool that was full since the last one |
| `[MEM] dead ptr at 0x<hex> target=0x<hex>` | with the checks: the frame start's sweep found a `Ptr` to a released entry |

A `caller` is an EE address; `mips64r5900el-ps2-elf-addr2line -f -C -e
bin/<game>.elf.sym 0x<hex>` names the function and the line (a debug build keeps
the unstripped `.elf.sym`).

### The telemetry block

`tyra_memory_budgets` is a 3744-byte block in `.data` (C linkage, kept by
`__attribute__((used))`) that `publishTelemetry()` rewrites every 10th frame, for
a tool that reads EE RAM while the game runs - an emulator's PINE, a debugger.
Its magic, version and size are in the ELF's file image, so a tool finds the
block's address without symbols. The layout is `Tyra::Memory::TelemetryBlock`
(`inc/memory/telemetry.hpp`, every size `static_assert`ed) and
`writeTelemetry(block, manager, frame)` (`src/memory/telemetry.cpp`, portable:
the verifiers run it on the host) fills it. Little-endian integers:

| offset | field | |
|---|---|---|
| 0 | `char magic[8]` | `TYRAMEM3`, no terminator: version 2 of the layout |
| 8 | `u32 seq` | odd while the block is being rewritten |
| 12 | `u32 count` | the budget rows filled, in budget id order (at most 48) |
| 16 | `u32 frame` | `publishTelemetry` calls so far, the one that wrote the block included |
| 20 | `u32 systemFree` | `Totals::systemFree` |
| 24 | `u32 heapFree` | the movable heap's free bytes |
| 28 | `u32 heapLargest` | its largest free block |
| 32 | `u32 movedLastFrame` | bytes the last frame start's step moved |
| 36 | `u32 allocsLastFrame` | `allocate` and `reallocate` calls in the last frame, every thread |
| 40 | `u32 ptrs` | registered `Ptr`s |
| 44 | `u32 version` | 2 |
| 48 | `u32 blockBytes` | 3744 |
| 52 | `u32 defragBytesPerFrame` | the step's budget (`Settings`) |
| 56 | `u32 largestRequest` | `largestRequest()`: the largest `allocate` size since the last publish |
| 60 | `u32 ringHead` | `frame()`: the newest frame in the ring |
| 64 | `u32 regionCount` | the region rows filled (at most 16) |
| 68 | `u32 heapDebt`, `heapBlocked`, `heapPinned`, `heapOversize`, `heapOversizeBytes`, `heapBuckets` | `heapHealth()` (oversize: the buckets bigger than the budget that still have to move) |
| 92 | `u32 heapFramesToCompact` | `heapHealth().framesToCompact`: the steps compaction takes; 0xFFFFFFFF never (a budget of 0) |
| 96 | 48 budget rows of 36 bytes | `char name[12]` (the budget's name, cut to 12 characters; zero-padded when shorter, with no terminator when it fills all 12), `u16 id`, `u8 kind`, `u8 onFull` (0 halts, 1 refuses), then `u32 cap`, `used`, `peak`, `exhaustions`, `refused` - `budgetStats(i)`, sizes in bytes |
| 1824 | 64 ring entries of 8 bytes | `u32 bytes`, `u16 buckets`, `u16 oversize` - `defragRing()`: frame `f` at entry `f % 64`, for `f` in `(ringHead - 64, ringHead]` |
| 2336 | 16 region rows of 88 bytes | `u16 id` (0xFFFF: the movable heap), `u16 cells`, `u32 capacity`, `freeBytes`, `largestFree`, `freeBlocks`, `largestRequest`, `u8 map[64]` - `regionStats(i)` |

A reader copies the block and keeps the copy only when `seq` was even and the
same before and after it. The ring holds 64 frames and a publish comes every 10,
so a reader that keeps a copy at least once every 54 frames (0.9 s at 60 Hz) sees
every frame: it takes the frames past the last one it saw, up to `ringHead`.

What the block costs the game: the manager keeps every number in it as it goes
([Fragmentation](#fragmentation)), so a publish copies them, sums each map's 64
cells and walks the movable heap's buckets - every 10th frame, and never a walk
of a routed region. A reader costs the game nothing: it reads memory from
outside.

Version 1 (`TYRAMEM2`, 1584 bytes) was the first 48 bytes of this header - its
offset 44 a reserved 0 - and 48 budget rows of 32 bytes: the name, then `cap`,
`used`, `peak`, `exhaustions`, `refused`. `TYRAMEM1` came before it, with 16-byte
names and no `refused`. The magic changed with each layout, so a tool tells them
apart by it.

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

Today there are eight verifiers:

- `heap` - alignment, coalescing in every release order, exhaustion and exact
  reuse of a hole, the over-aligned cases, `slideDown`, the accounting, the
  checks, the free-block count and the occupancy map (attached late and filled
  from the blocks there, a block over several cells, a hole, a slide, a tiny
  block reading 1, over-aligned gaps, `init` keeping the map, a range that is no
  multiple of 64 cells, a range under 1 KB, the map taken away, and 3 000 random
  allocations, slides and releases - each cell's bytes and fraction and the
  count compared with an address-order walk after every one), and a
  100 000-operation seeded stress run (sizes 1 to 65 536, alignments
  16/64/128, every payload filled with its own byte and checked at release,
  blocks compared for overlap, the heap, its free-block count and its map
  checked against the walk as it goes);
- `failure` - the exact line, the order print / handler / halt, a missing
  handler, the texts cut to fit, and a failure raised inside the handler;
- `registry` - the engine rows and the budget and pool calls, the bucket sizes for
  every part of the formula and the offsets of the parts (aligned, ordered,
  adding up to `bucketBytes`), the order, alignment and non-overlap of the layout,
  the fit check to the byte at its edge and each reason it answers `false` (and
  that it never calls `fail`), sizes too big for a word, and the exact table
  lines;
- `pool` - the pools of linked buckets against a stand-in bucket source that fills
  memory with junk and frees released and moved buckets for real: entries
  distinct, inside their bucket and aligned, extensions requested with the
  budget, token, size, alignment and pinning the pool must ask for, the three
  policies (Refuse and Halt to the byte of the failure line, RecycleOldest by
  age across buckets, after a release, after a move and with a source that has
  no room), `releaseEmptyExtensions` with its spare and its gaps (and a gap
  taken by the next extension), a moved bucket carrying its entries, stamps and
  generations, the accessors at their edges, every failure of the checks and,
  without them, that bad calls are ignored, a damaged free list (a zeroed link,
  a link to a live entry, one out of range, one to the entry itself, damage in
  an extension: a failure with the checks, and without them no live entry
  handed out twice; a link to another free entry going unnoticed), ids and
  generations (stale after release, recycle and a bucket given back), and a
  30 000-step seeded run against a model that fills, drains, moves and
  refuses;
- `ptr` - `Ptr<T>` against a fake movable heap (a static arena cut into 32 KB
  slots whose buckets the test moves by hand with `memmove`): what is and is not
  registered, the table size after every kind of construction, copy, move,
  assignment and `reset` (self-assignment too), the token of each row, a
  `std::vector` of `Ptr`s growing and shrinking, and a full table (`ptr table
  full` for a new row, none for a move, a re-keyed or a replaced one, and a
  failed copy leaving its target alone). `resolve` is checked with rings of
  `Ptr`s that live in a bucket and point into it, `Ptr`s outside, a `Ptr` in one
  bucket pointing into another, a pointer into the middle of an entry, a bucket
  sliding over part of its own old place (down and up), two buckets exchanging
  places in one call, a token-0 `Ptr` inside a moved bucket, and 200 seeded
  rounds (three moves each) of random links and random moves compared with the
  links that were made - after each the table has no row at the old places and
  every destroyed `Ptr` leaves none behind. The table by itself runs against a
  `std::map` model through thousands of random inserts, erases, re-keys and
  `dropBucket`s at capacities from 1 to 1024, checking after each operation that
  every row is still reachable, and is filled to the last slot and emptied in
  three orders.
  `dropBucket`, `init` and its failures, the pin hook (with, without and half a
  hook), no registry installed, and - with the checks - the id hook (ids carried
  through copy and move, `get()` checking only a `Ptr` that has an id, the `ptr
  to a dead object` line to the byte) complete it;
- `defrag` - the movable heap over a static arena with the `Ptr` registry
  installed, on its own and under `PoolCore`s whose `bucketMoved` is its move
  listener. The brief's case first: 36 entries in 8 extensions, `Ptr`s held
  outside to every third and a ring of `Ptr`s through all of them, three
  extensions released, 20 steps of 64 KB, then the largest free block is bigger,
  nothing is left but one free block, every entry holds its index, the ring and
  the held `Ptr`s are right and (with the checks) their ids are still valid. The
  budget to the byte (three 40 KB buckets behind a hole move one a step, a budget
  of exactly two moves two, one byte short moves one, a 100 KB bucket moves alone
  in a step of its own and the smaller ones wait their turn, a budget of 1 still
  moves a bucket a step), the 32-move cap, holes between buckets merging, a bucket
  that cannot move not ending the walk (pinned, held by a `Pin`, over-aligned),
  never-moved buckets (a `Pin`, nested pins, a pin on any byte of a bucket, a pin
  on memory in no bucket, pins of a released bucket, a pin on every bucket at
  once, an allocation made pinned, an over-aligned one, a `Render` budget's and an
  over-aligned type's pool extensions), a `Ptr` that lives in the last bytes of a
  moved bucket, the caps to the byte, the bytes an over-aligned block's gap leaves
  with its neighbour, `tokenAt` at both ends of 300 buckets and outside the heap,
  `releaseBucket` dropping the `Ptr`s that pointed into it, the health of a
  hand-built layout (holes in front of a pinned bucket, of a 100 KB oversize one
  and of the last: debt, blocked, fixed bytes, the buckets and bytes still to
  move, oversize and the frames to compact to the byte, with a bigger budget,
  a budget of 0, a `Pin` and an over-aligned bucket; the two steps that pay it
  off, the oversize one alone, each re-estimated after; the largest bucket asked
  for, a refused one included), the compaction estimate against the steps
  themselves (100 small buckets behind a hole: four steps of the 32-move cap;
  48 seeded layouts of small, big and oversize buckets with pinned, held and
  over-aligned ones between, at 8 KB and 64 KB - the frames, the bytes and the
  moves predicted are the ones the steps then took), a heap with no
  registry or no listener, the pin hook installed and taken back, every failure
  to the line, a full table of buckets refused like any other request and
  answered by each of the three pool policies, 12 seeded runs of 3 000 random allocations, releases, pins,
  `Ptr`s inside and outside buckets and steps (every bucket's bytes, every
  `Ptr`'s target and every table row checked as it goes; the plain runs end fully
  compacted), and a seeded run of two pools with entries linked by `Ptr`s,
  acquiring, releasing, giving extensions back and stepping.
- `scratch` - the region (alignment of the address, padding, the last-block
  roll-back, exactly full, sizes that would wrap, `reset` erasing nothing, `init`
  starting over), the two instances, every container really living in the region
  and giving the right answers, byte containers 16-aligned after any odd-sized
  block, over-aligned types, 200 frames of the same work
  needing the same room, the `scratch full` line to the byte, and with checks the
  `scratch kept past its frame` failure on `allocate` and `deallocate`, for both
  regions, with allocators of one epoch equal and of two not; without checks an
  allocator that is an empty class;
- `manager` - the manager on a fake platform (newlib is the host's `malloc`
  behind a counting, aligning wrapper; the thread id is a variable; the lock
  records being taken twice or left taken; `print` collects lines; `halt`
  throws). `Manager::instance()` first: allocations before `start` going to the
  C library and coming back to it after (freed and realloc'd), the block taken
  once at the layout's size and alignment and nothing else after, the globals
  and hooks installed; routing (unscoped and other-thread allocations to System,
  a scope's own region, a measured budget in System's region charged to itself,
  nested scopes), an over-aligned block realloc'd with no byte lost, `realloc`
  staying in a full budget and failing there with the block intact while System
  has room (also from another thread), the frame and load scratch resets, the
  allocation count of a frame (allocate and reallocate, every thread);
  `Pool<T>` (construct, destroy, `forEach`, `Refuse` to the cap, `Halt`,
  `RecycleOldest` destroying the oldest, a type or alignment that does not match
  its row, an over-aligned type), the exhaustion lines once per frame, a ring of
  `Ptr`s through an extension that a `Pin` holds and then the defragmentation
  moves (the pool following its bucket), the ids and the dead-`Ptr` sweep with
  the checks; the stats and the table lines; and a registry that does not fit.
  Managers of its own then cover every refusal of `start` (System's
  `regionBytes`, a region too small, rounding to 16, too many extensions, the
  block's alignment, no platform), the scopes' failures and other threads, the
  unscoped report to its 256th site, `release` and `reallocate` of every kind of
  address, and a 6 000-step seeded run of three budgets sharing System's region
  with over-aligned blocks, reallocs and releases in random order, each budget
  ending where it started to the byte. Full budgets: a `Render` budget (and an
  `Asset` one the game made refuse) answer null and count it while a `Logic` one
  and an engine `Asset` row (`Level`) fail; `allocate` (the nothrow `new`) answers null where `allocateOrFail`
  (the throwing one) fails with `throwing new in a refusing budget`; a refused
  `realloc` keeps its block; the `[MEM] refused` line once per call site per
  `refusalReportFrames` (and still for the 74th site), `noteFallback` once per
  key and at most `kMaxFallbackLinesPerWindow` lines a window (the rest
  counted in a `suppressed` line); the counts in the stats and the table; with
  `refusalLog` off, refusals and fallbacks counted with no line at all while a
  failure still prints. The headroom: a refusing budget in
  measure mode stops `refuseHeadroomBytes` short of the end of System's region
  (a request bigger than everything left refused too) while System and a halting
  measured budget still get that headroom, a refusing budget with its own region
  is not held to it, and with a headroom of 0 it takes the last block, then
  refuses. Fragmentation: the region rows (System, the budgets' own regions by
  id, no scratch, the movable heap last), a fragmented System region's free
  blocks, largest block and map cells to the byte, the largest request per
  region (served, refused, from a measured budget, a growing realloc) and its
  reset, the defrag ring over 70 frames of pool extensions moved one a frame
  (each oversize, the ring wrapping; after every frame `heapHealth()`'s frames to
  compact are the frames that still moved something), the heap's debt paid off,
  and
  `writeTelemetry` - every header field, budget row (a 12-character name with no
  terminator), ring entry and region row against the calls it copies, `seq`
  even after each publish, a manager that has not started left alone, and the
  request window restarted.
