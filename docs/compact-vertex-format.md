# The compact vertex format

The compact vertex format stores a static-pipeline vertex in 16-bit fixed point
- 8-bit where that is exact - laid out the way the VIF unpacks it, so VU1 memory
still sees four 32-bit lanes a vertex while EE RAM and the DMA carry about a
third of the bytes: a textured, coloured vertex is 14 bytes instead of 48 (three
`Vec4` / `Color` quadwords). The format is defined in
`vendor/tyra/engine/inc/renderer/3d/pipeline/static/core/compact/stapip_compact.hpp`
(namespace `Tyra::Compact`): the unpack formats, the boxes positions are
quantized in, the quantizers, and the VIF words a bag sends once and a run sends
in front of its data. That header
includes only `tamtypes.h`, so the editor and the host verifier run exactly the
engine's code.

## The formats

The VIF widens each format to 32-bit lanes while the data streams into VU
memory, at no EE or VU cost; a program converts a vector with one `ITOF`.

| attribute (`Compact::Attr`) | stored | unpack | lanes from COL | in the VU program |
|---|---|---|---|---|
| `Position` | int16 x, y, z in the bag's box (6 bytes) | V3-16, signed, masked, offset MODE (+ ROW) | w = 1.0 | `itof0.xyz` of the grid index; the step rides in the MVP |
| `St` | int16 u, v, 4.12 fixed point (4 bytes) | V2-16, signed, masked | z = w = 1.0 | `itof12.xy` |
| `Color` | u8 r, g, b, a (4 bytes) | V4-8, unsigned | - | unlit: used as it lands; lit and clip: `itof0` |
| `Normal`, `EnvNormal` | int8 x, y, z (x 127), 0 (4 bytes) | V4-8, signed | - | `itof0.xyz`; the 1/127 rides in the light / env rows |
| `BbCenter` | a billboard's centre, as a position | as `Position` | w = 1.0 | `itof0.xyz` |
| `BbParams` | a billboard's 2x2 basis weights, int16 x 4 (8 bytes) | V4-16, signed | - | `itof0` |
| `BbColor` | a billboard's colour, as a colour | V4-8, unsigned | - | used as it lands |

A run of `n` elements of one attribute is padded to whole quadwords
(`runQwords`): 75 positions are 450 bytes, 29 quadwords; 75 STs, colours or
normals 300 bytes, 19 quadwords.

**Masked lanes come from the COL register.** A V2 or V3 unpack leaves its
missing lanes undefined, and the programs need a position's w to be 1.0 and an
ST's z to be 1.0 (they multiply the whole ST by Q). So those lanes are masked
and the VIF writes them from COL, which every compact bag sets to 1.0f in all
four words (`Compact::kCol`); a masked lane reads COL of its write cycle, cycle
0 under STCYCL 1/1. The mask word (`maskWord`) holds 2 bits a lane - 0 data,
2 COL - in the same pattern for all four write cycles: `0x80808080` for a
position (w), `0xA0A0A0A0` for an ST (z and w). ROW is not a constant any more:
it carries the box's origin (below).

**V3-16 or V4-16 positions** (`kPositionFormat`): V3-16 is 6 bytes a vertex,
V4-16 8 bytes with the fourth halfword unused. w comes from COL either way, so
the VU programs are the same and the constant is the only switch.

## The bag setup and the run header

A compact bag sends three quadwords once, in front of everything else it
unpacks - its object data included (`bagSetup`, `kBagSetupWords` = 12):

    STCYCL 1/1, STMOD 0, STROW, row x, row y, row z, 0, STCOL, 1.0f, 1.0f, 1.0f, 1.0f

ROW is the box's origin as grid indices (`rowFor`), COL the 1.0 the masked lanes
read. Every run then follows one header quadword, four words with the MODE in
word 2 (`runHeader`):

| run | header words |
|---|---|
| position, billboard centre | `STMASK`, `0x80808080`, `STMOD 1` (offset), `UNPACK` |
| ST | `STMASK`, `0xA0A0A0A0`, `STMOD 0` (normal), `UNPACK` |
| colour, normal, env normal, billboard weights and colour | `NOP`, `NOP`, `STMOD 0`, `UNPACK` |

That is the quadword a baked block already spends on two NOPs, the STCYCL and
the UNPACK ([baked-vif-stream.md](baked-vif-stream.md)): STCYCL moves to the bag
setup, and its slot sets the MODE, so the mask and the MODE cost no byte. The
payload's padding words are zero: VIF1 reads the words after an unpack's data
as VIFcodes, and zero is a NOP. `Packet2TyraUtils::addCompactRun` sends a header
and its payload with one REF whose own VIFcode slots are NOPs.

**MODE is offset on a position run and normal everywhere else**, and that is a
rule for the whole chain, not just for compact runs: MODE applies to every
UNPACK, masked or not, and stays set until the next STMOD. In offset MODE the
VIF adds ROW to each data lane (the origin index, below); anything else unpacked
after a position run - the next run, a package's buffer header, the next bag's
object data - would get the origin added too. So every non-position run header
sets normal MODE, the bag setup does, and a package's entry quadword must (its
STCYCL slot is free for it now that the bag setup sets STCYCL). **No compact
stream or bag ever leaves the VIF in offset MODE**: a position run is never the
last unpack before another pipeline's data, so a package's kick quadword
carries STMOD 0 as well - a single-colour C package is a position run and
nothing else, and whatever unpacks after it must not get the origin added.

**Every UNPACK word is built by hand** (`unpackCode`). The ps2sdk helpers always
set USN - bit 14 of the immediate, "unsigned" - and every signed format needs it
clear. Bit 15, FLG, is "add TOPS" (the double buffer); `num` 256 is written as
0; the address is 10 bits. The VIF-hash gate tells geometry from uniforms by FLG
(`vifUnpackIsGeometry` in `stapip_vif_hash.hpp`); it read bit 14 before this
format, see [baked-stream-acceptance-gate.md](baked-stream-acceptance-gate.md).

## Boxes, and the seam rule

A position is stored as an integer in its bag's **box**: one step for x, y and z
(isotropic, so a uniform scale never bends a normal), a power of two, and an
origin that is a whole number of steps. The integer is

    q = round(v / step) - round(origin / step)

rounded half away from zero and saturated to +-32767 (-32768 is never written,
so the range is symmetric): the vertex's index on the step's grid less the
origin's. On the EE it decodes to `origin + step * q`. On the VU it needs no
decode: the VIF adds the origin's index back as it unpacks the run (offset MODE,
ROW = `rowFor(box)`, a 32-bit integer add), so VU memory holds the grid index
`round(v / step)` itself, `itof0` turns it into an exact float (every index is
under 2^24 in magnitude), and the step rides in the matrix the VU multiplies by
anyway: `MVP x D(step, 0)`, where `D` is the step on the diagonal with no
translation (`dequant`, column-major like `M4x4`).

**World bags** - a bag whose model matrix is exactly the identity - share the
**scene step** (`sceneStepFor`): the smallest power of two, at least 2^-10 m, at
which the scene's largest world bag fits wherever it lies. A box spans 65 534
steps. Its ends' grid points land 65 535 apart only when both are exact half
steps either side of 0 (half away from zero moves each out by half a step), so
the step holds 65 533 and that case never arises: 2^-9 m (1.95 mm) holds 127.99
m, 2^-8 m 255.99 m. `worldBox` puts the origin on the scene grid; a bag wider
than 65 534 steps takes the next coarser power of two that holds it, for itself
only, and says so (`coarse`). Every world bag on the scene step uploads the
same matrix, `MVP x D(sceneStep, 0)`.

**The seam rule: a vertex two world bags share reaches the VU as the same grid
index, under the same matrix** - so its clip-space position, and with it the
screen position and the depth, is the same bit for bit in both bags, and a
coplanar pass (a terrain layer, the AO pass, a light pool on its receiver)
lands on exactly the pixels and depths of its base. Two things make it hold:

- **The same index.** With a power-of-two step, `v / step`, the rounding and
  `origin / step` are all exact, so `q + origin / step` is the vertex's index on
  the scene grid whatever box it is in. Subtracting the origin first,
  `round((v - origin) / step)`, is not: `v - origin` rounds when `v` is small
  and the origin far away, and a value a hair below half a step can round onto
  it. 2^-10 - 2^-34 in a box at -32 m (step 2^-9) reads one step up that way and
  0 in a box at 0 m - a 2 mm crack along the seam. Exact operations also make
  the integers the same on every platform: the EE's FPU and VU0 round toward
  zero, a host to nearest, and data the editor quantized agrees with data the
  console quantized. Half away from zero is computed as `trunc(2x) - trunc(x)`,
  never as `floor(x + 0.5)`, which reads `0.5 - 2^-25` as 1 under
  round-to-nearest.
- **The same matrix.** A per-bag `MVP x D(step, origin)` - the box's origin as
  the matrix's translation, and the box-local integer on the VU - decodes to the
  same float on the EE but not on the VU: the translation column `MVP x origin`
  rounds differently for every origin, so the multiply that follows disagrees in
  the last bits, and a review probe found 130 of 20 000 shared vertices on a
  different 1/16-pixel or depth integer - sparkles along chunk and batch seams,
  z-fighting between coplanar passes. Moving the origin out of the matrix and
  into the VIF's integer add leaves one matrix for every world bag.

**Any other bag** uses a box of its own (`modelBox`), one per model - every part
and LOD tier of it - and its own `MVP x D(step, 0)` (the model's MVP), with ROW
the model box's origin index. The step is the smallest power of two at or above
the largest extent / 65 534 (a 2 m body: 2^-14 m, 0.061 mm), twice that in the
rare case both ends are exact half steps either side of 0; the origin is a whole
number of steps near the box's centre. A model further than 2^24 steps from its
own origin takes coarser steps until it is not. With a power-of-two step every
operation is exact, so a vertex comes back within half a step - no more - on
every platform, and the EE and the host write the same integers.

## ST

ST is 4.12 fixed point: +-8 repeats at 1/4096 of a repeat (`quantizeSt`,
saturated to +-32767 - so a value fits when `|x x 4096|` rounds inside 32767,
which -8 itself does not). A package whose ST does not fit is rebased by whole
repeats, each coordinate on its own (`stRebase` gives the shift: 0 when the
package already fits, else the floor of its middle, or one repeat either side
of it when that is the shift that fits - a package at u in [0.6, 15.0] would
leave 8.0 at the top under the floor, 7, and takes 8). Wrap addressing makes
the shift invisible - a package is drawn alone - and the fraction is kept at
1/4096: a package at u in [100.2, 101.7] stores u - 100. A package up to 15
repeats wide always rebases into range (the shifts that fit form an interval
at least a repeat long, within a repeat of the middle); one up to 16 wide
does when its ends allow, and one that fits no shift takes the floor of its
middle and saturates. There is no per-bag ST scale (it would cost a VU
instruction a vertex) and no split.

## Colour and normals

**Colour** is u8 a channel (`quantizeColor`: rounded, clamped to 0..255) - the
GS takes 8 bits a channel, so nothing is lost, and the textured nominal 128 is
exact. An unlit program uses the integers as they land: no `itof`, no `ftoi0`.
A single colour for a whole bag is uploaded as four integer lanes and copied,
never added. A lit program, or one that interpolates (the clip family), converts
with `itof0` and then does its usual float math.

**Normals** are int8 x 127 as `(x, y, z, 0)` (`quantizeNormal`: rounded,
clamped to +-127; `|n - q / 127| <= 1/254`). The 1/127 rides in the rows that
multiply the normal anyway - the folded light rows, the light matrix, the env
basis - so it costs no VU instruction. Env normals are normalised when they are
written.

## The VU side

Every static-pipeline program that reads a bag's vertices reads this format:
the five `cull` and the five `clip` programs (generated by `src/vugen.cpp`)
and the two `billboard` programs (hand-written). The `as_is` family keeps float
input - the EE clipper hands it NDC floats. Each attribute is converted once,
right after it is loaded:

| attribute | in the program |
|---|---|
| position, billboard centre | `itof0.xyz` - the grid index; w is COL's 1.0 |
| ST | `itof12.xy` - z and w are COL's 1.0 |
| normal, env normal | `itof0.xyz` |
| billboard weights | `itof0` |
| colour | unlit cull and billboard: none - stored as the u8 it is, no FixColor; spot-lit cull and every clip program: `itof0`, then the float math |

That costs no micro memory overall: measured with the toolchain's `size` when
the programs were adopted, the resident sets are 1 710 words for the
VU1-clipping set (1 712 before) and 1 434 for the EE-clipper set (1 434), 1 908
and 1 632 with the billboards (1 918 and 1 640) - the unlit paths' FixColor
goes, and the colour clamps a u8 makes redundant with it.

**What a compact bag uploads** (the EE side of the contract; the programs rely
on every line):

- `MVP x D(step, 0)` - the step on the diagonal, no translation (`dequant`);
  the VIF adds the origin's grid index.
- The rows that multiply a normal x 1/127: the folded light rows the default
  `_fold` images read (or the light matrix), and the env basis's x and y lanes;
  the env basis's X.w is **127** - the numerator of the renormalising env
  path's rsqrt, which normalises to the int8 normal's length so the one basis
  serves both the renormalising and the unit-normal (`_envn`) images.
- A single colour as four integers with the alpha **one higher** than the
  material's: the programs copy it (`move`); the float programs added `vf00`,
  so the GS always got alpha + 1.
- The spot light in grid units - its position / step, the 1 / range^2 and the
  softness factor x step^2 (all exact with a power-of-two step) - and its
  colour clamped at 0: the spot-lit cull loop keeps only FixColor's ceiling.
- A billboard's right and up basis divided by the step and by the weights'
  power-of-two scale (the weights are stored x that scale - the stream's
  `weightScale()`, below).
- The default-off paint images: the paint basis x 1/127.

**The wrappers** send each block as one run: per block `k` of the program's
layout, `Packet2TyraUtils::addCompactRun(packet, qbuffer->run[k],
qbuffer->runQwords[k])`, a debug build asserting that the run header's UNPACK
address is where the layout puts the block; the single-colour block is skipped
as before. `StaPipQBuffer::run[3]` points at each block's header quadword with
its payload behind it, `runQwords[3]` holds the payload's size.

**Proven, not assumed:** `--vu-check` runs every compact program against its
float reference on the same quantized draws (docs/vu-framework.md, "Compact
input"): positions, ST and unlit colours bit-identical, the renormalised env ST
within 4e-5 relative, every GS word within the GS window.

## The stream

A bag's vertices live in one `StaPipCompactStream`
(`vendor/tyra/engine/inc/renderer/3d/pipeline/static/core/compact/stapip_compact_stream.hpp`):
the one copy, laid out exactly as the baked VIF block a wholly visible bag
replays with one `REF` ([baked-vif-stream.md](baked-vif-stream.md)), with the
compact runs in place of the float ones. The core takes float arrays and
includes only `tamtypes.h`, so the host verifier runs it;
`stapip_compact_tyra.hpp` adds the engine's types - `Vec4` and `Color` in and
out of a writer or a view, and a `StaPipBag`'s arrays as a packer's source.

**The layout** (`StaPipCompactLayout`) is the program's: the attributes in
vugen's `attrBlocks` order - position; then, when textured, ST or the env
normal; then normals when lit, else colours unless one colour serves the whole
bag (the env program is never lit) - or a billboard's centres, weights and
colours (`forClass`, `forBillboards`, and `forBag` for the program StaPipCore
picks for a bag). `packageSize` is the bag's package size (3 to 256 - an UNPACK
moves at most 256), `dest(a, n)` the VU1 address a package of `n` vertices
unpacks attribute `a` to: `VERT_DATA + a x n`, where the generated wrappers put
it. A stream holds, per package `p`:

| quadwords | what |
|---|---|
| 3 (`entry(p)`) | the count / scale quadword, the prim GIF tag and their UNPACK - the program's, captured by the renderer; NOPs until then |
| 1 + `runQwords(p, a)` per attribute | the run header (`runHeader(p, a)`, written when the stream is allocated) and the run (`run(p, a)`): the package's elements in vertex order, zero-padded to whole quadwords |
| 1 (`kick(p)`) | FLUSH and MSCAL, the renderer's too |

so `qwords()` is the sum over packages of `3 + sum(1 + runQwords) + 1`, and a
75-vertex textured, coloured package is 74 quadwords (1 184 bytes: 3 + 30 + 20
+ 20 + 1) where the float baked block was 232. The bag setup (`Compact::bagSetup(box)`: STCYCL,
STMOD 0, the box's origin index in ROW, COL) is **not** in the block: the
renderer sends it once per bag, in front of the bag's object data and so of
anything the bag unpacks. Beside the block, in the same allocation, the
stream keeps the boxes a frustum classifier reads, made when the stream is
written - no vertex is read to classify it: one box per third of a package (a
*part*, `packageSize / 3` vertices - the split StaPipCore classifies with;
`partBoxes()`, six floats a part, contiguous), one per coarse group of 24
parts - eight full packages, `coarseBoxes()` - and one for the whole stream
(`bounds()`), each the union of the parts it covers, so a classifier forms no
union of its own. That is 24 bytes a part, about 0.6 % of a textured 75-vertex
package; the `StaPipBagPackagesBBox` the float bags' cache builds (some 170
bytes a part) is never made for a stream.

**Allocation** (`allocate(layout, vertexCount, box)`) is one nothrow block -
the boxes in it, nothing else allocated - 16-byte aligned, in whatever memory
scope is open (the caller's budget), zeroed, its run headers written and its
weight scale back at 4096 (as `release()` leaves it). It answers false, keeping nothing, when the memory is
refused, and for what cannot make a stream: no vertices, a package size outside
3..256, or a box that is not a `worldBox` / `modelBox` one (a power-of-two step,
an origin a whole number of steps whose index is under 2^24 - only such a box
decodes to the same float on the EE, a host and the VU). `contentVersion()`
moves on every allocate and every `finish()`. **Allocate releases the old
block first, and a packer rewrites a stream that already matches in place**:
neither may touch a block a DMA chain - an in-flight packet, a recorded frame -
still reads. A stream that was drawn is replaced by a second one (ping-pong),
never written again until no bank leases it.

**The writer** (`StaPipCompactWriter`) fills a stream in vertex order: per
vertex the attributes its layout has, in any order (one it lacks is ignored),
then `next()`; `end()` completes the last package and calls `finish()`, which
makes the boxes from the stored integers (each part's the smallest box holding
its positions as they decode, then the coarse groups and the whole stream's
from those) and moves `contentVersion`. Positions are quantized
in the stream's box as they come (`Compact::quantize`); colours, normals and
billboard weights likewise; an env normal is normalised as it is written. ST is
held until its package is complete and then rebased by whole repeats, u and v
each on their own (`stRebase`; a strided overload reads a bag's `Vec4` ST in
place): a package at u in [100.2, 101.7] stores u - 100. A package whose ST no
shift brings inside - wider than 15 repeats, a triangle spanning about 16 -
saturates and is counted (`stClamped()`, packages), for the editor's bake to
warn about. A billboard stream's weights are stored x its weight scale, so set
it (`setWeightScale(Compact::weightScaleFor(largest))`) before writing them.

**The view** (`StaPipCompactView`) reads a stream back as floats: a position as
the grid point the VU sees, step x (q + the origin's index); ST as stored (the
rebased value); colours 0..255; normals q / 127; billboard weights / the
stream's `weightScale()`. `positions(first, n, out4)` decodes a range in one
call (below). An attribute the layout lacks reads 0. A view reads the layout
and box when it is made.

**Packing** (`Compact::packArrays(layout, source, box, stream)`, and
`Compact::packBag(bag, packageSize, box, stream)` for a float bag's own arrays
on the EE) writes exactly the bytes a writer given the same values writes -
positions through the batch quantizer - keeping the stream's block when it
already has that layout, count and box. A billboard stream's weight scale is
the largest power of two that keeps its largest weight inside int16
(`Compact::weightScaleFor`); the renderer divides the uploaded basis by it.
This is the engine's packing step: a producer that still hands the engine float
arrays gets its bag packed when the content changes.

**Boxes** (`Compact::boxFor`, `boxForBag` on the EE): a bag whose model matrix
is exactly the identity is in world space and takes `worldBox` at the scene
step, anything else `modelBox`. The scene step (`setSceneStep` / `sceneStep`)
is set by the game at scene load - `sceneStepFor` its largest world bag - and is
2^-9 m until then; a world bag that does not fit logs `[COMPACT] coarse box
bag=<address> extent=<m>` once (for the first 16 bags).

**The batch paths** (`stapip_compact_ee.cpp`): `Compact::quantizePositions`
and `decodePositions` take a vertex a pass through VU0 macro mode and MMI on the
EE - `VMUL` by 1/step, `VMINI` / `VMAX` to +-2^29 (`FTOI0` saturates at 2^31 and
the `PSUBW` after it would wrap), two `FTOI0` and a `PSUBW` for half away from
zero, `PSUBW` the origin's index, `PMINW` / `PMAXW` to +-32767, `PPACH`; and
back with an unaligned `LDL` / `LDR`, `PEXTLH` and `PSRAW`, `PADDW` the origin's
index, `VITOF0`, `VMUL` by the step - and the same steps in C++ elsewhere. Every
step is exact, so they are bit-identical to `quantize` and to the view's
per-vertex decode on the platform they run on. `Compact::selfTest()` holds them
to it: it is `knownAnswers()` (the format's own known answers, what links with
`stapip_compact.cpp` alone) plus the batch paths against the per-element
functions on inputs that are numbers everywhere - the rounding edges,
saturation, -0, a denormal. NaN and infinity are not: the EE's FPU and VU0 have
neither and take the pattern as a large number (a host's `quantize` reads NaN as
0), and an emulator's clamping settings decide what it becomes.

`StaPipBag::compact` points at the stream a bag draws from; null means the
float arrays are the source, and the engine packs them.

## Verifying it

`python tools/verify-tyrax2-host.py --only compact` builds
`tools/verify-stapip-compact.cpp` with `stapip_compact.cpp`,
`stapip_compact_stream.cpp` and `stapip_compact_ee.cpp` and checks: every
attribute's format, sign, mask, MODE and sizes; every UNPACK, STMASK and STMOD
word and the bag setup (STCYCL, STMOD, STROW, STCOL) against the hardware's
field layout; every run of every attribute, `n` = 1 to 255, walked behind a bag
setup through `vif1_chain_check`'s VIF1 model (the unpack ends where the data
ends and the padding reads as NOPs); scene steps and world boxes (origins on the
grid, coarse past 65 534 steps); 10 000 random seam edges whose vertices decode
bit-identical in both boxes - a quarter of them built where subtracting first
fails, which it then does; 20 000 more taken through what a draw does - the
VIF's offset add of each bag's ROW, `itof0`, the VU's multiply chain under one
scene `MVP x D(step, 0)` - with 0 clip-space differences, where a per-bag
`MVP x D(step, origin)` gets nearly all of them wrong in the last bits; model
boxes of every size and place (power-of-two steps, half a step round trip,
every index an exact float); ST, its rebase, colour and normals; and the exact
paths again under every float rounding mode the host can select. Then the
stream, for every layout (TC, TCE, TD, D, C with colours and with one colour,
billboards, a stripped TC in runs of 75) and vertex counts from 1 to 1 000: its
quadwords, every run header, the entry and kick quadwords' places, a VIF1 walk
of the whole block behind the bag setup (each UNPACK expects exactly its run,
nothing pending at the end); what the view decodes (positions within half a
step, ST equal to the rebased value at 1/4096, colours exact, normals within
1/254, env normals unit within 1/127); every decoded position inside its part
box and the box tight, the coarse groups and the whole stream's box the exact
unions of the part boxes, and the whole box tight; the packer's bytes equal
to the writer's; colours in the block exactly when the layout carries them;
the batch decode equal to the per-vertex one for ranges across packages; a
package at u in [100.2, 101.7] storing u - 100 and a 20-repeat triangle counted once;
packages up to 24 repeats wide counted exactly when no whole-repeat shift fits
(every shift tried, not the rebase's three), and the rebase finding a fit
whenever one exists across 20 000 random packages;
refused and impossible allocations returning false with nothing leaked (the
verifier counts every `new[]`); the boxes for bags and the scene step; and the
batch paths against the per-element functions, NaN and infinity included, under
every rounding mode. `Compact::selfTest()` holds known answers that are exact
under any rounding mode (the quantizers' edge cases, the VIF words, the boxes)
and the batch paths, so a game can run it on the console to show the EE agrees
with the host that wrote its data.
