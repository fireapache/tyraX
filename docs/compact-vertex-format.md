# The compact vertex format

The compact vertex format stores a static-pipeline vertex in 16-bit fixed point
- 8-bit where that is exact - laid out the way the VIF unpacks it, so VU1 memory
still sees four 32-bit lanes a vertex while EE RAM and the DMA carry about a
third of the bytes: a textured, coloured vertex is 16 bytes instead of 48 (three
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
| `St` | int16 S, T, Q, 4.12 fixed point x the package's scale (6 bytes) | V3-16, signed, masked | w = 1.0 (unused) | `itof12.xyz` |
| `Color` | u8 r, g, b, a (4 bytes) | V4-8, unsigned | - | unlit: used as it lands; lit and clip: `itof0` |
| `Normal`, `EnvNormal` | int8 x, y, z (x 127), 0 (4 bytes) | V4-8, signed | - | `itof0.xyz`; the 1/127 rides in the light / env rows |
| `BbCenter` | a billboard's centre, as a position | as `Position` | w = 1.0 | `itof0.xyz` |
| `BbParams` | a billboard's 2x2 basis weights, int16 x 4 (8 bytes) | V4-16, signed | - | `itof0` |
| `BbColor` | a billboard's colour, as a colour | V4-8, unsigned | - | used as it lands |

A run of `n` elements of one attribute is padded to whole quadwords
(`runQwords`): 75 positions or STs are 450 bytes, 29 quadwords; 75 colours or
normals 300 bytes, 19 quadwords.

**Masked lanes come from the COL register.** A V3 unpack leaves its fourth
lane undefined, and the programs need a position's w to be 1.0 (an ST's w is
never read - the GS takes S, T and Q - but it is masked the same way, so the
lane is defined). So that lane is masked and the VIF writes it from COL, which
every compact bag sets to 1.0f in all four words (`Compact::kCol`); a masked
lane reads COL of its write cycle, cycle 0 under STCYCL 1/1. The mask word
(`maskWord`) holds 2 bits a lane - 0 data, 2 COL - in the same pattern for all
four write cycles: `0x80808080` (w) for a position and for an ST. ROW is not a
constant any more: it carries the box's origin (below).

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
| ST | `STMASK`, `0x80808080`, `STMOD 0` (normal), `UNPACK` |
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
A package built fresh ends with the SDK's FLUSH + MSCAL kick, which has no
slot for an STMOD; so every uniform upload the renderer makes - a bag's object
data, the EE clipper's other uniform forms, an instanced copy's MVP - starts
with a tag whose VIFcodes are FLUSHE and STMOD 0 (`Compact::kUniformHeadCodes`,
the renderer's `flusheHeader`).

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

ST is S, T and Q - a texture coordinate is S/Q and T/Q, which the GS divides
per pixel - each in 4.12 fixed point (`quantizeSt`, +-8 at 1/4096, saturated to
+-32767), all three times a power of two chosen per package, the package's
**scale** k. An ordinary texture has Q = 1; a projective one - a lamp's or a
flashlight's pool, a projected shadow - carries the light's forward distance
in Q and S, T in the same units (docs/flashlight.md), so its values are metres,
not repeats, and its Q is anything but 1. The programs convert all three
(`itof12.xyz`) and multiply them by the vertex's 1/w as they always did; the
GS's S/Q and T/Q cancel k exactly, and the clip programs, which interpolate
S, T and Q linearly, keep it a common factor. (k cancels in the PICTURE because
the static pipeline's texture LOD ignores Q: `StaPipCore::setLod` sets TEX1 to
a fixed K - `LOD_USE_K`, `K` 0, `MXL` 0, the same filter up and down - so no
mip level is chosen. A Q-based LOD, or mipmaps, would read Q x k and move the
level by m; the format would then have to undo k first.) So a package is
stored in two steps (`StaPipCompactWriter`, `Compact::packArrays`, `Compact::packRun`):

- **The rebase.** Each texture coordinate whose texture REPEATs on that axis
  is moved by whole repeats (`stRebase`: 0 when the package already fits, else
  the floor of its middle, or one repeat either side of it when that is the
  shift that fits - a package at u in [0.6, 15.0] would leave 8.0 at the top
  under the floor, 7, and takes 8). The stream stores S - n x Q, which is exact
  on a projective STQ too: (S - n Q) / Q = S / Q - n, and the GS's
  perspective-correct interpolation of S and Q keeps it. Wrap addressing makes
  the shift invisible - a package is drawn alone - and the fraction is kept: a
  package at u in [100.2, 101.7] stores u - 100.
- **The scale.** k is the largest power of two at most 1 that keeps the
  package's largest |S|, |T| or |Q| inside 4.12 (`stScaleFor`; at least
  2^-24). A package that fits as it stands - every package of an ordinary
  texture up to 15 repeats wide, and one up to 16 whose ends allow it - has
  k = 1 and is stored exactly as plain 4.12, with Q = 4096. One that does not
  - a TERRAIN strip row of 64 m at a tile of 0.5 a metre spans 32 repeats
  (examples/impostor-grove's ground), a lamp's pool 12 m out - takes a
  smaller k instead of saturating: k = 2^-m stores S, T and Q at 2^m / 4096
  (m bits of fraction lost, never a wrapped or clamped value). With Q = 1
  that is S/Q to 2^m / 8192 of a repeat (half a stored unit); a projective
  STQ divides by its own Q, so its S/Q is good to about
  (2^m / 8192) x (1 + |S/Q|) / Q - lighting's k = 1/8 at Q = 0.5 m is about
  1/256 of the pool's texture (`verify-stapip-compact` holds both bounds). The
  writer counts the worst (`stScaleBits()`), and the engine prints each new
  worst once on stdout - PCSX2's own log (`-logfile`), not the game's
  log.txt, because the packer builds on the host too: `[COMPACT] ST scale
  2^-m: a package's S, T and Q keep m fewer bits of fraction`. Measured in
  PCSX2 over the examples' first 400 frames: the worst package of
  examples/impostor-grove (its 32-repeat strip rows), deep-forest and
  night-walk takes k = 1/4 (S and T at 1/1024 of a repeat); lighting's takes
  1/8 (1/512); cube, large-terrain, material-lab, reflections, texture-atlas
  and vu-lab keep k = 1 in every package.

That costs 2 bytes a textured vertex over S and T alone (6 instead of 4: a
75-vertex textured package is 10 quadwords longer) and nothing on the VU -
`itof12.xyz` is the instruction `itof12.xy` was - and it removes both of the
format's ST limits: the hard 16-repeat ceiling and the projective Q, which an
S, T-only format (Q from COL as 1.0) dropped, so lamp pools did not draw.

**Only REPEAT hides the rebase.** A texture that CLAMPs on an axis (a lamp's
pool, a projected shadow - `Texture::getWrapSettings`, anything but
`WRAP_REPEAT`) would move by a whole-repeat shift, so that coordinate is never
rebased (`StaPipCompactLayout::stFixed`, bit 0 u, bit 1 v, set by `forBag` from
the texture's wrap; part of the cache key, so a wrap that changes packs
again); the package's scale holds its values as they are. There is no
per-bag ST scale (it would cost a VU instruction a vertex) and no split: the
scale is per package and rides in the data, S, T and Q alike.

## Colour and normals

**Colour** is u8 a channel (`quantizeColor`: truncated, clamped to 0..255) - the
GS takes 8 bits a channel, so nothing is lost, and the textured nominal 128 is
exact. Truncated, not rounded, because the float programs truncated what they
handed the GS (`FixColor`'s `ftoi0`): an unlit colour reaches the GS bit for
bit as before, a lit one at most a level lower (rounding put a level on about
half of every picture's pixels), and a single colour's upload truncates the
same way. An unlit program uses the integers as they land: no `itof`, no `ftoi0`.
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
| ST | `itof12.xyz` - S, T and Q x the package's scale; w is COL's 1.0 |
| normal, env normal | `itof0.xyz` |
| billboard weights | `itof0` |
| colour | unlit cull and billboard: none - stored as the u8 it is, no FixColor; spot-lit cull and every clip program: `itof0`, then the float math |

That costs no micro memory overall: measured with the toolchain's `size`, the
resident sets are 1 708 words for the VU1-clipping set (1 712 before the
compact format) and 1 434 for the EE-clipper set (1 434), 1 906 and 1 632 with
the billboards (1 918 and 1 640) - the unlit paths' FixColor goes, and the
colour clamps a u8 makes redundant with it; S, T and Q convert with the one
`itof12.xyz` S and T did, and TC's shared clip image no longer converts its
env path's z on its own (1 710 and 1 908 before the Q change).

**What a compact bag uploads** (the EE side of the contract; the programs rely
on every line). `StaPipQBufferRenderer::sendObjectData` writes all of it, and
the stamp replay (docs/stapip-stamps.md) re-derives the same values:

1. **The bag setup first** - `Compact::bagSetup(box)` once per bag, right
   behind the bag's FLUSHE and so before anything it unpacks, its object data
   included (four quadwords: the FLUSHE's CNT tag carries the twelve words).
2. **`MVP x D(step, 0)`** - the step on the diagonal, no translation
   (`dequant`; the first three columns scaled, exact with a power of two); the
   VIF adds the origin's grid index. Every world bag on the scene step uploads
   the same product - one scene-wide D - and a model bag its model's. The
   stamp replay and the instanced copies (`appendBakedInstance`) multiply it
   in the same way.
3. **No compact stream leaves VIF1 in offset MODE**: every package's entry
   quadword carries STMOD 0 (the program's count / GIF-tag CNT now carries
   STMOD 0 and its UNPACK - STCYCL 1/1 is the bag setup's), a stream's kick
   quadword carries STMOD 0 beside FLUSH and MSCAL - both written into the
   quadword's first slot whatever the program wrote
   (`StaPipCompactStream::setPackageWords`) - and so does every packet's END
   tag. A package built fresh (its program's entry, a `REF` of its runs, the
   SDK's FLUSH + MSCAL tag) has no slot left in its kick: the MODE a
   positions-only package leaves is reset by whatever unpacks next - the
   next package's entry, the next bag's setup, the packet's END, or the
   FLUSHE + STMOD 0 tag in front of every uniform upload
   (`Compact::kUniformHeadCodes`: the EE clipper's other uniform forms
   between buffers, an instanced copy's MVP).
4. **A single colour as four integer lanes**, RGB **clamped to 0..255** and
   truncated (the compact programs no longer saturate it - 300 would wrap to
   44), the alpha **one higher** than the material's (the float programs
   added `vf00`, so the GS always got alpha + 1): the programs copy it
   (`move`).
5. **The spot light in grid units** - its position / step, the 1 / range^2
   and the softness factor x step^2 (all exact with a power-of-two step) - and
   its colour clamped at 0 (in `buildSpotForBagInverse`, which the stamp
   replay builds through too): the spot-lit cull loop keeps only FixColor's
   ceiling. The EE clipper keeps the object-space constants (it reads decoded
   positions).
6. **The env basis's X.w is 127** - the numerator of the renormalising TCE
   paths' rsqrt, which normalise to the int8 normal's length - so one basis
   serves both the renormalising and the unit-normal (`_envn`) images. Without
   it every frustum-crossing env package would draw one constant matcap ST.
7. **1/127 in every row that multiplies a normal**: the folded light rows the
   default `_fold` images read, the light matrix (the unfolded arm a program
   override keeps), the env basis's x and y lanes, and - default off - the
   paint basis.
8. **A billboard's right and up basis divided by the step and by the
   weights' power-of-two scale** (the weights are stored x that scale - the
   stream's `weightScale()`, or for a billboard bag packed as it is drawn, the
   scale of its largest weight).

**The EE clipper's buffers** (a scene with `"clipping": "precise"`) are drawn by
the `as_is` programs, which keep float input and read two of these uniforms
in their old form: a float single colour (they add `vf00` and saturate
themselves) and an env basis without the 1/127 (they renormalise to length
1). Around those buffers the renderer swaps the two in - a FLUSHE, then the
other form - and back for the next compact buffer (`emitUniformForm`); only
in that mode, only for a bag that has one of them. The clipper reads its
input decoded (`StaPipQBuffer::decodeToFloats`): positions the grid points
the cull programs draw, so a crossing triangle meets its neighbours exactly,
S, T and Q as stored (the package's scale is a common factor the clipper's
interpolation keeps), and normals as their int8 values, which the light rows'
1/127 expects. Its float arrays are allocated nothrow: a refusal skips that
package's clip this frame.

**The wrappers** send each block as one run: per block `k` of the program's
layout, `Packet2TyraUtils::addCompactRun(packet, qbuffer->run[k],
qbuffer->runQwords[k])`, a debug build asserting that the run header's UNPACK
address is where the layout puts the block and that it is TOPS-relative (FLG,
bit 15 - `(w & 0x83FF) == (0x8000 | addr)`); the single-colour block is skipped
as before. `StaPipQBuffer::run[3]` points at each block's header quadword with
its payload behind it, `runQwords[3]` holds the payload's size.

**Proven, not assumed:** `--vu-check` runs every compact program against its
float reference on the same quantized draws (docs/vu-framework.md, "Compact
input"): positions, ST and unlit colours bit-identical, the renormalised env ST
within 1.4e-4 relative (the int8 normal's own quantization; under 2e-7
absolute, which is the arm that judges it), every GS word within the GS
window.

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
75-vertex textured, coloured package is 84 quadwords (1 344 bytes: 3 + 30 + 30
+ 20 + 1) where the float baked block was 232:

| program class (75 vertices) | runs | quadwords | bytes | bytes a vertex | float baked block |
| --- | --- | ---: | ---: | ---: | ---: |
| `cull_tc` (texture + colour) | position, STQ, colour | 84 | 1 344 | 17.9 | 49.5 |
| `cull_tc`, single colour | position, STQ | 64 | 1 024 | 13.7 | 33.3 |
| `cull_td` (texture + lights) | position, STQ, normal | 84 | 1 344 | 17.9 | 65.7 |
| `cull_tce` (matcap) | position, env normal, colour | 74 | 1 184 | 15.8 | 49.5 |
| `cull_c` (colour) | position, colour | 54 | 864 | 11.5 | 33.3 |

The STQ run is 29 quadwords of 75 vertices (6 bytes a vertex); S and T alone
were 19 (the Q change put Q in the data: every textured package is 10
quadwords, 160 bytes, longer). The bag setup (`Compact::bagSetup(box)`: STCYCL,
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
moves on every allocate and every `finish()`, from one counter for every
stream: it names one stream's content at one moment, so a cache keyed on a
block's address and the version (the strip subranges, the expansion cache)
never takes a freed block's entry for a new stream allocated at its address -
a per-stream count made every fresh pack read the same number. **Allocate releases the old
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
billboard weights likewise; an env normal is normalised as it is written. ST
comes as `st(u, v)` (Q = 1) or `stq(s, t, q)` (projective) and is held until
its package is complete, then rebased and scaled ("ST", above): a package at u
in [100.2, 101.7] stores u - 100 at scale 1, a 20-repeat triangle stores at
half scale. `stScaleBits()` is the most fraction bits a package gave up, for
the editor's bake to warn about; `stClamped()` counts packages where a value
still saturated - only past 2^27 repeats or for an infinity. A billboard
stream's weights are stored x its weight scale, so set it
(`setWeightScale(Compact::weightScaleFor(largest))`) before writing them.

**The view** (`StaPipCompactView`) reads a stream back as floats: a position as
the grid point the VU sees, step x (q + the origin's index); `st` the texture
coordinate the stream holds, S/Q and T/Q (the rebased value, exact for an
ordinary package), `stq` the three lanes as the VU reads them (x the package's
scale - `Compact::st` on a view hands that back as a bag's `Vec4`); colours
0..255; normals q / 127; billboard weights / the stream's `weightScale()`. `positions(first, n, out4)` decodes a range in one
call (below). An attribute the layout lacks reads 0. A view reads the layout
and box when it is made.

**Packing** (`Compact::packArrays(layout, source, box, stream)`, and
`Compact::packBag(bag, packageSize, box, stream)` for a float bag's own arrays
on the EE - a bag's ST `Vec4` is S, T, Q, its z the Q every float program has
always read) writes exactly the bytes a writer given the same values writes -
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

## Routes

Every static-pipeline bag draws from a compact stream
(`StaPipQBufferRenderer::acquireStream`, `setCompactSource`): its own
(`StaPipBag::compact`), or - for a bag that hands the engine float arrays -
the one the renderer packs it into, an entry of the baked-stream cache
(`StaPipBakedStreams`: the same key - the arrays, their bbox and content
versions - budget, LRU and ping-pong as before; [baked-vif-stream.md](baked-vif-stream.md)).
A bag whose stream is packed classifies with the stream's own boxes, read in
place (`StaPipBagPackager::setRenderStream`); only a float bag the cache does
not hold this frame still takes the bbox cacher's, from its floats.

**A float bag that changes every frame** - a skinned body re-posed by the EE,
a mesh re-shaded in place - is packed ONCE per change, into its entry's spare,
and every draw of that frame replays it (split screen's second view reuses
the first view's pack; a body posed every 4th frame is packed every 4th
frame). Packing is the engine's whole cost for such a producer - the float
route used to REF its arrays where they lay - so it is kept small:

- the spare takes the new pose's box in place (`StaPipCompactStream::fits`,
  `rebox`): a body re-posed every frame allocates nothing;
- a repack keeps the runs whose attribute did not move
  (`Compact::packArrays`' `keepFrom` / `keepSlots`, from the entry's
  `packedStamps`): positions and normals move together (`bboxVersion`), ST and
  colours only with their own content stamp - a body's ST never changes
  between poses, so it is copied, not quantized again; the vertices' content
  stamp moving packs everything;
- normals go through a batch path (`Compact::quantizeNormals`, VU0 and MMI on
  the EE), and a plain ST package - every Q exactly 1 - skips the general
  path's second pass (`Compact::stqRun`), both bit-identical to the general
  ones (the host verifier, and `selfTest` on the EE).

Only a SHAPE that keeps changing (a vertex count or layout that moves on two
frames within `kChurnGapFrames`) churns, and is packed into the copy pool as
it is drawn. Measured in a 24-fighter split-screen match (crossings-coop):
before this the bodies churned and were packed per package at every draw -
about 10 000 vertices a frame, 15-20 ms of EE: positions 1.4 ms, ST 12.8,
normals 5.3.

**A bag's own stream must be the layout its program reads**
(`StaPipCompactLayout::forBag` at the bag's package size: the attributes, the
package size, strips, the fixed ST axes). Every route's fallback - the
classifier's boxes, a package packed into the pool - reads float arrays a
native bag need not have, so `acquireStream` refuses any other stream: a bag
with float arrays draws from them instead, one without is not drawn
(`StaPipCore::streamRefused()` says so right after its `render`) and
`[COMPACT] refused bag=<address> ...` is logged once (for the first 16 bags).

| route | what the VU's input is |
| --- | --- |
| wholly visible, entries captured for this program and state | ONE `REF` over the whole stream (`replayWholeBakedBag`); a stream over 65 535 quadwords: one `REF` per run of consecutive packages, entry to kick |
| wholly visible, entries captured for another program or state | per package the program's fresh entry, ONE `REF` of its header + run pairs, the kick |
| a crossing package, VU1 clipping | the package straight out of the stream to the clip program: fresh entry (its own plane mask), one `REF` of its runs; the partial replay (`emitPartialRuns`) sends runs of culled packages as one `REF` of the stream and each clipped package so |
| a slice, merged subpackages, a strip expanded to a list | packed into the qbuffer copy pool from the stream's elements, copied as they are (`Compact::copyRun`) |
| the strip-expansion cache | a stripped package expanded once into a list stream of clip-sized packages; a chunk that is one of them is drawn from it in place, any other range from the pool |
| a float bag the cache does not hold (churning, refused, billboards) | each package packed into the copy pool as it is drawn, from the floats (`Compact::packRun` - exactly what a stream holds), in a box from the bag's bounds |
| the EE clipper | the buffer's runs decoded to floats (`Compact::decodeRunToFloats`), clipped, sent to the `as_is` programs as floats |

**The EE clipper** (`"clipping": "precise"`) reads floats in the `as_is`
programs' convention (`Compact::decodeRunToFloats`, what
`StaPipQBuffer::decodeToFloats` runs): positions in the box's units, S, T and
Q as stored (x the package's scale, which its linear interpolation keeps a
common factor), normals as their int8 values (the light rows carry the 1/127),
env normals unit length, colours as their u8 values; a spot light is added to
the colours at the decoded positions in object space, as before. Measured on
examples/lighting with EE clipping (2026-11-03): with its time-driven content
pinned - every torch's flicker 0, its physics props static - the compact build
against the float engine differs on 9.5% of the pixels by at most 2 levels and
on 0.06% by more (silhouette edges, a 1-pixel move of a grid-snapped vertex),
the quantization's signature the VU1 route shows too. As authored, the floor
around its torches differs far more, and differs between two boots: a torch's
flicker and a physics prop run on the measured frame time (the generated
game's `g_frameDt`), which grows when a frame misses its field, so builds of
different EE cost - EE clipping costs the EE more than VU1 clipping - reach a
captured frame at a different flicker phase and prop pose. That is the scene's
clock, not the format.

**Captured entries.** A stream's entry and kick quadwords are the program's:
the renderer runs the program's own writers once per package and copies their
words in (`captureEntries`, through `StaPipCompactStream::setPackageWords`) -
never re-derived, the rule the baked block always kept - with STMOD 0 in each
quadword's first slot. Package 0 carries MSCAL and the
GS-state flag, the rest MSCNT, as a whole-bag replay needs. They are written
once, into a stream whose entries were never captured - so no chain can be
reading them - and the key they were captured for is kept
(`StaPipCompactStream::entryKey`: the program, its micro-memory address, the
prim state, the three scales of the count quadword). A draw whose key differs
sends fresh entries; a program cache rebuild clears the cache and they are
captured again.

**Stamps** ([stapip-stamps.md](stapip-stamps.md)) replay a wholly visible bag's
head and its ONE `REF`, for a cache stream and for a bag's own stream alike.
Their key holds the bag's own stream pointer and its `contentVersion()`, so a
native bag's stream repacked under the same pointers - or a bag stamped from
the cache whose own stream became drawable later - is captured again, never
replayed stale.

**The copy pool** holds compact runs now: per slot and pool side one block,
grown nothrow to what the slot's largest package takes (a refused block draws
the package next frame); the float arrays stay for the EE clipper only. Both
grow in the Renderer budget's scope with the memory manager - a refusing
budget, so a full region skips that clip this frame instead of halting. A
buffer's runs lie back to back, so a package goes as its program's entry and
ONE `REF` of them (`StaPipQBuffer::bodyQwords`) - a game-supplied program keeps
its own writers.

**Immutable spans.** A cache stream is registered when it is packed and
retired when it is replaced or evicted; its memory waits in the graveyard
until no bank reads it (the cache's teardown drains VIF1 first, then collects
every span). Only a stream's own block is allocated in the BakedStreams
budget's scope - that budget refuses instead of halting, so nothing that
allocates with a throwing `new` (the graveyard growing as evictions bury
streams) may run inside it. A bag's own stream is registered the first time it is
drawn and unregistered by its `release()` (which waits for every reader). The
float arrays are no longer registered (`StaPipBag::immutableStreams`): no DMA
tag names them.

**What goes**: the runtime normalisation of env normals (the packer
normalises them), the transcoded float arena and its payload patches (a moved
payload is packed again into the entry's spare), `fillCachedPackage` (a
crossing package is in an immutable span already), and the partial-baked cull
and whole-clip arenas (the stream serves both).

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
every index an exact float); ST, its rebase and its scale (the largest power
of two that fits, over 20 000 random magnitudes), colour and normals; and the exact
paths again under every float rounding mode the host can select. Then the
stream, for every layout (TC, TCE, TD, D, C with colours and with one colour,
billboards, a stripped TC in runs of 75, TC and TD with a projective STQ) and
vertex counts from 1 to 1 000: its
quadwords, every run header, the entry and kick quadwords' places, a VIF1 walk
of the whole block behind the bag setup (each UNPACK expects exactly its run,
nothing pending at the end); what the view decodes (positions within half a
step, S/Q the original less whole repeats within what the package's scale
keeps, a package that fits stored exactly as plain 4.12, colours exact, normals within
1/254, env normals unit within 1/127); every decoded position inside its part
box and the box tight, the coarse groups and the whole stream's box the exact
unions of the part boxes, and the whole box tight; the packer's bytes equal
to the writer's; colours in the block exactly when the layout carries them;
the batch decode equal to the per-vertex one for ranges across packages; every
run decoded for the EE clipper (`decodeRunToFloats`) equal to what the view
decodes, in the `as_is` convention, and a clip vertex interpolated between two
decoded vertices of a package at S/Q within the package scale's bound of the
float twin's (less the package's whole repeats); a
package at u in [100.2, 101.7] storing u - 100 and a 20-repeat triangle at half
scale, exactly (nothing saturates); packages up to 24 repeats wide scaled
exactly when no whole-repeat shift fits (every shift tried, not the rebase's
three), at the largest scale that fits, and the rebase finding a fit whenever
one exists across 20 000 random packages; a projective package under REPEAT
coming back as S/Q less whole repeats, a CLAMPed lamp pool (Q 0.5 to 20) at
1/4 with S/Q and T/Q inside its error bound, and `st(u, v)` writing what
`stq(u, v, 1)` writes;
refused and impossible allocations returning false with nothing leaked (the
verifier counts every `new[]`); the boxes for bags and the scene step; and the
batch paths against the per-element functions, NaN and infinity included, under
every rounding mode. `Compact::selfTest()` holds known answers that are exact
under any rounding mode (the quantizers' edge cases, the VIF words, the boxes)
and the batch paths, so a game can run it on the console to show the EE agrees
with the host that wrote its data.

For the routes it also checks, per layout and vertex count: the entries and
kicks filled as the renderer fills them (`setPackageWords`, a program's words;
a program whose CNT carries no STMOD gets one all the same), then the stream
walked behind the bag setup with `vif1_chain_check` tracking MODE - normal
behind the setup and every entry, offset exactly on the position runs, normal
after every kick - and the same walk with package 0's kick STMOD
cleared, which must end a positions-only package in offset MODE (the check can
fail; a layout that shows it must have run); `clearPackageWords` back to NOPs;
and every package of every stream packed run by run (`Compact::packRun`, from
the floats, as a range and as an index list) and copied out of the stream
(`Compact::copyRun`) equal to the stream's run, padding included, nothing
written past it, and both equal reversed.

## What changes

Per user-facing change, against the float format (the engine before this branch):

- The compact format is the static pipeline's default; the float route remains for bags without a compact stream.
- Texture coordinates carry Q (S/Q, T/Q, the projective divide): a textured vertex is 2 bytes larger, and a package wider than 16 repeats is drawn at a coarser texture scale instead of being clamped.
- Colour quantize keeps truncation. The vu-lab posterise band therefore shifts by one level on lit colours; accepted, and the cost is one level on lit colours.
- Object-space stages of project VU looks see grid steps, not metres.
- The VIF-hash gate reads FLG (bit 15) to tell geometry from uniforms.
- Skinned bodies skin straight into their compact stream (`TYRA_SKIN_INTO_STREAMS`, default 1; `docs/animated-models.md`, "Skinning into compact streams"). On the stress view the stream path is still slower than the float VU0 skin (47.7 fps against 60.0 fps, below).

Measured: split-screen CROSSINGS (`crossings-coop`, perf-probe development build, 24 fighters) against the float baseline (work 20.47 ms, 29.96 fps): the compact build reads work 25.78 ms, 29.51 fps; crossings-solo work 15.53 ms against 20.90 ms, 44.16 fps against 32.70 fps; the stress view 57.9 fps against 47.5 fps. The baked-stream cache peaks at 1 678 KB against 5 420 KB (and 0 refused against 1 758). The residual is the packing of re-posed bodies (`Compact::packArrays`, one call per pose): about 4.9 ms a frame in split screen, of which `StaPipCompactStream::finish()` (the part boxes, a walk over every position) is 2.5 ms, positions 1.1 ms, normals 0.9 ms, and the ST copies of unmoved runs 0.3 ms. That is why the branch is held, not merged.

Measured with the stream skinning (perf-probe development build, the same
summarizer as the baseline): crossings-coop 11.65 ms of work at 29.97 fps (the
baseline reads 20.47 ms at 29.96 fps); crossings-solo 7.26 ms at 59.95 fps, not
comparable with the baseline's solo run (its game speed was 73.3 %, this run's
99.5 %: the script diverged). These two were taken before the skin core changes
below and are not re-measured.

Measured after the stream skin's bind-time extent and one-pass part boxes (paired
runs, two repeats of each, 2026-11-04, perf-probe development builds): crossings-coop
reads 11.66 ms of work and 26.82 ms of render at 29.97 fps (the float skin 20.47 ms,
25.09 ms, 29.96 fps). The work figure falls because the idle share grows; the render
pass is 1.7 ms slower at the same frame rate, so the coop gain is not a gain in the
scene pass. The stress view reads 47.7 fps at 79.5 % game speed against 60.0 fps at
99.9 % for the float skin (work 15.40 ms against 7.41 ms, render 16.09 ms against
12.83 ms). crossings-solo is not compared: its game speed differs between the builds
(73.3 % float, 99.5 % stream; the script diverges).

Earlier, before those changes, the stress view read 24.3 fps with the first stream
skin core against 57.9 fps for the float baseline.
A per-joint bound for the pose box in place of the second blend of every corner,
and a truncating round and one reciprocal per corner, bring it to 31.2 fps; the
package-by-package corner walk leaves it at 31.6 fps. Per written part, before
the bound: extent 0.44 ms, corners 0.86 ms, finish 0.12 ms. The stream path stays
below the float baseline on the stress view, so the vector skin path has to fix
it; the scalar core is the reference it must match.
