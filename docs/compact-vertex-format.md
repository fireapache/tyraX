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
STCYCL slot is free for it now that the bag setup sets STCYCL).

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
saturated to +-32767). A package whose ST leaves [-8, 8) is rebased by whole
repeats, each coordinate on its own (`stRebase` gives the shift: 0 when the
package is already inside, else the floor of its middle). Wrap addressing makes
the shift invisible - a package is drawn alone - and the fraction is kept at
1/4096: a package at u in [100.2, 101.7] stores u - 100. A package narrower
than 14 repeats always rebases into range; what still does not fit saturates. There
is no per-bag ST scale (it would cost a VU instruction a vertex) and no split.

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

## Verifying it

`python tools/verify-tyrax2-host.py --only compact` builds
`tools/verify-stapip-compact.cpp` with `stapip_compact.cpp` and checks: every
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
paths again under every float rounding mode the host can select.
`Compact::selfTest()` holds known answers that are exact under any rounding
mode - the quantizers' edge cases, the VIF words, the boxes - so a game can run
it on the console to show the EE agrees with the host that wrote its data.
