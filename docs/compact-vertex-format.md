# The compact vertex format

The compact vertex format stores a static-pipeline vertex in 16-bit fixed point
- 8-bit where that is exact - laid out the way the VIF unpacks it, so VU1 memory
still sees four 32-bit lanes a vertex while EE RAM and the DMA carry about a
third of the bytes: a textured, coloured vertex is 14 bytes instead of 48 (three
`Vec4` / `Color` quadwords). The format is defined in
`vendor/tyra/engine/inc/renderer/3d/pipeline/static/core/compact/stapip_compact.hpp`
(namespace `Tyra::Compact`): the unpack formats, the boxes positions are
quantized in, the quantizers, and the VIF words in front of a run. That header
includes only `tamtypes.h`, so the editor and the host verifier run exactly the
engine's code.

## The formats

The VIF widens each format to 32-bit lanes while the data streams into VU
memory, at no EE or VU cost; a program converts a vector with one `ITOF`.

| attribute (`Compact::Attr`) | stored | unpack | lanes from ROW | in the VU program |
|---|---|---|---|---|
| `Position` | int16 x, y, z in the bag's box (6 bytes) | V3-16, signed, masked | w = 1.0 | `itof0.xyz`; the box rides in the MVP |
| `St` | int16 u, v, 4.12 fixed point (4 bytes) | V2-16, signed, masked | z = w = 1.0 | `itof12.xy` |
| `Color` | u8 r, g, b, a (4 bytes) | V4-8, unsigned | - | unlit: used as it lands; lit and clip: `itof0` |
| `Normal`, `EnvNormal` | int8 x, y, z (x 127), 0 (4 bytes) | V4-8, signed | - | `itof0.xyz`; the 1/127 rides in the light / env rows |
| `BbCenter` | a billboard's centre, as a position | as `Position` | w = 1.0 | `itof0.xyz` |
| `BbParams` | a billboard's 2x2 basis weights, int16 x 4 (8 bytes) | V4-16, signed | - | `itof0` |
| `BbColor` | a billboard's colour, as a colour | V4-8, unsigned | - | used as it lands |

A run of `n` elements of one attribute is padded to whole quadwords
(`runQwords`): 75 positions are 450 bytes, 29 quadwords; 75 STs, colours or
normals 300 bytes, 19 quadwords.

**Masked lanes come from the ROW register.** A V2 or V3 unpack leaves its
missing lanes undefined, and the programs need a position's w to be 1.0 and an
ST's z to be 1.0 (they multiply the whole ST by Q). So those lanes are masked
and the VIF writes them from ROW, which every compact bag sets to
`(0, 0, 1.0f, 1.0f)` (`Compact::kRow`). The mask word (`maskWord`) holds 2 bits a
lane - 0 data, 1 ROW - in the same pattern for all four write cycles:
`0x40404040` for a position, `0x50505050` for an ST.

**V3-16 or V4-16 positions** (`kPositionFormat`): V3-16 is 6 bytes a vertex,
V4-16 8 bytes with the fourth halfword unused. w comes from ROW either way, so
the VU programs are the same and the constant is the only switch.

## The run header

Every run follows one header quadword: `{STMASK, mask, STCYCL 1/1, UNPACK}` for
a masked attribute, `{NOP, NOP, STCYCL 1/1, UNPACK}` otherwise (`runHeader`).
That is the quadword a baked block already spends on two NOPs, the STCYCL and
the UNPACK ([baked-vif-stream.md](baked-vif-stream.md)), so the mask costs no
byte. The payload's padding words are zero: VIF1 reads the words after an
unpack's data as VIFcodes, and zero is a NOP. `Packet2TyraUtils::addCompactRun`
sends a header and its payload with one REF whose own VIFcode slots are NOPs.

**Every UNPACK word is built by hand** (`unpackCode`). The ps2sdk helpers always
set USN - bit 14 of the immediate, "unsigned" - and every signed format needs it
clear. Bit 15, FLG, is "add TOPS" (the double buffer); `num` 256 is written as
0; the address is 10 bits. The VIF-hash gate tells geometry from uniforms by FLG
(`vifUnpackIsGeometry` in `stapip_vif_hash.hpp`); it read bit 14 before this
format, see [baked-stream-acceptance-gate.md](baked-stream-acceptance-gate.md).

## Boxes, and the seam rule

A position is stored as an integer in its bag's **box**: one step for x, y and z
(isotropic, so a uniform scale never bends a normal) and an origin that is a
whole number of steps. The integer is

    q = round(v / step) - round(origin / step)

rounded half away from zero and saturated to +-32767 (-32768 is never written,
so the range is symmetric). It decodes to `origin + step * q`. A draw needs no
decode: the box rides in the matrix the VU multiplies by anyway, `MVP x D`, where
`D` is the box as a matrix (`dequant`: the step on the diagonal, the origin as
the translation, column-major like `M4x4`).

**World bags** - a bag whose model matrix is exactly the identity - share the
**scene step** (`sceneStepFor`): the smallest power of two, at least 2^-10 m, at
which the scene's largest world bag fits wherever it lies. A box spans 65 534
steps and a bag that does not start on the grid can need one more, so the step
holds 65 533: 2^-9 m (1.95 mm) holds 127.99 m, 2^-8 m 255.99 m. `worldBox` puts
the origin on the scene grid; a bag wider than 65 534 steps takes the next
coarser power of two that holds it, for itself only, and says so (`coarse`).

**The seam rule: a vertex two world bags share decodes to the same float in
both.** That is why the integer is a difference of grid indices. With a
power-of-two step, `v / step`, the rounding and `origin / step` are all exact,
so `q + origin / step` is the vertex's index on the scene grid whatever box it
is in, and the decode `step x index` is exact too. Subtracting the origin first,
`round((v - origin) / step)`, is not: `v - origin` rounds when `v` is small and
the origin far away, and a value a hair below half a step can round onto it.
2^-10 - 2^-34 in a box at -32 m (step 2^-9) reads one step up that way and 0 in
a box at 0 m - a 2 mm crack along the seam. Exact operations also make the
integers the same on every platform: the EE's FPU and VU0 round toward zero, a
host to nearest, and data the editor quantized agrees with data the console
quantized. Half away from
zero is computed as `trunc(2x) - trunc(x)`, never as `floor(x + 0.5)`, which
reads `0.5 - 2^-25` as 1 under round-to-nearest.

**Any other bag** uses a box of its own (`modelBox`), one per model - every part
and LOD tier of it: step = the largest extent / 65 534 (a 2 m body 0.0305 mm),
origin a whole number of steps near the box's centre. A vertex comes back within
half a step while the model lies within 2^24 steps of its own origin (about 256
times its size). That step is not a power of two, so a host and the EE can
disagree by one step on a rare vertex; one producer writes a model's bags.

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
attribute's format, sign, mask and sizes; every UNPACK, STMASK and STCYCL word
against the hardware's field layout; every run of every attribute, `n` = 1 to
255, walked through `vif1_chain_check`'s VIF1 model (the unpack ends where the
data ends and the padding reads as NOPs); scene steps and world boxes (origins
on the grid, coarse past 65 534 steps); 10 000 random seam edges whose vertices
decode bit-identical in both boxes - a quarter of them built where subtracting
first fails, which it then does; a 2 m body's model box; ST, its rebase, colour
and normals; and the exact paths again under every float rounding mode the host
can select. `Compact::selfTest()` holds known answers that are exact under any
rounding mode, so a game can run it on the console to show the EE agrees with
the host that wrote its data.
