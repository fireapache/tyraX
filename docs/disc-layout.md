# Disc layout

The Disc Layout window shows what goes on the PS2 disc and in which order,
how big the image is and whether it fits a CD-R or a DVD-5 - the plan *Project
> Export PS2 ISO* writes. Open it from *Project > Disc Layout...*; it opens as a
tab beside the viewport.

## Before the first build

The disc is planned from the files a build puts in `bin/` - the ELF, the baked
models, textures and audio - so before any build the window says so and offers
**Build &lt;project&gt;** (the same as *Build > Build*, Ctrl+Shift+B). The path it
looked for (`bin/<project>.elf`) is the footnote. After a build, or an ISO
export, the plan refreshes by itself; the header says how old it is ("from the
last build, 4 min ago") and **Refresh** re-reads `bin/` by hand.

## The files, by load stage

The left side lists every file in disc order, grouped by when the game reads
it: **boot** (read first, fixed: `SYSTEM.CNF` and the ELF), **startup** (HUD,
loading screens, sounds), one **scene:&lt;name&gt;** group per scene (the start
scene first), **music** (streamed) and **other** (not claimed by a group). Each
group header shows its colour, what it means, its file count and total size;
click it to fold it. Rows show the path, the size and the first sector (LBA).

The automatic order is the load order - what boot and the first scene read sits
together, so the laser seeks less. Drag a file to override it: the whole
visible order is then saved to `iso-layout.txt` in the project, files placed by
hand are marked **moved**, the header counts them ("2 moved by hand"), and
**Reset to automatic** deletes the override.

## Headless export

*Project > Export PS2 ISO* is the GUI entry point. The same writer is available
from the command line for scripts and CI:

```
tyrax-editor --export-iso <projectDir>
```

It loads the project, plans the disc exactly as the window previews it, prints
each warning and the final LBA layout to stdout, and writes
`<projectDir>/<name>.iso`. This needs no editor session and no Docker; it uses
the files already in `bin/`, so build first.

## PS2 limits are handled at export

A disc can be valid ISO9660 and still fail on a PS2 (or in PCSX2): cdvdman
scans only the first 2048 bytes of a directory extent and matches identifiers
by their first 16 characters. Those used to be the author's problem — split a
directory, rename colliding assets, update every reference. The exporter owns
both limits now, so content keeps whatever names and folders it needs:

- Every asset in `bin/` gets a unique 10-hex-digit name with its original
  extension, bucketed into two short directory levels (`D0/3/<hash>.PNG;1`).
  Names differ well inside the first 16 characters, and no bucket comes near
  one sector.
- The logical→physical table is written at the disc root as `PATHMAP.BIN`. The
  engine loads it once, right after the IOP is ready in `Engine::initAll`, and
  resolves every `cdrom0:` open in `FileUtils::fromCwd` (plus the three loaders
  that open already-prefixed paths). Project assets, bake output and generated
  code keep their authored names. The table is small, fixed-size and needs no
  heap.
- `SYSTEM.CNF`, the ELF and the boot markers that are opened before the map
  loads (`gym.run`, `ps2link.run`, `launch.scene`) keep their real names.
- With no map (a host or ps2link run) resolution is a no-op, so the same ELF
  plays from `bin/` and from an ISO.

`iso9660::validateForPs2` still runs, on the physical file list, as an
assertion before writing. A report from it now means the exporter's own layout
is wrong rather than that an author should rename something. The baked-name
hashes in the animated-model, vehicle and model-AO bakes predate this and are
harmless; new assets do not need them.

## The disc

On the right, the disc drawn the way data sits on it: a spiral from the hub
outward, so an outer turn holds more than an inner one and a big file is a band
of turns. Choose the capacity it is drawn against - **Fit to data**, **CD-R
700 MB** or **DVD-5**. Point at the disc (or select a file) and the card under
it says which file that is, its group, size and sectors. A legend names the
group colours. The footer states the image size and whether it fits ("0.3% of
a CD-R, 700 MB free", or in red "12 MB over a CD-R"), next to **Export ISO**.
