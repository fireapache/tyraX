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

## PS2 limits the export checks

A disc can be valid ISO9660 and still fail on a PS2 (or in PCSX2), so *Export
PS2 ISO* checks the plan before it writes and prints a `PS2:` line in Output for
each problem:

- **One directory sector.** The console's cdvdman only scans the first 2048
  bytes of a directory extent. A directory with more entries than fit there
  looks fine to a PC ISO reader but `fopen()` cannot see anything past the
  first sector. Split the assets into two shorter directories (and update the
  project's asset paths) until every directory fits one sector.
- **Sixteen identifier characters.** The console matches a name by its first
  16 characters only. Two files in the same directory sharing that prefix open
  the same file — a `.PNG` can resolve to a `.TSKL` and fail in libpng. Give
  colliding assets short, distinct leading names. The animated-model bake
  already prefixes its extracted textures with a hash for this reason
  ([animated-models.md](animated-models.md), "Build output").

The warnings do not stop the export: a disc with them still mounts on a PC and
is useful for inspection. They describe what the console will do, not a fault
in the image.

## The disc

On the right, the disc drawn the way data sits on it: a spiral from the hub
outward, so an outer turn holds more than an inner one and a big file is a band
of turns. Choose the capacity it is drawn against - **Fit to data**, **CD-R
700 MB** or **DVD-5**. Point at the disc (or select a file) and the card under
it says which file that is, its group, size and sectors. A legend names the
group colours. The footer states the image size and whether it fits ("0.3% of
a CD-R, 700 MB free", or in red "12 MB over a CD-R"), next to **Export ISO**.
