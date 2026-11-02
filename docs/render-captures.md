# Render captures: a picture of every example, and a comparer

`tools/render-captures.py` builds every project under `examples/`, boots each one in a private PCSX2 with the software GS renderer, photographs its framebuffer at exact logic frames, and writes the pictures with a `manifest.json`. A second command, `compare`, lines two such captures up shot by shot and says which pictures are identical, which are near and which a person has to look at. Capture before an engine change that touches rendering, capture again after it, compare: a vertex format, a new cull program or a changed batcher shows up as pictures, not only as numbers.

It writes nothing in your TyraX checkout (the examples are copied, built and baked in a scratch folder), starts its own editor configuration so its engine cache does not race a build you are running, and only reads your PCSX2.ini.

## Capturing

```
python tools/render-captures.py capture --out captures/before --pcsx2 <path to pcsx2-qt>
python tools/render-captures.py capture --out captures/before --pcsx2 <...> --only cube,lighting
```

What it needs: Windows with WSL and the TyraX toolchain (the editor provisions it), a built `build/tyrax-editor`, Python 3.8 or newer with Pillow and numpy, a BIOS in your PCSX2.ini, and a PCSX2 that understands `-datapath` (2.8.2 or newer; a portable copy is fine, `PCSX2_EXE` replaces `--pcsx2`). The installed PCSX2 2.0.x cannot run private and is refused.

Per example the tool:

1. copies the files git would hold for it into `<work>/p/<example>` (`--work`, default `%TEMP%/txb`) and builds that copy with `tyrax-editor --build <copy> --debug`, because only a debug build carries the [Live Debugger](live-debugger.md) that answers the capture. An example with `"liveDebug": false` gets it switched on in the copy, and an example that places procedural objects "fresh per run" is pinned to its authored seed in the copy (the engine seeds `rand()` from the clock), so the pictures are the same project every time;
2. boots the ELF in a private PCSX2 (software GS, unthrottled, MTVU off, vsync off, audio muted, window hidden) and writes the step to the Live Debugger's command file before the emulator starts. The game applies it at its first poll (logic frame 20), runs that many frames, halts, and the tool then asks the game to photograph itself, so nothing the host does lands inside the game's timeline. `f0100.png` is the picture of logic frame 120: the target frame plus the 20 frames before the first poll. `--frames 100,400,1000` is the default; `--continuous` takes one boot per example instead, which is faster but lets the host's reaction time leak into time-driven scenes.

The second pass (`--repeat 2`, the default) boots every example again a few minutes after the first and compares the two pictures of each frame. An example whose shots are identical twice is marked `"deterministic": true`; the others are not, and the second pictures are kept beside the first as `f0400.r2.png` with an eight-times diff `f0400.r2-diff.png`. Wait at least a clock minute between the passes (the tool does): the engine's `rand()` is seeded from the emulated clock, which ticks in minutes, so two boots seconds apart can look the same and still depend on it.

A full pass over the examples takes about 20 minutes once the scratch folder is warm; the first one also builds all of them. `--resume` skips examples already in `results.json`, `--redo a,b` does those again, `--clean-work` or `clean` removes the scratch folder (and the WSL build mirrors the native backend keeps for it) without touching the real toolchain.

### What it writes

```
DIR/manifest.json
DIR/<example>/<shot>.png        f0100.png, f0400.png, f1000.png
DIR/<example>/<shot>.r2.png     only when the second boot differed
DIR/index.md, index.html        the table and a contact sheet
DIR/results.json, run.json      everything the run measured, and with what
```

`manifest.json` is the interface other tools use:

```
{ "<example>": { "status": "ok" | "crash" | "no-boot" | "timeout",
                 "deterministic": true | false,
                 "shots": [ { "name": "f0100", "frame": 120, "sha256": "..." } ] } }
```

`status` is what happened to the example: `ok`, `crash` (the game asserted, wrote `crash.txt` or the emulator died), `no-boot` (it did not build or never answered the Live Debugger) or `timeout` (a hang, or a build over `--build-timeout`). `frame` is the logic frame the picture shows, as the game reported it. `sha256` is the hash of the decoded RGB pixels with the size in front, so the same picture hashes the same whichever encoder wrote the PNG. A folder that is not a project (no `.tyra`) is left out. Any other capture tool that writes this layout can use `compare`.

## Comparing

```
python tools/render-captures.py compare captures/before captures/after --out captures/diff
```

Every shot of BASE (the first folder) gets one verdict:

| Verdict | Meaning |
|---|---|
| `IDENTICAL` | the same pixels |
| `NEAR` | some pixels differ, none by more than 4 levels in any channel |
| `REVIEW` | anything else, or a different picture size; writes `<example>/<shot>-diff.png` (red, eight times the difference) and `<example>/<shot>-side.png` (BASE left, NEW right) under `--out` |
| `MISSING` | NEW has no such picture (the example is gone, or crashed before the frame) |

The last line is `SUMMARY identical=<n> near=<n> review=<n> missing=<n>`. The exit code is 1 only when a shot is `MISSING` or an example's status got worse (`ok` to `crash`, `ok` to `timeout`, and so on); a `REVIEW` shot is for a person to open, and the tool does not decide whether a difference is a bug. A shot of an example that BASE marked not deterministic is flagged `[base not deterministic]`: a difference there may be the example, not the change. `compare.md` in `--out` keeps the table.

Read a `REVIEW` side by side before calling it a regression: a crack between tiles, a texture that swims, banding, a missing mesh or a new crash is one; a moved particle in a scene that was already not deterministic is not.

## Other commands

- `report --out DIR` rewrites `manifest.json`, `index.md` and `index.html` from `results.json` and an optional `review.json` (`{"cube": {"render": "OK", "note": "..."}}`, where a person's verdict on the pictures goes).
- `sheet --out DIR [--frame 400]` writes PNG contact sheets, a grid with one frame per example, for a quick look.
- `list` shows the examples this checkout has; `clean` removes the scratch folder.

## Limits

The pictures are PCSX2's software renderer, not a real PS2 or the hardware renderer, and they are the game's own read-back at a halted frame, so a game that needs a button press shows only what it draws without input (a menu, a title). The tool has no opinion about what a correct picture looks like: it compares with an earlier capture, so the first capture of a checkout still has to be looked at.

The tests are `tools/test_render_captures.py` (`python -m pytest tools/test_render_captures.py`, or `python -m unittest discover -s tools -p "test_render_captures.py"`): synthetic pictures through every verdict and exit code, without an emulator.
