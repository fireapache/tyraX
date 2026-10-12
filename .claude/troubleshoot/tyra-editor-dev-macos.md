# tyra-editor-dev — macOS: the app, signing, the native toolchain

From upstream's macOS additions to `build-platform-release.md` (1.172.x). See
`docs/updates.md#macos`, `docs/native-toolchain.md`. The cross-platform half
(packaging pair, `stage-tree.sh`, the `.tyrax-package` marker) is in
`tyra-editor-dev-build.md`.

## A PS2DEV binary dies with `Killed: 9` after the macOS relink
**Cause:** `tools/toolchain/setup.sh` (part of the `.ps2dev-base` install) copies
the libs to `<toolchain>/lib/tyrax-host`, `machorelink` rewrites absolute
`.../opt/<lib>` refs to `@loader_path/<lib>` in place, the libs are HARD-LINKED
beside each binary, and each is re-signed `codesign -s -`. The archive has
hard-linked twins (`<triple>-as` / `<triple>/bin/as`); codesign writes a new file,
so the twin kept the patched bytes + a dead signature.
**Fix:** machorelink lists every name that references the libs, already-patched
ones included.

## The app builds games on the build Mac but not on a clean one
**Cause:** the app must build games with NO Homebrew and NO Command Line Tools.
`/usr/bin/make`, `otool`, `install_name_tool` (also git, clang) are xcode-select
SHIMS (`md5` equals `/usr/bin/git`'s); rsync, curl, tar, shasum, codesign, ditto,
hdiutil, sips, iconutil, tiffutil are real.
**Fix:** ship `tools/macos/` from `installer/macos/build-host-tools.sh` (pinned +
hashed sources, built for macOS 15 = PS2DEV's minimum): gmake,
gmp/mpfr/mpc(**1.4** - PS2DEV wants compat 8)/zstd, prebuilt openvcl/vclpp/bin2s,
`machorelink`. Homebrew bottles target the build Mac's OS and must never be copied.
Verify with the isolated-env build in `tyra-testing-packages.md`.

## codesign fails with "timestamps differ by N seconds"
**Cause:** a keychain-access prompt left open too long while `build-macos.sh`
signs (hardened runtime + `--timestamp`).
**Fix:** rerun. Signing goes inside out: every Mach-O found by `file`, then the
bundle, then the dmg.

## A macOS release job fails on `stapler staple` (Error 68)
**Cause:** `stapler staple` (CloudKit, separate from the notarytool verdict) can
fail for minutes right after an accepted submission (both 1.172.1 Intel runs, on
the dmg).
**Fix:** `notarize()` retries it 6x (~5 min), never resubmits, and lets only the
DMG ship unstapled (notarized; the app inside is stapled). release.yml's publish
step uploads `--clobber` into an existing release, so `gh run rerun <id> --failed`
on one macOS job finishes that release.

## A new file under `tools/toolchain/` never reaches the release
**Cause:** `tools/toolchain/*` is an ALLOW-LIST in .gitignore — a new file there is
invisible to git until listed (1.172.0 shipped without a dmg because
`machorelink.c` was not).
**Fix:** add the file to the allow-list in the same commit, then check
`git status` shows it.

## A `.sh` script works on Linux and breaks on macOS
**Cause:** macOS runs bash 3.2 and BSD userland.
**Fix:** keep the `.sh` scripts bash 3.2 + BSD clean: no `mapfile`, no `sed -i`,
no `sha256sum` (use the `sha256` helper), no `sed \+`, no `install -D`, and no
`grep -q` at the end of a pipe under `pipefail` (it SIGPIPEs the writer and fails
the pipeline). When testing an rsync, write `echo x > f`, not `echo x>f` (that
once "proved" openrsync broken).

## Reference facts — the app and the release
- `installer/build-macos.sh` (docs/updates.md#macos): code signing forbids anything
  but the binary in `Contents/MacOS` and arbitrary dirs in `Contents/`, hence
  `Resources` + `platform::resourceBinDir` (`Contents/Resources/bin`). Notarize via
  `--notary-profile` or `APPLE_API_KEY_*`; release.yml uploads a dmg only when
  signed AND notarized (secrets: docs/updates.md#releasing-for-macos).
- Window art and icon are committed PNGs from `installer/macos/make-*.py` (Pillow,
  `build-macos/py`), geometry in `dmg_layout.py`; the layout is written by
  `dmgbuild` (no Finder scripting).

## Reference facts — building on macOS
- Xcode Command Line Tools (clang) + Homebrew (`SYSTEM_PACKAGES_brew`, no root:
  `tyrax_root_prefix` returns empty on Darwin). build.sh skips the pkg-config/X11
  and zenity checks there and checks `c++`/`xcode-select` instead of
  `g++`/`pkg-config`. GLFW builds its Cocoa backend; CMake needs no Apple branch
  (miniaudio loads CoreAudio at runtime).
- Native PS2 toolchain: `tools/toolchain/setup.sh` picks the PS2DEV asset by
  `uname -s`-`uname -m` (sha per asset; the local archive name keeps the old
  `ubuntu-x86_64` spelling so Linux caches stay valid). The macOS archive's gcc
  loads Homebrew's gmp/mpfr/libmpc/zstd (the app ships its own, above). The system
  make 3.81 and rsync (openrsync since 15.4; the samba 2.6.9 of 15.0-15.3 is
  untested) build the engine and games fine — verified with a full libtyra build.

## Reference facts — `platform.cpp` on Darwin
- `exePath` via `_NSGetExecutablePath`; `processesNamed` via libproc
  (`proc_listallpids`/`proc_pidpath`) + `KERN_PROCARGS2` for the command line.
- Dialogs, alerts and the image clipboard via `osascript` (`choose file ... of type {ext}`,
  `display alert`, `«class PNGf»`); `openUrl`/`revealInFileManager` via
  `open` / `open -R`; `installDesktopEntry` is a no-op.
- Fonts from `/System/Library/Fonts` (+ `/Library/Fonts`, `~/Library/Fonts`), UI
  font Helvetica Neue (`.ttc`, face 0).
- A static initializer APPENDS `/usr/local/bin:/opt/homebrew/bin` to `PATH` (an
  app started from Finder gets launchd's bare PATH, so `code`/`docker`/a Homebrew
  PCSX2 vanished).
- `configDir` stays `~/.config/tyra-editor` (a space in `Application Support`
  would break the toolchain makefiles).
- `app.cpp`: the GL context asks for `GLFW_OPENGL_FORWARD_COMPAT` on Cocoa (no 3.3
  core context otherwise) and skips `glfwSetWindowIcon`; the GPU GI compute shader
  (GL 4.3) is unavailable since macOS caps at 4.1.
- HiDPI: ImGui's GLFW backend reports content scale 1 on macOS and renders at
  framebuffer scale, so `applyUiScale` needs nothing extra.
- PCSX2 is found as `PCSX2.app` (`runner.cpp`), its ini under
  `~/Library/Application Support/PCSX2` (`pcsx2_config.cpp`).
