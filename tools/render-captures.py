#!/usr/bin/env python3
r"""render-captures.py - a repeatable "state of rendering" capture of every
TyraX example project, run in PCSX2's software GS renderer, and a comparer for
two such captures. See docs/render-captures.md.

    python tools/render-captures.py capture --out DIR [--only a,b]  # build + boot + photograph
    python tools/render-captures.py compare BASE NEW --out DIR      # BASE vs NEW, shot by shot
    python tools/render-captures.py report  --out DIR               # rewrite index.md / index.html
    python tools/render-captures.py sheet   --out DIR               # PNG contact sheets, to look at
    python tools/render-captures.py list                            # examples this checkout has
    python tools/render-captures.py clean                           # delete the scratch root + its WSL mirrors

`capture` writes DIR/<example>/<shot>.png and DIR/manifest.json:

    {example: {status: ok|crash|no-boot|timeout, deterministic: bool,
               shots: [{name, frame, sha256}]}}

`compare` classifies every shot of BASE as IDENTICAL (the same pixels), NEAR (every
differing pixel within 4 levels in every channel), REVIEW (anything else: writes
<shot>-diff.png and <shot>-side.png under --out) or MISSING (NEW has no such
shot), prints `SUMMARY identical=<n> near=<n> review=<n> missing=<n>` and exits 1
only on a MISSING shot or an example whose status got worse. The same manifest
layout is what any other project's capture tool can write to reuse `compare`.

What `capture` does, per example (examples/<name>, default: every one with a .tyra):

  1. Copies the example's git files (tracked + untracked-not-ignored, i.e. what a
     clean checkout holds) into <work>/p/<name>. NOTHING in the TyraX checkout is
     ever written: builds, bakes and generated sources all land in the copy.
  2. Builds it with `tyrax-editor --build <copy> --debug` (native/WSL backend).
     --debug because only a debug build carries the Live Debugger that
     answers the capture; the stored profile of release examples is overridden
     for the copy, and an example with "liveDebug": false gets it switched on in
     the copy's .tyra. The editor runs with a PRIVATE config dir (LOCALAPPDATA ->
     <work>/cfg), so its engine cache never races another build on the machine
     (the shared one belongs to whoever builds a game); the toolchain is a
     junction to the real one.
  3. Boots the ELF in a private PCSX2 (-datapath, software GS = Renderer 13,
     unthrottled, MTVU off, vsync off, audio muted, GS window hidden) and drives
     the game's Live Debugger over its own command channel (bin/livedbg.cmd).
     Default = ISOLATED boots: one boot per target frame, with `steps=N` already
     written into bin/livedbg.cmd BEFORE the emulator starts. The guest applies it
     at its first command poll (logic frame 20), runs N frames, halts, and the
     host then asks for a capture-frame: nothing the host does (reaction time, a
     capture's file writes) happens inside the guest timeline before the picture.
     So every screenshot is the GS framebuffer of an EXACT guest logic frame
     (fNNNN.png = target NNN = logic frame NNN+20), never "whatever frame the wall
     clock reached". --continuous = one boot, halt, step, capture, step ...:
     faster, but the host's halt latency leaks into the guest clock and time-driven
     scenes drift between runs. The picture is the game's own
     (`tyrax-editor --capture-frame`'s mechanism: GS VRAM read-back by the game),
     so no window, focus or desktop is involved.
  4. Repeats step 3 as a second PASS over every example (--repeat, default 2),
     minutes after the first, and pixel-compares the two: the per-example
     determinism verdict. (The engine seeds rand() from the guest RTC, which has
     minute granularity: a re-boot seconds later would hide that dependence.)

Output (--out DIR): manifest.json (above), <example>/f<NNNN>.png (+ f<NNNN>.r2.png
and f<NNNN>.r2-diff.png when the second boot differed), build.log (filtered),
game-log.txt, result.json; results.json (everything the run measured), run.json,
index.md and index.html (contact sheet, relative image paths). `review.json` in DIR
({"cube": {"render": "OK", "note": "..."}}) is merged into the report - it is where a
human verdict on the pictures goes.

Repeat after an engine branch with the same arguments and `compare` the folders.
Prerequisites: Windows, WSL + the TyraX toolchain (the editor provisions it),
PCSX2 >= 2.8.2 (has -datapath; --pcsx2 or $PCSX2_EXE), a BIOS in your PCSX2.ini,
Python 3.8+ with Pillow and numpy. Your own PCSX2.ini is only READ.
"""
import argparse
import datetime
import hashlib
import html
import json
import os
import pathlib
import queue
import re
import shutil
import struct
import subprocess
import sys
import threading
import time

try:
    import numpy as np
    from PIL import Image
except ImportError:
    sys.exit("render-captures: needs Pillow and numpy (pip install pillow numpy)")

IS_WIN = os.name == "nt"
HERE = pathlib.Path(__file__).resolve().parent

# ----------------------------------------------------------------- defaults
DEFAULT_TYRAX = os.environ.get("TYRAX_DIR") or str(HERE.parent)
DEFAULT_PCSX2 = os.environ.get("PCSX2_EXE") or ""
DEFAULT_FRAMES = "100,400,1000"
SKIP_ALWAYS = {
    "character-generator": "not a project (no .tyra - only a stray docker-compose.yml and .vscode/)",
}
# Examples that need a different plan; reason shown in the report. frames= overrides
# --frames for that example only.
PER_EXAMPLE = {}

# Live Debugger wire constants (src/livedbg.cpp). Re-read from the checkout at start-up.
CMD_MAGIC, CMD_VERSION, FOOTER_XOR = 0x43445854, 2, 0x5A5A5A5A
SNAP_MAGIC, SNAP_VERSIONS = 0x42445854, (3, 4, 5)

STALE_CHANNEL_FILES = ("livedbg.cmd", "livedbg.bin", "livedbg.cmd.tmp", "livepad.bin", "frame.tga",
                       "log.txt", "ps2link.run", "crash.txt", "vucap.bin", "livelink.bin",
                       "livelink.sig", "livelogic.bin", "livetex.bin", "livetime.bin", "livetime.rst",
                       "replay.in", "replay.arm", "replay.out", "replay.stop", "replay.st",
                       "rendercost.txt")

_print_lock = threading.Lock()


def log(msg):
    with _print_lock:
        print("%s  %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


# ------------------------------------------------------------ small helpers
def rm_tree(path):
    """rmtree that survives Windows handles that linger a moment after a process dies."""
    path = pathlib.Path(path)
    for attempt in range(6):
        shutil.rmtree(path, ignore_errors=True)
        if not path.exists():
            return True
        time.sleep(0.5)
    return not path.exists()


def kill_tree(pid):
    """Kill ONE process tree by PID (never by image name)."""
    if IS_WIN:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)], capture_output=True)
    else:
        try:
            os.kill(pid, 9)
        except OSError:
            pass


def sha1_file(p):
    return hashlib.sha1(pathlib.Path(p).read_bytes()).hexdigest()


def read_text(p):
    try:
        return pathlib.Path(p).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def git(tyrax, *args):
    r = subprocess.run(["git", "-C", str(tyrax)] + list(args), capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else ""


def load_wire_constants(tyrax):
    """The command/snapshot constants live in the editor source; read them so a
    branch that bumps a version is noticed instead of silently ignored."""
    global CMD_MAGIC, CMD_VERSION, FOOTER_XOR, SNAP_MAGIC, SNAP_VERSIONS
    text = read_text(pathlib.Path(tyrax) / "src" / "livedbg.cpp")
    def grab(name):
        m = re.search(r"constexpr\s+uint32_t\s+%s\s*=\s*(0x[0-9A-Fa-f]+|\d+)U?" % name, text)
        return int(m.group(1), 0) if m else None
    for name, var in (("kCmdMagic", "CMD_MAGIC"), ("kCmdVersion", "CMD_VERSION"),
                      ("kFooterXor", "FOOTER_XOR"), ("kSnapMagic", "SNAP_MAGIC")):
        v = grab(name)
        if v is not None:
            globals()[var] = v
    v = grab("kSnapVersion")
    if v is not None and v not in SNAP_VERSIONS:
        SNAP_VERSIONS = SNAP_VERSIONS + (v,)


# ------------------------------------------------------------ example list
def list_examples(tyrax):
    out = []
    ex_dir = pathlib.Path(tyrax) / "examples"
    for d in sorted(ex_dir.iterdir()):
        if not d.is_dir():
            continue
        tyras = sorted(d.glob("*.tyra"))
        out.append((d.name, tyras[0] if tyras else None))
    return out


# ------------------------------------------------------------ project copy
def transform_source(rel, data, notes):
    """The only edits a capture makes to an example, applied to the COPY:
      * "liveDebug": false -> true  (the capture is answered by the Live Debugger)
      * "seedMode": 1 -> 0          (procedural placement "fresh per run" re-seeds
                                     from rand(), itself seeded from the wall clock
                                     (engine.cpp srand(time)): a different picture
                                     every minute. The authored seed pins it.)"""
    if rel.endswith(".tyra"):
        t = data.decode("utf-8")
        n = re.sub(r'("liveDebug"\s*:\s*)false', r"\g<1>true", t, count=1)
        if n != t:
            notes.add("liveDebug was off: switched on in the copy")
        return n.encode("utf-8")
    if rel.endswith(".json") and rel.startswith("objects/"):
        t = data.decode("utf-8")
        n = re.sub(r'("seedMode"\s*:\s*)1\b', r"\g<1>0", t)
        if n != t:
            notes.add("procedural seedMode=fresh pinned to the authored seed in the copy")
        return n.encode("utf-8")
    return data


def sync_project(tyrax, name, dst, clean):
    """Copy the example's git files (tracked + untracked-not-ignored) into dst,
    applying transform_source. A manifest of the sources' hashes makes a re-sync
    touch only what CHANGED IN GIT: the editor rewrites generated files in the
    copy on every build, and putting the committed version back each time would
    bump their mtimes and recompile the game for nothing. A changed/new file gets
    mtime = now (a copied old mtime makes make keep a stale object)."""
    tyrax = pathlib.Path(tyrax)
    dst = pathlib.Path(dst)
    if clean and dst.exists():
        rm_tree(dst)
    dst.mkdir(parents=True, exist_ok=True)
    man_path = dst / ".capture-sync.json"
    try:
        manifest = json.loads(man_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        manifest = {}
    out = subprocess.run(["git", "-C", str(tyrax), "ls-files", "-z", "-co", "--exclude-standard",
                          "--", "examples/%s" % name], capture_output=True)
    files = [f for f in out.stdout.decode("utf-8", "surrogateescape").split("\x00") if f]
    prefix = "examples/%s/" % name
    n_new, notes = 0, set()
    for f in files:
        rel = f[len(prefix):] if f.startswith(prefix) else f
        src = tyrax / f
        if not src.is_file():
            continue
        data = transform_source(rel, src.read_bytes(), notes)
        h = hashlib.sha1(data).hexdigest()
        d = dst / rel
        if manifest.get(rel) == h and d.exists():
            continue
        d.parent.mkdir(parents=True, exist_ok=True)
        d.write_bytes(data)
        manifest[rel] = h
        n_new += 1
    man_path.write_text(json.dumps(manifest), encoding="utf-8")
    return len(files), n_new, sorted(notes)


def makefile_target(proj):
    m = re.search(r"^TARGET\s*:?=\s*(\S+)", read_text(proj / "Makefile"), re.M)
    return m.group(1) if m else None


# ------------------------------------------------------------ editor config
def ensure_private_config(work):
    """<work>/cfg/tyra-editor with the real editor.ini and a junction to the real
    toolchain: the editor's native-build cache is then ours alone."""
    cfg = pathlib.Path(work) / "cfg"
    ed = cfg / "tyra-editor"
    ed.mkdir(parents=True, exist_ok=True)
    real = pathlib.Path(os.environ.get("LOCALAPPDATA", "")) / "tyra-editor"
    ini = ed / "editor.ini"
    if not ini.exists():
        if (real / "editor.ini").is_file():
            shutil.copyfile(real / "editor.ini", ini)
        else:
            ini.write_text("buildBackend=native\n")
    # The shared config must build natively whatever the user's last choice was.
    t = read_text(ini)
    if re.search(r"^buildBackend=", t, re.M):
        t = re.sub(r"^buildBackend=.*$", "buildBackend=native", t, flags=re.M)
    else:
        t += "\nbuildBackend=native\n"
    ini.write_text(t)
    tc = ed / "toolchain"
    if not tc.exists() and (real / "toolchain").is_dir():
        if IS_WIN:
            subprocess.run(["cmd", "/c", "mklink", "/J", str(tc), str(real / "toolchain")],
                           capture_output=True)
        else:
            tc.symlink_to(real / "toolchain")
    return cfg


# ------------------------------------------------------------ building
def filter_build_log(text):
    """Compile/link command lines are huge and carry no news; keep everything else."""
    keep, dropped = [], 0
    for line in text.splitlines():
        if "-fdebug-prefix-map" in line or len(line) > 600 and " -c -o " in line:
            dropped += 1
            continue
        keep.append(line[:300])
    keep.append("[render-captures] %d compiler/linker command lines omitted" % dropped)
    return "\n".join(keep) + "\n"


def build_example(ctx, name, tyra_file, proj, outdir):
    """Returns the build dict. Never raises for build problems."""
    res = {"status": "failed", "seconds": 0.0, "exit": None, "notes": []}
    t0 = time.time()
    try:
        total, changed, notes = sync_project(ctx.tyrax, name, proj, ctx.clean)
        res["files"] = total
        res["files_updated"] = changed
        res["notes"] += notes
        if not (proj / tyra_file.name).exists():
            res["notes"].append("no .tyra in the copy")
            return res
        target = makefile_target(proj) or (name + ".elf")
        elf = proj / "bin" / target
        res["elf"] = str(elf)
        if ctx.no_build and elf.exists():
            res["status"] = "ok"
            res["notes"].append("build reused (--no-build)")
            return res
        env = dict(os.environ)
        env["LOCALAPPDATA"] = str(ctx.cfg)
        cmd = [str(ctx.editor), "--build", str(proj), "--debug"]
        if ctx.rebuild:
            cmd.append("--rebuild")
        raw_log = outdir / "build.raw.log"
        attempts = 0
        while True:
            attempts += 1
            started = time.time()
            with open(raw_log, "wb") as fh:
                p = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, env=env,
                                     cwd=str(proj))
                try:
                    p.wait(ctx.build_timeout)
                except subprocess.TimeoutExpired:
                    kill_tree(p.pid)
                    res["status"] = "timeout"
                    res["notes"].append("build exceeded %ds" % ctx.build_timeout)
                    break
            res["exit"] = p.returncode
            text = read_text(raw_log)
            if p.returncode != 0 and attempts == 1 and re.search(r"migrat", text, re.I) \
                    and "pending" in text.lower():
                res["notes"].append("project needed --migrate (done in the copy)")
                subprocess.run([str(ctx.editor), "--migrate", str(proj)], capture_output=True, env=env)
                continue
            if p.returncode == 0 and elf.exists() and elf.stat().st_mtime >= started - 2:
                res["status"] = "ok"
            elif p.returncode == 0 and elf.exists():
                res["status"] = "ok"
                res["notes"].append("ELF not rewritten by this build (up to date)")
            else:
                res["status"] = "failed"
            break
        text = read_text(raw_log)
        m = re.search(r'-Cache "[^"]*[\\/]native-build[\\/]([^"\\/]+)"', text)
        if m:
            vf = ctx.work / ".engine-volume"
            if m.group(1) not in read_text(vf).split():
                with open(vf, "a", encoding="utf-8") as fh:
                    fh.write(m.group(1) + "\n")
        (outdir / "build.log").write_text(filter_build_log(text), encoding="utf-8")
        try:
            raw_log.unlink()
        except OSError:
            pass
        m = re.findall(r"^.*(?:error:|Error \d|FAILED|failed).*$", text, re.M)
        if res["status"] != "ok":
            res["error_lines"] = [l[:240] for l in m[:8]]
    except Exception as e:  # noqa: BLE001 - the harness must keep going
        res["status"] = "failed"
        res["notes"].append("harness error: %r" % (e,))
    res["seconds"] = round(time.time() - t0, 1)
    return res


# ------------------------------------------------------------ PCSX2 profile
def user_documents():
    if IS_WIN:
        import ctypes
        import ctypes.wintypes
        buf = ctypes.create_unicode_buffer(ctypes.wintypes.MAX_PATH)
        ctypes.windll.shell32.SHGetFolderPathW(None, 5, None, 0, buf)
        return pathlib.Path(buf.value)
    return pathlib.Path.home() / "Documents"


def find_user_ini(exe):
    cands = [pathlib.Path(exe).parent / "inis" / "PCSX2.ini", user_documents() / "PCSX2" / "inis" / "PCSX2.ini"]
    for c in cands:
        if c.is_file():
            return c
    sys.exit("render-captures: no PCSX2.ini to take the BIOS setting from (looked in %s)"
             % ", ".join(map(str, cands)))


def ini_set(text, section, key, value):
    lines = text.split("\n")
    sec, start, end = None, None, len(lines)
    for i, l in enumerate(lines):
        m = re.match(r"^\[(.+)\]\s*$", l.rstrip("\r"))
        if m:
            if sec == section:
                end = i
                break
            sec = m.group(1)
            if sec == section:
                start = i
    if start is None:
        return text.rstrip("\n") + "\n\n[%s]\n%s = %s\n" % (section, key, value)
    for i in range(start + 1, end):
        if re.match(r"^%s\s*=" % re.escape(key), lines[i], re.I):
            lines[i] = "%s = %s" % (key, value)
            return "\n".join(lines)
    lines.insert(end if lines[end - 1].strip() else end - 1, "%s = %s" % (key, value))
    return "\n".join(lines)


def ini_del(text, section, key):
    lines = text.split("\n")
    sec, start, end = None, None, len(lines)
    for i, l in enumerate(lines):
        m = re.match(r"^\[(.+)\]\s*$", l.rstrip("\r"))
        if m:
            if sec == section:
                end = i
                break
            sec = m.group(1)
            if sec == section:
                start = i
    if start is None:
        return text
    kept = [l for l in lines[start + 1:end] if not re.match(r"^%s\s*=" % re.escape(key), l, re.I)]
    return "\n".join(lines[:start + 1] + kept + lines[end:])


def ini_get(text, section, key):
    sec = None
    for l in text.split("\n"):
        m = re.match(r"^\[(.+)\]\s*$", l.rstrip("\r"))
        if m:
            sec = m.group(1)
        elif sec == section:
            m = re.match(r"^%s\s*=\s*(.*?)\s*$" % re.escape(key), l, re.I)
            if m:
                return m.group(1)
    return None


def strip_sections(text, sections):
    out, skip = [], False
    for line in text.split("\n"):
        m = re.match(r"^\[(.+)\]\s*$", line.rstrip("\r"))
        if m:
            skip = m.group(1) in sections
        if not skip:
            out.append(line)
    return "\n".join(out)


FOLDER_KEYS = {"Bios": "bios", "Snapshots": "snaps", "SaveStates": "sstates", "MemoryCards": "memcards",
               "Logs": "logs", "Cheats": "cheats", "Patches": "patches", "UserResources": "resources",
               "Cache": "cache", "Textures": "textures", "InputProfiles": "inputprofiles",
               "Videos": "videos", "Covers": "covers"}
# Emulation-BEHAVIOUR sections of a user's ini that a repeatable run must not inherit.
HERMETIC_SECTIONS = ("EmuCore/Speedhacks", "EmuCore/CPU", "EmuCore/CPU/Recompiler", "EmuCore/Gamefixes",
                     "Framerate")


def prepare_datapath(dp, exe, mute=True):
    """Private PCSX2 data root: <dp>/PCSX2/{inis,bios,...}. Software GS, MTVU off,
    vsync off, no SDL, PINE off, HostFs on, every [Folders] key under the root."""
    dp = pathlib.Path(dp)
    user_ini = find_user_ini(exe)
    root = dp / "PCSX2"
    (root / "inis").mkdir(parents=True, exist_ok=True)
    text = user_ini.read_text(encoding="utf-8-sig")
    bios_dir = ini_get(text, "Folders", "Bios") or "bios"
    src_bios = pathlib.Path(bios_dir)
    if not src_bios.is_absolute():
        src_bios = user_ini.parent.parent / src_bios
    bios = ini_get(text, "Filenames", "BIOS")
    dst_bios = root / "bios"
    dst_bios.mkdir(exist_ok=True)
    stem = pathlib.Path(bios).stem if bios else None
    copied = 0
    for f in src_bios.iterdir() if src_bios.is_dir() else []:
        if f.is_file() and (stem is None or f.stem == stem) and not (dst_bios / f.name).exists():
            shutil.copy2(f, dst_bios / f.name)
            copied += 1
    if not any(dst_bios.iterdir()):
        sys.exit("render-captures: no BIOS found in %s (ini says %s)" % (src_bios, bios))
    text = strip_sections(text, HERMETIC_SECTIONS)
    for key, sub in FOLDER_KEYS.items():
        (root / sub).mkdir(exist_ok=True)
        text = ini_set(text, "Folders", key, str(root / sub))
    settings = [
        ("EmuCore", "EnablePINE", "false"), ("EmuCore", "HostFs", "true"),
        ("EmuCore", "EnableMTVU", "false"), ("EmuCore", "EnableCheats", "false"),
        ("EmuCore/GS", "Renderer", "13"),          # 13 = Software: a real GS raster, no GPU API
        ("EmuCore/GS", "VsyncEnable", "false"),
        ("InputSources", "SDL", "false"), ("Logging", "EnableVerbose", "false"),
        ("UI", "SetupWizardIncomplete", "false"), ("UI", "PauseOnFocusLoss", "false"),
        ("UI", "ConfirmShutdown", "false"),
    ]
    if mute:
        settings += [("SPU2/Output", "OutputMuted", "true"), ("SPU2/Output", "OutputVolume", "0"),
                     ("SPU2/Output", "FastForwardVolume", "0")]
    for sec, key, val in settings:
        text = ini_set(text, sec, key, val)
    for sec, key in (("GameList", "Paths"), ("GameList", "RecursivePaths"), ("EmuCore/GS", "Adapter")):
        text = ini_del(text, sec, key)
    (root / "inis" / "PCSX2.ini").write_bytes(text.replace("\r\n", "\n").encode("utf-8"))
    written = (root / "inis" / "PCSX2.ini").read_text(encoding="utf-8")
    leaks = [k for k in FOLDER_KEYS if not str(ini_get(written, "Folders", k) or "").startswith(str(root))]
    if leaks:
        sys.exit("render-captures: private ini would still write outside %s: %s" % (root, ", ".join(leaks)))
    return root


def assert_datapath_support(exe):
    r = subprocess.run([str(exe), "-help"], capture_output=True, text=True, timeout=30,
                       creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    if "-datapath" not in (r.stdout or "") + (r.stderr or ""):
        sys.exit("render-captures: %s has no -datapath (need PCSX2 >= 2.8.2; point --pcsx2 or "
                 "$PCSX2_EXE at a portable copy). PCSX2 2.0.x cannot run private." % exe)


def hide_windows_hook(pid):
    if not IS_WIN:
        return lambda: None
    try:
        import ctypes
        from ctypes import wintypes
        user32 = ctypes.WinDLL("user32", use_last_error=True)
        proc_t = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    except Exception:  # noqa: BLE001
        return lambda: None

    def hook():
        def cb(hwnd, _):
            wpid = wintypes.DWORD()
            user32.GetWindowThreadProcessId(hwnd, ctypes.byref(wpid))
            if wpid.value == pid and user32.IsWindowVisible(hwnd):
                user32.ShowWindow(hwnd, 0)
            return True
        user32.EnumWindows(proc_t(cb), 0)
    return hook


# ------------------------------------------------------------ Live Debugger channel
class Channel:
    """Writer of bin/livedbg.cmd, reader of bin/livedbg.bin (the game's side of
    src/livedbg.cpp). Commands carry the full desired state; `seq` must change."""

    def __init__(self, game_dir):
        self.dir = pathlib.Path(game_dir)
        self.seq = int(time.time()) & 0x7FFFFFFF

    def send(self, halt=False, steps=0, capture=False):
        self.seq += 1
        flags = (1 if halt else 0) | (64 if capture else 0)
        body = struct.pack("<8I", CMD_MAGIC, CMD_VERSION, self.seq, flags, steps & 0xFFFFFFFF, 0, 0, 0)
        body += struct.pack("<I", self.seq ^ FOOTER_XOR)
        tmp = self.dir / "livedbg.cmd.tmp"
        dst = self.dir / "livedbg.cmd"
        tmp.write_bytes(body)
        for _ in range(100):
            try:
                os.replace(tmp, dst)
                return self.seq
            except OSError:
                time.sleep(0.02)
        raise RuntimeError("cannot replace livedbg.cmd")

    def snapshot(self):
        """(seq, frame, halted) from the 64-byte snapshot header, or None."""
        try:
            with open(self.dir / "livedbg.bin", "rb") as fh:
                b = fh.read(64)
        except OSError:
            return None
        if len(b) < 64:
            return None
        magic, ver, seq, frame, scene, halted = struct.unpack_from("<IIIIiI", b, 0)
        if magic != SNAP_MAGIC or ver not in SNAP_VERSIONS:
            return None
        return seq, frame, bool(halted)


# ------------------------------------------------------------ images
def read_tga(path):
    b = pathlib.Path(path).read_bytes()
    if len(b) < 18 or b[2] != 2 or b[16] != 32:
        return None
    w = b[12] | (b[13] << 8)
    h = b[14] | (b[15] << 8)
    if w <= 0 or h <= 0 or len(b) < 18 + w * h * 4:
        return None
    arr = np.frombuffer(b, dtype=np.uint8, count=w * h * 4, offset=18).reshape(h, w, 4)[::-1]
    rgb = arr[:, :, [2, 1, 0]]
    return np.ascontiguousarray(rgb)


def tga_complete(path):
    try:
        b = pathlib.Path(path).read_bytes()
    except OSError:
        return False
    if len(b) < 18 or b[2] != 2 or b[16] != 32:
        return False
    w = b[12] | (b[13] << 8)
    h = b[14] | (b[15] << 8)
    return w > 0 and h > 0 and len(b) >= 18 + w * h * 4


def image_stats(rgb):
    flat = rgb.reshape(-1, 3)
    nonblack = float((flat.max(axis=1) > 8).mean() * 100.0)
    packed = (flat[:, 0].astype(np.uint32) << 16) | (flat[:, 1].astype(np.uint32) << 8) | flat[:, 2]
    uniq = int(np.unique(packed).size)
    return {"w": int(rgb.shape[1]), "h": int(rgb.shape[0]),
            "mean": round(float(rgb.mean()), 2), "std": round(float(rgb.std()), 2),
            "nonblack_pct": round(nonblack, 2), "unique_colors": uniq,
            "sha1": hashlib.sha1(rgb.tobytes()).hexdigest()}


def pixel_diff(a, b):
    """Pixels that differ + max/mean abs channel diff. Different sizes -> None."""
    if a.shape != b.shape:
        return None
    d = np.abs(a.astype(np.int16) - b.astype(np.int16))
    changed = d.max(axis=2) > 0
    n = int(changed.sum())
    return {"px": n, "pct": round(100.0 * n / changed.size, 4),
            "max": int(d.max()) if n else 0, "mean": round(float(d.mean()), 4)}, changed


def save_diff_png(a, b, path):
    d = np.abs(a.astype(np.int16) - b.astype(np.int16)).max(axis=2)
    img = np.clip(d * 8, 0, 255).astype(np.uint8)  # x8: a one-level difference must be visible
    rgb = np.zeros(a.shape, dtype=np.uint8)
    rgb[:, :, 0] = img
    rgb[:, :, 1] = (img > 0) * 40
    Image.fromarray(rgb, "RGB").save(path)


# ------------------------------------------------------------ one PCSX2 run
class RunCtx:
    pass


def classify_log(text):
    """Cheap verdicts from the guest log (bin/log.txt)."""
    if "======= TYRA" in text or "==== TYRA" in text or "TYRAX  =====" in text:
        return "assert"
    return None


def wait_until(pred, timeout, step=0.05, tick=None, alive=None):
    end = time.time() + timeout
    while time.time() < end:
        if tick:
            tick()
        r = pred()
        if r:
            return r
        if alive is not None and not alive():
            return pred()
        time.sleep(step)
    return pred()


def run_pcsx2(ctx, name, proj, elf_name, frames, outdir, run_no):
    """One run of one example. Default mode (continuous): ONE boot, halt, then step
    to each target frame in turn and photograph it. Isolated mode (--isolate): one
    boot PER target frame, with the step command already sitting in bin/livedbg.cmd
    before the emulator starts, so the guest applies it at its first command poll and
    nothing the host does (reaction time, a capture's file writes) happens inside the
    guest's timeline before the frame is photographed."""
    if ctx.isolate:
        merged = {"run": run_no, "end": "ok", "captures": [], "notes": [], "mode": "isolated",
                  "started": datetime.datetime.now().isoformat(timespec="seconds"), "seconds": 0.0}
        for t in frames:
            one = run_pcsx2_boot(ctx, name, proj, elf_name, [t], outdir, run_no, preload=t,
                                 log_suffix="f%04d." % t)
            merged["captures"] += one["captures"]
            merged["notes"] += one["notes"]
            merged["seconds"] = round(merged["seconds"] + one.get("seconds", 0.0), 1)
            for k in ("capture_log", "log_tail", "log_lines", "halt_frame"):
                if k in one:
                    merged.setdefault(k, one[k] if not isinstance(one[k], list) else [])
                    if isinstance(one[k], list):
                        merged[k] += one[k]
            if one["end"] != "ok":
                merged["end"] = one["end"]
                if not any(c.get("png") for c in merged["captures"]):
                    break  # the first boot already produced nothing: no point in two more
        return merged
    return run_pcsx2_boot(ctx, name, proj, elf_name, frames, outdir, run_no)


def run_pcsx2_boot(ctx, name, proj, elf_name, frames, outdir, run_no, preload=None, log_suffix=""):
    run = {"run": run_no, "end": "?", "captures": [], "notes": [],
           "started": datetime.datetime.now().isoformat(timespec="seconds")}
    t_start = time.time()
    stage_root = ctx.work / "run"
    game = stage_root / "g"
    rm_tree(stage_root)
    stage_root.mkdir(parents=True, exist_ok=True)
    src_bin = proj / "bin"
    shutil.copytree(src_bin, game, ignore=shutil.ignore_patterns(*STALE_CHANNEL_FILES, "*.sym"))
    elf = game / elf_name
    if not elf.exists():
        run["end"] = "no-elf"
        return run
    prepare_datapath(ctx.dp, ctx.pcsx2, mute=True)
    chan = Channel(game)
    if preload is not None:
        chan.send(halt=False, steps=preload)  # waiting for the guest's first command poll
    args = [str(ctx.pcsx2), "-batch", "-nogui"]
    if ctx.unlimited:
        args.append("-unlimited")
    args += ["-datapath", str(ctx.dp)]
    if ctx.emulog:
        args += ["-logfile", str(outdir / ("emulog.r%d.txt" % run_no))]
    args += ["-elf", str(elf)]
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ctx.pids.add(proc.pid)
    hide = hide_windows_hook(proc.pid)
    guest_log = game / "log.txt"
    alive = lambda: proc.poll() is None  # noqa: E731

    def step_wait(target):
        """Wait until the game is halted at frame >= target. None = it never got there
        (run["end"] and the note say why)."""
        last_prog, last_frame = time.time(), -1
        while True:
            hide()
            s = chan.snapshot()
            if s and s[2] and s[1] >= target:
                return s
            if s and s[1] > last_frame:
                last_frame, last_prog = s[1], time.time()
            if not alive():
                run["end"] = "emulator-exit(%s)" % proc.returncode
                run["notes"].append("emulator exited while stepping to frame %d" % target)
                return None
            if time.time() - last_prog > ctx.step_stall:
                run["end"] = classify_log(read_text(guest_log)) or "hang"
                run["notes"].append("no frame progress for %ds before frame %d (last seen %d)"
                                    % (ctx.step_stall, target, last_frame))
                return None
            time.sleep(0.02)

    try:
        hide()
        snap = wait_until(chan.snapshot, ctx.boot_timeout, 0.1, tick=hide, alive=alive)
        run["boot_seconds"] = round(time.time() - t_start, 1)
        if not snap:
            text = read_text(guest_log)
            if not alive():
                run["end"] = "emulator-exit(%s)" % proc.returncode
            elif classify_log(text):
                run["end"] = "assert"
            else:
                run["end"] = "no-boot"
            run["notes"].append("no Live Debugger snapshot within %ds" % ctx.boot_timeout)
            return run
        if preload is not None:
            s = step_wait(preload)       # the command written before boot runs the game to N
            if s is None:
                return run
            cur = s[1]
            run["halt_frame"] = cur
        else:
            # ---- halt, then step to each target frame and photograph it
            chan.send(halt=True)

            def halted():
                s = chan.snapshot()
                return s if s and s[2] else None
            s = wait_until(halted, 30, 0.02, tick=hide, alive=alive)
            if not s:
                run["end"] = "no-halt-ack"
                run["notes"].append("halt command not acknowledged in 30 s (game stuck, or livedbg "
                                    "command version mismatch: CMD_VERSION=%d)" % CMD_VERSION)
                return run
            cur = s[1]
            run["halt_frame"] = cur
        for target in frames:
            if preload is None and target > cur:
                chan.send(halt=False, steps=target - cur)
                s = step_wait(target)
                if s is None:
                    return run
                cur = s[1]
            actual = cur
            tga = game / "frame.tga"
            try:
                tga.unlink()
            except OSError:
                pass
            chan.send(halt=True, capture=True)
            ok = wait_until(lambda: tga_complete(tga), ctx.capture_timeout, 0.1, tick=hide, alive=alive)
            if ok:
                time.sleep(0.15)  # let the last host: write settle
                ok = tga_complete(tga)
            if not ok:
                run["notes"].append("frame %d: no complete frame.tga in %ds" % (target, ctx.capture_timeout))
                run["captures"].append({"target": target, "actual": actual, "png": None, "error": "no frame.tga"})
                continue
            rgb = read_tga(tga)
            if rgb is None:
                run["captures"].append({"target": target, "actual": actual, "png": None, "error": "bad tga"})
                continue
            name_png = "f%04d.png" % target if run_no == 1 else "f%04d.r%d.png" % (target, run_no)
            Image.fromarray(rgb, "RGB").save(outdir / name_png, optimize=False)
            cap = {"target": target, "actual": actual, "png": name_png}
            cap.update(image_stats(rgb))
            run["captures"].append(cap)
        run["end"] = "ok"
        return run
    finally:
        ctx.pids.discard(proc.pid)
        try:
            proc.kill()
            proc.wait(15)
        except Exception:  # noqa: BLE001
            kill_tree(proc.pid)
        run["seconds"] = round(time.time() - t_start, 1)
        text = read_text(guest_log)
        if text and (preload is None or frames[0] == ctx.log_frame):
            (outdir / ("game-log.txt" if run_no == 1 else "game-log.r%d.txt" % run_no)).write_text(
                text[-60000:], encoding="utf-8")
        crash = game / "crash.txt"
        if crash.exists():
            shutil.copyfile(crash, outdir / ("crash.r%d.txt" % run_no))
            run["notes"].append("crash.txt written by the game")
        m = re.findall(r"Frame capture: (\d+)x(\d+) psm (\d+) at frame (\d+)", text)
        if m:
            run["capture_log"] = ["%sx%s psm %s frame %s" % t for t in m]
        run["log_lines"] = len(text.splitlines())
        run["log_tail"] = [l[:200] for l in text.splitlines()[-12:]]
        rm_tree(stage_root)


def judge_render(captures):
    """Automatic render verdict from pixel statistics only. A human pass is merged
    from review.json afterwards."""
    good = [c for c in captures if c.get("png")]
    if not good:
        return "no-capture"
    blank = [c for c in good if c["std"] < 1.0 or c["nonblack_pct"] < 0.5]
    if len(blank) == len(good):
        c = good[0]
        return "blank (%s)" % ("black" if c["mean"] < 8 else "uniform colour")
    if blank:
        return "partly blank (%d of %d frames)" % (len(blank), len(good))
    return "ok (auto)"


# ------------------------------------------------------------ the run command
def parse_frames(s):
    return sorted({int(x) for x in s.split(",") if x.strip()})


def cmd_capture(a):
    tyrax = pathlib.Path(a.tyrax)
    editor = pathlib.Path(a.editor) if a.editor else tyrax / "build" / ("tyrax-editor.exe" if IS_WIN else "tyrax-editor")
    if not editor.is_file():
        sys.exit("render-captures: editor not found: %s" % editor)
    if not a.pcsx2:
        sys.exit("render-captures: no PCSX2 given (--pcsx2 <pcsx2-qt> or $PCSX2_EXE)")
    pcsx2 = pathlib.Path(a.pcsx2)
    if not pcsx2.is_file():
        sys.exit("render-captures: PCSX2 not found: %s (use --pcsx2)" % pcsx2)
    assert_datapath_support(pcsx2)
    load_wire_constants(tyrax)

    work = pathlib.Path(a.work or (pathlib.Path(os.environ.get("TEMP", "/tmp")) / "txb"))
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    work.mkdir(parents=True, exist_ok=True)

    ctx = RunCtx()
    ctx.tyrax, ctx.editor, ctx.pcsx2, ctx.work = tyrax, editor, pcsx2, work
    ctx.cfg = ensure_private_config(work)
    ctx.dp = work / "pcsx2dp"
    ctx.clean, ctx.rebuild, ctx.no_build = a.clean, a.rebuild, a.no_build
    ctx.build_timeout, ctx.boot_timeout = a.build_timeout, a.boot_timeout
    ctx.step_stall, ctx.capture_timeout = a.step_stall, 60
    ctx.unlimited = not a.realtime
    ctx.emulog = a.emulog
    ctx.pids = set()
    ctx.isolate = a.isolate
    ctx.log_frame = None

    frames_default = parse_frames(a.frames)
    wanted = [x for x in a.only.split(",") if x] if a.only else None
    skipped = set(a.skip.split(",")) if a.skip else set()
    todo = []
    for name, tyra in list_examples(tyrax):
        if wanted and name not in wanted:
            continue
        if name in skipped:
            continue
        todo.append((name, tyra))
    if wanted:
        missing = set(wanted) - {n for n, _ in todo}
        if missing:
            sys.exit("render-captures: unknown example(s): %s" % ", ".join(sorted(missing)))

    meta = {
        "when": datetime.datetime.now().isoformat(timespec="seconds"),
        "tyrax_head": git(tyrax, "rev-parse", "--short", "HEAD").strip(),
        "tyrax_branch": git(tyrax, "rev-parse", "--abbrev-ref", "HEAD").strip(),
        "tyrax_dirty": bool(git(tyrax, "status", "--porcelain", "--untracked-files=no").strip()),
        "tyrax_subject": git(tyrax, "log", "-1", "--format=%s").strip(),
        "editor": str(editor),
        "editor_mtime": datetime.datetime.fromtimestamp(editor.stat().st_mtime).isoformat(timespec="seconds"),
        "pcsx2": str(pcsx2),
        "frames": frames_default, "repeat": a.repeat,
        "mode": "isolated: one boot per target frame, step command pre-written before boot" if a.isolate else
                "continuous: one boot, halt then step between frames",
        "renderer": "PCSX2 Software GS (Renderer=13), MTVU off, vsync off, %s" % (
            "real-time" if a.realtime else "unthrottled (-unlimited)"),
        "build": "tyrax-editor --build <copy> --debug (native/WSL), private config dir",
        "argv": sys.argv[1:],
    }
    if a.resume and (out / "run.json").exists():
        meta = json.loads((out / "run.json").read_text(encoding="utf-8"))  # keep the original run's header
    else:
        (out / "run.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    log("capture of %d example(s) -> %s (tyrax %s%s)" % (len(todo), out, meta["tyrax_head"],
                                                         "+dirty" if meta["tyrax_dirty"] else ""))

    results = {}
    results_path = out / "results.json"
    if results_path.exists() and a.resume:
        results = json.loads(results_path.read_text(encoding="utf-8"))
    for n in [x for x in (a.redo or "").split(",") if x]:
        results.pop(n, None)  # --redo: build + boot this one again even when resuming
        for f in (out / n).glob("*.png"):
            f.unlink()

    # ---- build thread: builds ahead while the main thread drives PCSX2
    q = queue.Queue(maxsize=3)
    stop = threading.Event()

    def builder():
        for name, tyra in todo:
            if stop.is_set():
                break
            if name in results and a.resume and results[name].get("runs"):
                q.put((name, tyra, None))
                continue
            od = out / name
            od.mkdir(parents=True, exist_ok=True)
            if name in SKIP_ALWAYS or tyra is None:
                q.put((name, tyra, {"status": "skipped", "seconds": 0.0, "exit": None,
                                    "notes": [SKIP_ALWAYS.get(name, "no .tyra project file")]}))
                continue
            log("[build] %s ..." % name)
            b = build_example(ctx, name, tyra, work / "p" / name, od)
            log("[build] %s: %s in %ss%s" % (name, b["status"], b["seconds"],
                                             "  " + "; ".join(b["notes"]) if b["notes"] else ""))
            q.put((name, tyra, b))
        q.put(None)

    th = threading.Thread(target=builder, daemon=True)
    th.start()

    def do_run(name, r, od, rn):
        proj = work / "p" / name
        elf_name = pathlib.Path(r["build"]["elf"]).name
        log("[run %d/%d] %s frames=%s%s" % (rn, a.repeat, name, r["frames"], " (isolated boots)" if ctx.isolate else ""))
        ctx.log_frame = r["frames"][-1]
        try:
            run = run_pcsx2(ctx, name, proj, elf_name, r["frames"], od, rn)
        except Exception as e:  # noqa: BLE001
            run = {"run": rn, "end": "harness-error", "captures": [], "notes": [repr(e)]}
        r["runs"].append(run)
        log("[run %d/%d] %s: %s, %d capture(s) in %ss  %s" % (
            rn, a.repeat, name, run["end"], len([c for c in run["captures"] if c.get("png")]),
            run.get("seconds"), "; ".join(run["notes"])))
        return run

    def save(name, r):
        (out / name / "result.json").write_text(json.dumps(r, indent=1), encoding="utf-8")
        results_path.write_text(json.dumps(results, indent=1), encoding="utf-8")

    try:
        # ---- pass 1: every example is built (by the thread) and booted once
        while True:
            item = q.get()
            if item is None:
                break
            name, tyra, b = item
            if b is None:
                log("[skip] %s: resumed from results.json" % name)
                continue
            r = {"name": name, "build": b, "runs": [], "frames": PER_EXAMPLE.get(name, {}).get(
                "frames", frames_default)}
            if b["status"] == "skipped":
                r["skipped"] = b["notes"][0]
            results[name] = r
            if b["status"] == "ok":
                do_run(name, r, out / name, 1)
            finalize_example(r)
            save(name, r)
        # ---- later passes: boot every example that produced pictures again. A pass
        # is a full sweep, so two boots of one example are minutes apart: the engine
        # seeds rand() from the guest RTC (srand(time)), and anything that rides on
        # it changes between wall-clock minutes - a re-boot seconds later would not
        # see that.
        for rn in range(2, a.repeat + 1):
            again = [n for n in sorted(results) if results[n].get("runs") and
                     any(c.get("png") for c in results[n]["runs"][0]["captures"])]
            if not again or stop.is_set():
                break
            started = time.time()
            first_min = time.strftime("%Y%m%d%H%M")
            while time.strftime("%Y%m%d%H%M") == first_min and time.time() - started < 75:
                time.sleep(1)  # never let pass N start inside the minute pass N-1 ended in
            log("---- pass %d of %d: %d example(s) ----" % (rn, a.repeat, len(again)))
            for name in again:
                r = results[name]
                do_run(name, r, out / name, rn)
                compare_runs(name, r, out / name)
                finalize_example(r)
                save(name, r)
    except KeyboardInterrupt:
        log("interrupted")
        stop.set()
    finally:
        stop.set()
        for pid in list(ctx.pids):
            kill_tree(pid)
        results_path.write_text(json.dumps(results, indent=1), encoding="utf-8")
    write_reports(out)
    if a.clean_work:
        log("removed %s (+ WSL build mirrors %s)" % (work, remove_work_root(work)))
    log("done: %s" % (out / "index.md"))


def load_rgb(path):
    return np.asarray(Image.open(path).convert("RGB"))


def compare_runs(name, r, outdir):
    """Pixel-compare run 1's pictures with the newest later run's. Identical
    pictures delete the later copy; differing ones keep it (.rN.png) plus an x8 diff."""
    det = r.setdefault("determinism", {"compared": False, "frames": {}})
    runs = r["runs"]
    if len(runs) < 2:
        return
    run = runs[-1]
    rn = run["run"]
    det["gap_minutes"] = None
    try:
        t1 = datetime.datetime.fromisoformat(runs[0]["started"])
        t2 = datetime.datetime.fromisoformat(run["started"])
        det["gap_minutes"] = round((t2 - t1).total_seconds() / 60.0, 1)
    except (KeyError, ValueError):
        pass
    for cap in run["captures"]:
        if not cap.get("png"):
            continue
        t = cap["target"]
        c1 = next((c for c in runs[0]["captures"] if c["target"] == t and c.get("png")), None)
        if c1 is None:
            continue
        det["compared"] = True
        entry = {"identical": c1["sha1"] == cap["sha1"], "run": rn}
        p2 = outdir / cap["png"]
        if not entry["identical"]:
            a_, b_ = load_rgb(outdir / c1["png"]), load_rgb(p2)
            d = pixel_diff(a_, b_)
            if d:
                entry.update(d[0])
                save_diff_png(a_, b_, outdir / ("f%04d.r2-diff.png" % t))
        else:
            try:
                p2.unlink()  # identical to run 1: nothing to keep
            except OSError:
                pass
            cap["png"] = None
            cap["dropped_identical"] = True
        det["frames"][str(t)] = entry


def example_status(r):
    """ok | crash | no-boot | timeout from what the build and the first boot did; None = not a project."""
    b = r["build"]
    if r.get("skipped"):
        return None
    if b["status"] == "timeout":
        return "timeout"
    if b["status"] != "ok":
        return "no-boot"
    runs = r.get("runs") or []
    if not runs:
        return "no-boot"
    run = runs[0]
    end = run.get("end", "?")
    got = [c for c in run["captures"] if c.get("png")]
    if any("crash.txt" in n for n in run.get("notes", [])) or end == "assert" or end.startswith("emulator-exit"):
        return "crash"
    if end in ("no-boot", "no-elf", "harness-error"):
        return "no-boot"
    if end in ("hang", "no-halt-ack") or not got:
        return "timeout"
    return "ok"


def finalize_example(r):
    b = r["build"]
    r["build_status"] = b["status"]
    r["status"] = example_status(r)
    if r.get("skipped"):
        r["boot"], r["render"] = "skipped", "skipped"
        return
    if b["status"] != "ok":
        r["boot"], r["render"] = "not built", "n/a"
        return
    runs = r["runs"]
    r1 = runs[0] if runs else None
    if not r1:
        r["boot"], r["render"] = "not run", "n/a"
        return
    got = [c for c in r1["captures"] if c.get("png")]
    r["boot"] = "ok" if (r1["end"] == "ok" or got or r1.get("halt_frame") is not None) else r1["end"]
    if r1["end"] not in ("ok",):
        r["run_end"] = r1["end"]
    r["render"] = judge_render(r1["captures"])
    det = r.get("determinism", {})
    if det.get("compared"):
        fr = det["frames"]
        ident = [v["identical"] for v in fr.values()]
        gap = det.get("gap_minutes")
        later = " vs a boot %s min later" % gap if gap is not None else ""
        r["deterministic_flag"] = bool(ident) and all(ident)
        if all(ident):
            r["deterministic"] = "yes (%d/%d frames bit-identical%s)" % (len(ident), len(ident), later)
        else:
            worst = max((v.get("pct", 0) for v in fr.values()), default=0)
            lvl = max((v.get("max", 0) for v in fr.values()), default=0)
            r["deterministic"] = "%s (%d/%d frames differ; worst %.3f%% of pixels, max %d/255 per channel)" % (
                "no" if lvl > 2 else "almost (noise level)", len([x for x in ident if not x]), len(ident), worst, lvl) + later
    else:
        r["deterministic"] = "not compared"
        r["deterministic_flag"] = False


# ------------------------------------------------------------ reports
def esc(s):
    return html.escape(str(s))


def load_review(out):
    p = pathlib.Path(out) / "review.json"
    if p.exists():
        try:
            return json.loads(p.read_text(encoding="utf-8"))
        except ValueError:
            pass
    return {}


def row_for(r, review):
    rv = review.get(r["name"], {})
    render = rv.get("render") or r.get("render", "?")
    notes = []
    b = r["build"]
    if b.get("notes"):
        notes += [n for n in b["notes"] if not n.startswith("ELF not rewritten")]
    if b["status"] != "ok" and b.get("error_lines"):
        notes.append("build: " + " | ".join(b["error_lines"][:2]))
    for run in r.get("runs", [])[:1]:
        if run["end"] != "ok":
            notes.append("run ended: %s" % run["end"])
        notes += run.get("notes", [])
    if rv.get("note"):
        notes.append(rv["note"])
    if r.get("skipped"):
        notes.append(r["skipped"])
    frames = ""
    if r.get("runs"):
        caps = [c for c in r["runs"][0]["captures"] if c.get("png") or c.get("dropped_identical")]
        frames = ",".join("%d/%d" % (c["target"], c["actual"]) for c in caps)
    return {
        "name": r["name"],
        "status": r.get("status") or "-",
        "build": {"ok": "ok", "failed": "FAILED", "timeout": "TIMEOUT"}.get(b["status"], b["status"]) if not r.get("skipped") else "skipped",
        "boot": r.get("boot", "?"),
        "render": render,
        "det": r.get("deterministic", "-"),
        "frames": frames,
        "notes": "; ".join(dict.fromkeys(notes)),
        "bsec": b.get("seconds"),
    }


def write_reports(out):
    out = pathlib.Path(out)
    results = json.loads((out / "results.json").read_text(encoding="utf-8")) if (out / "results.json").exists() else {}
    meta = json.loads((out / "run.json").read_text(encoding="utf-8")) if (out / "run.json").exists() else {}
    review = load_review(out)
    for r in results.values():
        finalize_example(r)  # idempotent: lets a newer report format apply to an old results.json
    rows = [row_for(results[n], review) for n in sorted(results)]
    # ---- markdown
    write_manifest(out, results)
    md = ["# Render captures - TyraX example projects", ""]
    md.append("- TyraX: `%s` (%s)%s - %s" % (meta.get("tyrax_head", "?"), meta.get("tyrax_branch", "?"),
                                              ", working tree dirty" if meta.get("tyrax_dirty") else "",
                                              meta.get("tyrax_subject", "")))
    md.append("- Editor binary built %s; captured %s" % (meta.get("editor_mtime", "?"), meta.get("when", "?")))
    md.append("- Emulator: %s" % meta.get("renderer", "?"))
    md.append("- Capture: the game's own framebuffer read-back (GS VRAM) at guest logic frames %s (+20 for "
              "the first command poll, see mode); %s boot pass(es) per example" % (
                  meta.get("frames", "?"), meta.get("repeat", "?")))
    md.append("- Capture mode: %s" % meta.get("mode", "continuous: one boot, halt then step between frames"))
    md.append("- Build: %s" % meta.get("build", "?"))
    md.append("")
    counts = {}
    for r in rows:
        k = r["render"].split(" ")[0]
        counts[k] = counts.get(k, 0) + 1
    md.append("Summary: %d examples - %s" % (len(rows), ", ".join("%d %s" % (v, k) for k, v in sorted(counts.items()))))
    md.append("")
    md.append("| Example | Status | Builds | Boots | Renders | Deterministic | Frames (file/logic) | Notes |")
    md.append("|---|---|---|---|---|---|---|---|")
    for r in rows:
        md.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["name"], r["status"], r["build"], r["boot"], r["render"], r["det"], r["frames"],
            r["notes"].replace("|", "/")))
    md.append("")
    extra = review.get("_notes") or []
    if extra:
        md.append("## Notes")
        md.append("")
        md += ["- " + n for n in extra]
        md.append("")
    (out / "index.md").write_text("\n".join(md), encoding="utf-8")
    # ---- html contact sheet
    h = ["<!doctype html><html lang=en><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>",
         "<title>TyraX example render captures</title><style>",
         ":root{--bg:#fff;--fg:#1c1c1c;--mut:#666;--card:#f4f4f4;--ok:#1b7f3b;--bad:#b3261e;--warn:#9a6700}",
         "@media(prefers-color-scheme:dark){:root{--bg:#161616;--fg:#e6e6e6;--mut:#999;--card:#222;--ok:#5ec27d;--bad:#ff8a80;--warn:#e0b44c}}",
         "body{background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif;margin:16px}",
         "h1{font-size:20px}.meta{color:var(--mut);margin-bottom:16px}",
         ".card{background:var(--card);border-radius:6px;padding:10px;margin:0 0 14px}",
         ".card h2{font-size:15px;margin:0 0 4px}.tag{font-size:12px;margin-left:8px;font-weight:600}",
         ".ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}",
         ".info{color:var(--mut);font-size:12px;margin:2px 0 8px}",
         ".shots{display:flex;flex-wrap:wrap;gap:8px}.shots figure{margin:0}",
         ".shots img{width:320px;max-width:100%;image-rendering:pixelated;display:block;background:#000}",
         ".shots figcaption{font-size:11px;color:var(--mut)}</style>",
         "<h1>TyraX example render captures</h1><div class=meta>%s</div>" % esc(
             "tyrax %s (%s)%s | %s | frames %s | %s" % (
                 meta.get("tyrax_head", "?"), meta.get("tyrax_branch", "?"),
                 " dirty" if meta.get("tyrax_dirty") else "", meta.get("when", ""),
                 meta.get("frames", ""), meta.get("renderer", "")))]
    if review.get("_notes"):
        h.append("<ul class=meta>%s</ul>" % "".join("<li>%s</li>" % esc(n) for n in review["_notes"]))
    for r in rows:
        res = results[r["name"]]
        cls = "ok" if r["render"].startswith(("ok", "OK")) else ("warn" if r["render"].startswith(("partly", "n/a", "skipped")) else "bad")
        h.append("<section class=card><h2>%s<span class='tag %s'>%s</span></h2>" % (esc(r["name"]), cls, esc(r["render"])))
        h.append("<div class=info>status %s | build %s | boot %s | deterministic: %s%s</div>" % (
            esc(r["status"]), esc(r["build"]), esc(r["boot"]), esc(r["det"]), (" | " + esc(r["notes"])) if r["notes"] else ""))
        h.append("<div class=shots>")
        pngs = []
        if res.get("runs"):
            for c in res["runs"][0]["captures"]:
                if c.get("png"):
                    pngs.append((c["png"], "frame %s" % c["actual"]))
            for f in sorted((out / r["name"]).glob("f????.r?.png")):
                pngs.append((f.name, "run 2 (differs)"))
            for f in sorted((out / r["name"]).glob("f*.r2-diff.png")):
                pngs.append((f.name, "diff x8"))
        for fn, cap in pngs:
            src = "%s/%s" % (r["name"], fn)
            h.append("<figure><a href='%s'><img loading=lazy src='%s' alt='%s %s'></a><figcaption>%s</figcaption></figure>" % (
                esc(src), esc(src), esc(r["name"]), esc(cap), esc(cap)))
        h.append("</div></section>")
    (out / "index.html").write_text("\n".join(h), encoding="utf-8")


def cmd_report(a):
    write_reports(a.out)
    print("wrote", pathlib.Path(a.out) / "index.md", "and index.html")


# ------------------------------------------------------------ the manifest
def pixels_sha256(path):
    """sha256 of the decoded RGB pixels (width, height, rows): the same picture hashes the same
    whatever encoder wrote the PNG."""
    rgb = load_rgb(path)
    h = hashlib.sha256()
    h.update(struct.pack("<II", rgb.shape[1], rgb.shape[0]))
    h.update(rgb.tobytes())
    return h.hexdigest()


def write_manifest(out, results):
    """DIR/manifest.json: {example: {status, deterministic, shots: [{name, frame, sha256}]}} for every
    example that is a project (a skipped folder is not listed)."""
    out = pathlib.Path(out)
    man = {}
    for name in sorted(results):
        r = results[name]
        if r.get("skipped") or not r.get("status"):
            continue
        shots = []
        for run in r.get("runs", [])[:1]:
            for cap in run["captures"]:
                png = cap.get("png")
                if png and (out / name / png).is_file():
                    shots.append({"name": pathlib.Path(png).stem, "frame": cap["actual"],
                                  "sha256": pixels_sha256(out / name / png)})
        man[name] = {"status": r["status"], "deterministic": bool(r.get("deterministic_flag")),
                     "render": r.get("render", "?"), "shots": shots}
    (out / "manifest.json").write_text(json.dumps(man, indent=1), encoding="utf-8")
    return man


def load_manifest(folder):
    p = pathlib.Path(folder) / "manifest.json"
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        sys.exit("render-captures: no readable manifest.json in %s" % folder)


# ------------------------------------------------------------ compare two captures
STATUS_RANK = {"ok": 0, "timeout": 1, "no-boot": 2, "crash": 3}
NEAR_LEVELS = 4  # a differing pixel is "near" when no channel moved more than this


def classify(a, b):
    """(verdict, stats) of two RGB arrays: IDENTICAL, NEAR or REVIEW."""
    if a.shape != b.shape:
        return "REVIEW", {"px": None, "pct": None, "max": None,
                          "note": "size %dx%d -> %dx%d" % (a.shape[1], a.shape[0], b.shape[1], b.shape[0])}
    d = np.abs(a.astype(np.int16) - b.astype(np.int16))
    px = int((d.max(axis=2) > 0).sum())
    if px == 0:
        return "IDENTICAL", {"px": 0, "pct": 0.0, "max": 0, "note": ""}
    worst = int(d.max())
    return ("NEAR" if worst <= NEAR_LEVELS else "REVIEW"), {
        "px": px, "pct": round(100.0 * px / d[:, :, 0].size, 4), "max": worst, "note": ""}


def save_side_png(a, b, path):
    """BASE on the left, NEW on the right, whatever their sizes."""
    h = max(a.shape[0], b.shape[0])
    canvas = np.zeros((h, a.shape[1] + 8 + b.shape[1], 3), dtype=np.uint8)
    canvas[:a.shape[0], :a.shape[1]] = a
    canvas[:b.shape[0], a.shape[1] + 8:] = b
    Image.fromarray(canvas, "RGB").save(path)


def cmd_compare(a):
    base_dir, new_dir = pathlib.Path(a.base), pathlib.Path(a.new)
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    base, new = load_manifest(base_dir), load_manifest(new_dir)
    counts = {"IDENTICAL": 0, "NEAR": 0, "REVIEW": 0, "MISSING": 0}
    worse = []
    rows = []

    def emit(verdict, label, detail=""):
        counts[verdict] = counts.get(verdict, 0) + 1
        print("%s %s%s" % (verdict, label, ("  " + detail) if detail else ""), flush=True)
        rows.append((verdict, label, detail))

    for name in sorted(base):
        be, ne = base[name], new.get(name)
        bs = be.get("status", "ok")
        ns = ne.get("status", "ok") if ne else None
        if ne is None:
            ns = "missing"
        if ns is None or STATUS_RANK.get(ns, 99) > STATUS_RANK.get(bs, 0):
            worse.append(name)
            print("STATUS %s %s -> %s" % (name, bs, ns), flush=True)
        nshots = {s["name"]: s for s in (ne or {}).get("shots", [])}
        for shot in be.get("shots", []):
            label = "%s/%s" % (name, shot["name"])
            pb = base_dir / name / (shot["name"] + ".png")
            pn = new_dir / name / (shot["name"] + ".png")
            if shot["name"] not in nshots or not pn.is_file() or not pb.is_file():
                emit("MISSING", label, "no picture in %s" % ("NEW" if pb.is_file() else "BASE"))
                continue
            ia, ib = load_rgb(pb), load_rgb(pn)
            verdict, st = classify(ia, ib)
            noisy = "" if be.get("deterministic", True) else " [base not deterministic]"
            if verdict == "IDENTICAL":
                emit(verdict, label, noisy.strip())
                continue
            detail = st["note"] or "%d px (%.3f%%), max %d/255%s" % (st["px"], st["pct"], st["max"], noisy)
            if verdict == "REVIEW":
                (out / name).mkdir(parents=True, exist_ok=True)
                if ia.shape == ib.shape:
                    save_diff_png(ia, ib, out / name / (shot["name"] + "-diff.png"))
                save_side_png(ia, ib, out / name / (shot["name"] + "-side.png"))
            emit(verdict, label, detail)
    for name in sorted(set(new) - set(base)):
        print("ADDED %s (%d shot(s))" % (name, len(new[name].get("shots", []))), flush=True)
    summary = "SUMMARY identical=%d near=%d review=%d missing=%d" % (
        counts["IDENTICAL"], counts["NEAR"], counts["REVIEW"], counts["MISSING"])
    md = ["# Render captures compared", "", "BASE `%s`, NEW `%s`" % (base_dir.name, new_dir.name), "",
          "`%s`" % summary, ""]
    if worse:
        md += ["Status got worse: " + ", ".join(worse), ""]
    md += ["| Verdict | Shot | Detail |", "|---|---|---|"]
    md += ["| %s | %s | %s |" % r for r in rows if r[0] != "IDENTICAL"]
    (out / "compare.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(summary, flush=True)
    return 1 if counts["MISSING"] or worse else 0


def cmd_sheet(a):
    """PNG contact sheets (grid of one frame per example, captioned) for a quick look."""
    from PIL import ImageDraw
    out = pathlib.Path(a.out)
    dest = pathlib.Path(a.dest) if a.dest else out / "sheets"
    dest.mkdir(parents=True, exist_ok=True)
    man = load_manifest(out)
    items = []
    for name in sorted(man):
        shots = [x for x in man[name]["shots"] if not a.frame or x["name"] == "f%04d" % a.frame]
        if shots:
            items.append((name, out / name / (shots[-1]["name"] + ".png")))
    cell, cols = a.cell, a.cols
    per = cols * a.rows
    for n in range(0, len(items), per):
        batch = items[n:n + per]
        rows = (len(batch) + cols - 1) // cols
        sheet = Image.new("RGB", (cols * cell, rows * (cell + 14)), (24, 24, 24))
        dr = ImageDraw.Draw(sheet)
        for i, (name, png) in enumerate(batch):
            im = Image.open(png).convert("RGB")
            im.thumbnail((cell, cell), Image.NEAREST if im.width < cell else Image.LANCZOS)
            x, y = (i % cols) * cell, (i // cols) * (cell + 14)
            sheet.paste(im, (x, y + 14))
            dr.text((x + 2, y + 1), "%s %s" % (name, png.stem), fill=(230, 230, 230))
        fn = dest / ("sheet-%02d.png" % (n // per + 1))
        sheet.save(fn)
        print(fn)


def wsl_path(p):
    """The /mnt/<drive>/... spelling WSL uses for a Windows path (long names, forward slashes)."""
    p = os.path.realpath(str(p))
    if len(p) > 1 and p[1] == ":":
        p = "/mnt/" + p[0].lower() + p[2:]
    return p.replace("\\", "/")


def remove_work_root(work, wsl_cache=True):
    """Delete the scratch root WITHOUT following the toolchain junction (a naive
    rmtree, or a realpath through the junction, would reach the REAL toolchain),
    then the WSL-side build mirrors the native backend keyed on this root's paths
    (~/.cache/tyrax/native/*/<sha256[:24] of the WSL path>). Keys are built from the
    root's own path strings; the editor's own caches and other checkouts' builds
    live under other keys and are not touched."""
    work = pathlib.Path(work)
    keys = {"toolchains": [], "engines": [], "projects": []}
    if work.exists():
        base = wsl_path(work)  # resolves only the root itself; no junction inside it
        key = lambda sfx: hashlib.sha256((base + sfx).encode()).hexdigest()[:24]  # noqa: E731
        keys["toolchains"].append(key("/cfg/tyra-editor/toolchain/ps2dev"))
        for vol in read_text(work / ".engine-volume").split():
            keys["engines"].append(key("/cfg/tyra-editor/native-build/" + vol))
        if (work / "p").is_dir():
            keys["projects"] += [key("/p/" + d.name) for d in (work / "p").iterdir() if d.is_dir()]
        try:
            os.rmdir(work / "cfg" / "tyra-editor" / "toolchain")  # the junction itself, never its target
        except OSError:
            pass
        rm_tree(work)
    if wsl_cache and IS_WIN and any(keys.values()) and shutil.which("wsl.exe"):
        cmds = " ".join('"$HOME/.cache/tyrax/native/%s/%s"' % (sub, k) for sub, ks in keys.items() for k in ks)
        subprocess.run(["wsl.exe", "--exec", "bash", "-c", "rm -rf " + cmds], capture_output=True)
    return {k: len(v) for k, v in keys.items()}


def cmd_clean(a):
    work = pathlib.Path(a.work or (pathlib.Path(os.environ.get("TEMP", "/tmp")) / "txb"))
    n = remove_work_root(work, wsl_cache=not a.keep_wsl_cache)
    print("removed %s and its WSL build mirrors %s" % (work, n))


def cmd_list(a):
    for name, tyra in list_examples(a.tyrax):
        print("%-24s %s" % (name, tyra.name if tyra else "(no .tyra: skipped)"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("capture", help="build, boot and photograph every example")
    r.add_argument("--out", required=True, help="output folder (DIR/<example>/<shot>.png + DIR/manifest.json)")
    r.add_argument("--tyrax", default=DEFAULT_TYRAX, help="TyraX checkout (default %(default)s)")
    r.add_argument("--editor", help="tyrax-editor binary (default <tyrax>/build/tyrax-editor.exe)")
    r.add_argument("--pcsx2", default=DEFAULT_PCSX2, help="pcsx2-qt with -datapath (default $PCSX2_EXE)")
    r.add_argument("--only", help="comma list of examples (default: every example with a .tyra)")
    r.add_argument("--skip", help="comma list to leave out")
    r.add_argument("--frames", default=DEFAULT_FRAMES, help="guest logic frames to photograph (default %(default)s)")
    r.add_argument("--repeat", type=int, default=2, help="boots per example; >1 gives the determinism verdict (default 2)")
    r.add_argument("--work", help="scratch root (default %%TEMP%%\\txb): project copies, private editor config, build caches")
    r.add_argument("--clean", action="store_true", help="wipe each project copy first (full regen + recompile of the game)")
    r.add_argument("--rebuild", action="store_true", help="pass --rebuild to the editor (engine too)")
    r.add_argument("--no-build", action="store_true", help="reuse the ELF already in the work copy")
    r.add_argument("--build-timeout", type=int, default=1500)
    r.add_argument("--boot-timeout", type=int, default=150, help="seconds to the game's first frame (default 150)")
    r.add_argument("--step-stall", type=int, default=120, help="seconds without frame progress before 'hang' (default 120)")
    r.add_argument("--continuous", dest="isolate", action="store_false",
                   help="ONE boot per example, halting and stepping between frames. Faster, but the host's "
                        "halt reaction time and each capture's file writes are inside the guest timeline, so "
                        "time-driven scenes (day-night cycle, animated HUD, FPS text) drift between runs. "
                        "The default is isolated boots: one per target frame, the step command pre-written "
                        "into bin/livedbg.cmd before the emulator starts")
    r.set_defaults(isolate=True)
    r.add_argument("--realtime", action="store_true", help="frame-limited emulator instead of -unlimited")
    r.add_argument("--emulog", action="store_true", help="also keep PCSX2's own log (slow)")
    r.add_argument("--resume", action="store_true", help="skip examples already in results.json")
    r.add_argument("--redo", help="with --resume: comma list of examples to do again")
    r.add_argument("--clean-work", action="store_true", help="delete the scratch root when done")
    r.set_defaults(fn=cmd_capture)
    p = sub.add_parser("report", help="regenerate manifest.json, index.md and index.html from results.json (+ review.json)")
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_report)
    c = sub.add_parser("compare", help="compare two capture folders shot by shot (exit 1: a MISSING shot or a worse status)")
    c.add_argument("base")
    c.add_argument("new")
    c.add_argument("--out", required=True, help="where REVIEW shots write <example>/<shot>-diff.png and -side.png, and compare.md")
    c.set_defaults(fn=cmd_compare)
    sh = sub.add_parser("sheet", help="PNG contact sheets of a capture (to look at)")
    sh.add_argument("--out", required=True)
    sh.add_argument("--dest")
    sh.add_argument("--frame", type=int, default=400, help="which frame (0 = the last one of each example)")
    sh.add_argument("--cell", type=int, default=320)
    sh.add_argument("--cols", type=int, default=3)
    sh.add_argument("--rows", type=int, default=3)
    sh.set_defaults(fn=cmd_sheet)
    cl = sub.add_parser("clean", help="delete the scratch root (project copies, private editor config, "
                                       "WSL build mirrors) without touching the real toolchain")
    cl.add_argument("--work")
    cl.add_argument("--keep-wsl-cache", action="store_true")
    cl.set_defaults(fn=cmd_clean)
    l = sub.add_parser("list")
    l.add_argument("--tyrax", default=DEFAULT_TYRAX)
    l.set_defaults(fn=cmd_list)
    a = ap.parse_args()
    return a.fn(a) or 0


if __name__ == "__main__":
    sys.exit(main())
