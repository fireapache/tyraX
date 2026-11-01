#!/usr/bin/env python3
"""Build and run the memory-manager host verifiers (tools/verify-memory-*.cpp).

Every verifier is compiled with all of vendor/tyra/engine/src/memory/*.cpp (not
the ps2/ folder) twice, once with -DTYRA_MEMORY_CHECKS=0 and once with =1, run,
and must print "ALL <n> CHECKS PASSED" and exit 0. Nothing is written into the
repository: the builds live in a temporary folder. Needs a host g++ or clang++
(C++17); never builds the editor or a game and never starts an emulator.

  python tools/verify-memory.py                  # every verifier, both modes
  python tools/verify-memory.py --only heap      # tools/verify-memory-heap.cpp
  python tools/verify-memory.py --compiler g++   # pick the compiler
  python tools/verify-memory.py --sanitize       # + -fsanitize=address,undefined
                                                 #   (run it where that exists,
                                                 #   e.g. under WSL)

Exits non-zero when anything fails to build, fails a check or crashes. The last
line is "PASS memory verifiers (<n> builds)" when everything passed.
"""
import argparse
import concurrent.futures
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
ENGINE = ROOT / "vendor" / "tyra" / "engine"
PREFIX = "verify-memory-"
CHECK_MODES = (0, 1)
SANITIZE_FLAGS = ("-fsanitize=address,undefined", "-fno-sanitize-recover=undefined",
                  "-fno-omit-frame-pointer")
BUILD_TIMEOUT = 900
RUN_TIMEOUT = 900


def verifiers():
    """{name: source path} for every tools/verify-memory-*.cpp."""
    found = {}
    for path in sorted(TOOLS.glob(PREFIX + "*.cpp")):
        found[path.stem[len(PREFIX):]] = path
    return found


def engine_sources():
    """The portable memory sources; the ps2/ folder is the EE's and is skipped."""
    return sorted((ENGINE / "src" / "memory").glob("*.cpp"))


def tail(text, lines=40):
    rows = text.strip().splitlines()
    return "\n".join(rows[-lines:])


def build_and_run(compiler, environment, scratch, name, source, mode, sanitize, sources):
    """Compile one verifier for one TYRA_MEMORY_CHECKS value, run it. -> (ok, text)."""
    exe = scratch / ("%s-checks%d%s" % (name, mode, ".exe" if os.name == "nt" else ""))
    command = [compiler, "-std=gnu++17", "-O1", "-Wall", "-Wextra", "-Werror",
               "-I", str(ENGINE / "inc"), "-DTYRA_MEMORY_CHECKS=%d" % mode]
    if sanitize:
        command += SANITIZE_FLAGS
    command += [str(source)] + [str(s) for s in sources] + ["-o", str(exe)]
    try:
        built = subprocess.run(command, capture_output=True, text=True,
                               timeout=BUILD_TIMEOUT, env=environment)
    except subprocess.TimeoutExpired:
        return False, "build timed out after %d s" % BUILD_TIMEOUT
    if built.returncode != 0:
        return False, "build failed:\n" + tail(built.stderr + built.stdout, 60)
    try:
        ran = subprocess.run([str(exe)], capture_output=True, text=True,
                             timeout=RUN_TIMEOUT, env=environment)
    except subprocess.TimeoutExpired:
        return False, "run timed out after %d s" % RUN_TIMEOUT
    output = ran.stdout + ran.stderr
    if ran.returncode != 0:
        return False, "exit code %d:\n%s" % (ran.returncode, tail(output))
    if "ALL " not in ran.stdout or " CHECKS PASSED" not in ran.stdout:
        return False, "exited 0 but did not print ALL <n> CHECKS PASSED:\n" + tail(output)
    return True, ran.stdout.strip().splitlines()[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--only", action="append", metavar="NAME",
                        help="run only tools/verify-memory-NAME.cpp (repeatable)")
    parser.add_argument("--compiler", default=None, help="host g++/clang++ executable")
    parser.add_argument("--sanitize", action="store_true",
                        help="add -fsanitize=address,undefined (UB aborts)")
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 1,
                        help="builds in flight at once (default: the CPU count)")
    args = parser.parse_args()

    compiler = args.compiler or shutil.which("g++") or shutil.which("clang++")
    if not compiler:
        parser.error("No host compiler found; pass --compiler /path/to/g++")
    environment = dict(os.environ)
    if Path(compiler).is_file():
        # A MinGW build needs its runtime DLLs next to the compiler on PATH.
        environment["PATH"] = str(Path(compiler).resolve().parent) + os.pathsep + \
            environment.get("PATH", "")

    available = verifiers()
    names = args.only or sorted(available)
    unknown = [n for n in names if n not in available]
    if unknown:
        parser.error("no tools/%s<name>.cpp for: %s (have: %s)" %
                     (PREFIX, ", ".join(unknown), ", ".join(sorted(available)) or "none"))
    if not names:
        parser.error("no tools/%s*.cpp verifiers found" % PREFIX)
    sources = engine_sources()

    jobs = [(n, available[n], m) for n in names for m in CHECK_MODES]
    failed = 0
    with tempfile.TemporaryDirectory(prefix="tyrax-memory-", ignore_cleanup_errors=True) as tmp:
        scratch = Path(tmp)
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            futures = [pool.submit(build_and_run, compiler, environment, scratch,
                                   n, src, m, args.sanitize, sources)
                       for n, src, m in jobs]
            for (n, _, m), future in zip(jobs, futures):
                ok, text = future.result()
                label = "%-10s TYRA_MEMORY_CHECKS=%d" % (n, m)
                if ok:
                    print("ok   %s  %s" % (label, text))
                else:
                    failed += 1
                    print("FAIL %s  %s" % (label, text))
                sys.stdout.flush()

    if failed:
        print("FAIL memory verifiers (%d of %d builds failed)" % (failed, len(jobs)))
        return 1
    print("PASS memory verifiers (%d builds)" % len(jobs))
    return 0


if __name__ == "__main__":
    sys.exit(main())
