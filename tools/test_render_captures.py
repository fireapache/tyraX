"""Host-only tests of the render-captures comparer, on synthetic PNGs (no emulator).

    python -m pytest tools/test_render_captures.py
    python -m unittest discover -s tools -p "test_render_captures.py"
"""
import contextlib
import importlib.util
import io
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

TOOL = Path(__file__).resolve().parent / "render-captures.py"
spec = importlib.util.spec_from_file_location("render_captures", TOOL)
rc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rc)


def picture(seed, w=64, h=48):
    rng = np.random.RandomState(seed)
    # mid-range values so that +40 on a channel never saturates
    return rng.randint(40, 200, size=(h, w, 3)).astype(np.uint8)


def write_capture(root, examples):
    """examples: {name: (status, deterministic, {shot: array})} -> root/<name>/<shot>.png + manifest.json"""
    manifest = {}
    for name, (status, det, shots) in examples.items():
        listed = []
        for i, (shot, arr) in enumerate(shots.items()):
            (root / name).mkdir(parents=True, exist_ok=True)
            Image.fromarray(arr, "RGB").save(root / name / (shot + ".png"))
            listed.append({"name": shot, "frame": 120 + i, "sha256": rc.pixels_sha256(root / name / (shot + ".png"))})
        manifest[name] = {"status": status, "deterministic": det, "shots": listed}
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def run_compare(base, new, out):
    """(exit code, stdout lines) of `compare` run in-process."""
    args = type("A", (), {"base": str(base), "new": str(new), "out": str(out)})()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        code = rc.cmd_compare(args)
    return code, buf.getvalue().splitlines()


class CompareTests(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="rc-test-"))
        self.base, self.new, self.out = self.tmp / "base", self.tmp / "new", self.tmp / "out"
        self.examples = {
            "alpha": ("ok", True, {"f0100": picture(1), "f0400": picture(2)}),
            "beta": ("ok", False, {"f0100": picture(3)}),
        }
        write_capture(self.base, self.examples)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def new_from_base(self, mutate=None, statuses=None):
        """NEW = BASE's pictures with `mutate(examples)` applied; `statuses` overrides an example's status."""
        ex = {n: (st, d, {k: v.copy() for k, v in shots.items()}) for n, (st, d, shots) in self.examples.items()}
        if mutate:
            mutate(ex)
        for n, status in (statuses or {}).items():
            ex[n] = (status,) + ex[n][1:]
        write_capture(self.new, ex)

    def test_a_folder_against_its_copy_is_all_identical(self):
        shutil.copytree(self.base, self.new)
        code, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 0)
        self.assertEqual(lines[-1], "SUMMARY identical=3 near=0 review=0 missing=0")
        self.assertEqual(sum(1 for l in lines if l.startswith("IDENTICAL ")), 3)
        self.assertEqual(list(self.out.rglob("*.png")), [])

    def test_three_levels_on_ten_pixels_is_near(self):
        def mutate(ex):
            arr = ex["alpha"][2]["f0100"]
            arr[0, :10, 0] = arr[0, :10, 0] + 3
        self.new_from_base(mutate)
        code, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 0)
        self.assertEqual(lines[-1], "SUMMARY identical=2 near=1 review=0 missing=0")
        near = [l for l in lines if l.startswith("NEAR ")]
        self.assertEqual(len(near), 1)
        self.assertIn("alpha/f0100", near[0])
        self.assertIn("10 px", near[0])
        self.assertEqual(list(self.out.rglob("*.png")), [])

    def test_four_levels_is_still_near_and_five_is_review(self):
        def mutate(ex):
            ex["alpha"][2]["f0100"][0, 0, 1] += 4
            ex["alpha"][2]["f0400"][0, 0, 1] += 5
        self.new_from_base(mutate)
        _, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(lines[-1], "SUMMARY identical=1 near=1 review=1 missing=0")

    def test_forty_levels_is_review_and_writes_a_diff_and_a_side_by_side(self):
        def mutate(ex):
            arr = ex["alpha"][2]["f0400"]
            arr[5, :10, :] = arr[5, :10, :] + 40
        self.new_from_base(mutate)
        code, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 0, "a REVIEW shot is for a human, it does not fail the exit code")
        self.assertEqual(lines[-1], "SUMMARY identical=2 near=0 review=1 missing=0")
        self.assertTrue((self.out / "alpha" / "f0400-diff.png").is_file())
        side = np.asarray(Image.open(self.out / "alpha" / "f0400-side.png"))
        self.assertEqual(side.shape[0], 48)
        self.assertGreater(side.shape[1], 2 * 64)
        diff = np.asarray(Image.open(self.out / "alpha" / "f0400-diff.png"))
        self.assertEqual(int((diff.max(axis=2) > 0).sum()), 10)
        self.assertEqual(sorted(p.name for p in (self.out / "alpha").glob("*.png")),
                         ["f0400-diff.png", "f0400-side.png"])

    def test_a_deleted_shot_is_missing_and_exits_1(self):
        shutil.copytree(self.base, self.new)
        (self.new / "alpha" / "f0400.png").unlink()
        code, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 1)
        self.assertEqual(lines[-1], "SUMMARY identical=2 near=0 review=0 missing=1")
        self.assertTrue(any(l.startswith("MISSING alpha/f0400") for l in lines))

    def test_a_shot_dropped_from_the_manifest_is_missing(self):
        shutil.copytree(self.base, self.new)
        man = json.loads((self.new / "manifest.json").read_text(encoding="utf-8"))
        man["alpha"]["shots"] = man["alpha"]["shots"][:1]
        (self.new / "manifest.json").write_text(json.dumps(man), encoding="utf-8")
        code, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 1)
        self.assertEqual(lines[-1], "SUMMARY identical=2 near=0 review=0 missing=1")

    def test_an_example_gone_from_new_is_all_missing(self):
        shutil.copytree(self.base, self.new)
        man = json.loads((self.new / "manifest.json").read_text(encoding="utf-8"))
        del man["beta"]
        (self.new / "manifest.json").write_text(json.dumps(man), encoding="utf-8")
        code, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 1)
        self.assertEqual(lines[-1], "SUMMARY identical=2 near=0 review=0 missing=1")

    def test_ok_to_crash_exits_1_even_with_identical_pictures(self):
        self.new_from_base(statuses={"beta": "crash"})
        code, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 1)
        self.assertIn("STATUS beta ok -> crash", lines)
        self.assertEqual(lines[-1], "SUMMARY identical=3 near=0 review=0 missing=0")

    def test_a_status_that_got_better_or_stayed_bad_exits_0(self):
        base_ex = dict(self.examples)
        base_ex["beta"] = ("crash", False, {"f0100": picture(3)})
        shutil.rmtree(self.base)
        write_capture(self.base, base_ex)
        self.new_from_base()  # beta is ok again
        code, _ = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 0)
        shutil.rmtree(self.new)
        self.new_from_base(statuses={"beta": "crash"})  # crash -> crash
        code, _ = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 0)

    def test_a_picture_that_changed_size_is_review(self):
        def mutate(ex):
            ex["beta"][2]["f0100"] = picture(3, w=80, h=48)
        self.new_from_base(mutate)
        code, lines = run_compare(self.base, self.new, self.out)
        self.assertEqual(code, 0)
        self.assertEqual(lines[-1], "SUMMARY identical=2 near=0 review=1 missing=0")
        self.assertTrue((self.out / "beta" / "f0100-side.png").is_file())

    def test_the_pixel_hash_ignores_how_the_png_was_encoded(self):
        arr = picture(7)
        a, b = self.tmp / "a.png", self.tmp / "b.png"
        Image.fromarray(arr, "RGB").save(a, optimize=True)
        Image.fromarray(arr, "RGB").save(b, compress_level=0)
        self.assertNotEqual(a.read_bytes(), b.read_bytes())
        self.assertEqual(rc.pixels_sha256(a), rc.pixels_sha256(b))
        arr[0, 0, 0] ^= 1
        Image.fromarray(arr, "RGB").save(b)
        self.assertNotEqual(rc.pixels_sha256(a), rc.pixels_sha256(b))

    def test_the_command_line_exit_code_and_summary(self):
        shutil.copytree(self.base, self.new)
        cmd = [sys.executable, str(TOOL), "compare", str(self.base), str(self.new), "--out", str(self.out)]
        ok = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertIn("SUMMARY identical=3 near=0 review=0 missing=1".replace("missing=1", "missing=0"), ok.stdout)
        (self.new / "beta" / "f0100.png").unlink()
        bad = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(bad.returncode, 1)
        self.assertIn("missing=1", bad.stdout)


class StatusTests(unittest.TestCase):
    def result(self, build="ok", end="ok", captures=1, notes=()):
        caps = [{"target": 100, "actual": 120, "png": "f0100.png"}] * captures
        return {"name": "x", "build": {"status": build},
                "runs": [{"end": end, "captures": caps, "notes": list(notes)}]}

    def test_how_a_run_ends_becomes_a_status(self):
        s = rc.example_status
        self.assertEqual(s(self.result()), "ok")
        self.assertEqual(s(self.result(end="assert")), "crash")
        self.assertEqual(s(self.result(end="emulator-exit(1)")), "crash")
        self.assertEqual(s(self.result(notes=["crash.txt written by the game"])), "crash")
        self.assertEqual(s(self.result(end="no-boot", captures=0)), "no-boot")
        self.assertEqual(s(self.result(build="failed")), "no-boot")
        self.assertEqual(s(self.result(build="timeout")), "timeout")
        self.assertEqual(s(self.result(end="hang", captures=0)), "timeout")
        self.assertEqual(s(self.result(captures=0)), "timeout")
        self.assertIsNone(s({"name": "x", "build": {"status": "skipped"}, "skipped": "not a project"}))


if __name__ == "__main__":
    unittest.main()
