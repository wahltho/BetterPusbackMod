#!/usr/bin/env python3
"""Compare the actual first coupled tug step against the frozen 1x code."""
import os
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
current = (root / "src/tug.c").read_text()
base = subprocess.check_output(
    ["git", "show", "72a3295:src/tug.c"], cwd=root, text=True)
legacy = base[base.index("void\ntug_run("):base.index("void\ntug_anim(")]
production = current[current.index("static void\ntug_run_impl("):
                     current.index("void\ntug_anim(")]
constants = [re.search(r"^#define\s+" + key + r"\s+[^\n]+", current,
                       re.M).group() for key in ["TUG_MAX_ACCEL", "TUG_STEER_RATE"]]
with tempfile.TemporaryDirectory(prefix="bpb-attached-tug-") as tmp:
    path = Path(tmp)
    (path / "steps.inc").write_text("\n".join(constants) + "\n" + production +
                                   legacy.replace("void\ntug_run(", "void\nreference_run("))
    binary = path / "test"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c99", "-Wall", "-Wextra",
                    "-Werror", "-I" + tmp, str(root / "tests/attached_motion_test.c"),
                    "-lm", "-o", str(binary)], check=True)
    subprocess.run([binary], check=True)
