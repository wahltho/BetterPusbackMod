#!/usr/bin/env python3
"""Compile actual route/controller code and compare against the frozen baseline."""
import os
from pathlib import Path
import re
import subprocess
import tempfile

BASELINE = "c202b4377869a1fe370ab1ac9760709f911621ce"
repo = Path(__file__).resolve().parent.parent
lib = repo.parent / "libacfutils"


def function(source, name):
    match = re.search(r"^(?:static\s+)?(?:inline\s+)?[\w]+\s+"
                      + re.escape(name) + r"\([^;{}]*\)\s*\{", source, re.M)
    assert match, name
    depth = 1
    for token in re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\{|\}',
                             source[match.end():], re.S):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if not depth:
                return source[match.start():match.end() + token.end()]
    raise AssertionError(name)


def original(path):
    return subprocess.check_output(["git", "show", f"{BASELINE}:{path}"],
                                   cwd=repo, text=True)


driving = (repo / "src/driving.c").read_text()
baseline = original("src/driving.c")
bp = (repo / "src/bp.c").read_text()
base_bp = original("src/bp.c")
cfg = (repo / "src/cfg.cpp").read_text()
names = ["veh_pos2fixed_pos", "drive_on_line", "next_seg_speed",
         "straight_run_speed", "turn_run_speed", "turn_run", "drive_segs",
         "ang_vel_speed_limit"]

# Check every original core function, not just a selected set of brake writes.
allowed = {"automatic_push_speed_limits", "telemetry_start", "telemetry_record", "bp_state_init",
           "bp_start", "bp_run_push"}
for match in re.finditer(r"^(?:static\s+)?(?:void|bool_t|double|float|int|vect2_t|"
                        r"vehicle_pos_t)\s+(\w+)\([^;{}]*\)\s*\{", base_bp, re.M):
    name = match.group(1)
    if name not in allowed:
        assert function(bp, name) == function(base_bp, name), name
for path in ["src/bp_cam.c", "src/tug.c", "src/tug.h", "src/fast_brake_handoff.h"]:
    assert (repo / path).read_text() == original(path), path
assert function(bp, "telemetry_record").replace(
    "    telemetry_refresh_speed_limits();\n", "", 1) == function(base_bp, "telemetry_record")
header = (repo / "src/driving.h").read_text()
base_header = original("src/driving.h")
vehicle = r"typedef struct \{\s+double wheelbase;.*?\} vehicle_t;"
assert re.search(vehicle, header, re.S).group() == re.search(
    vehicle, base_header, re.S).group()
assert "bp.automatic_push_speed_factor = 1.0;" in function(bp, "bp_state_init")
for name in ["recon_handler", "bp_complete"]:
    assert "automatic_push_speed_factor" not in function(bp, name)
assert "bp_run_push_manual();" in function(bp, "bp_run_push")
assert 'conf_set_d(bp_conf, "pushback_speed_factor"' in function(cfg, "SettingsWindow::SaveConfig")
for name in ["automatic_push_speed_limits", "bp_run_push"]:
    body = function(bp, name)
    assert "bp_fast_ground_handling" not in body and "bp_interface_mode" not in body

with tempfile.TemporaryDirectory(prefix="bpb-pushback-speed-") as directory:
    temp = Path(directory)
    current = [function(driving, n) for n in names +
               ["drive_segs_impl", "drive_segs_with_speed_limits"]]
    reference = [function(baseline, n) for n in names]
    mapping = re.compile(r"\b(" + "|".join(names) + r")\b")
    reference = [mapping.sub(lambda m: "reference_" + m.group(), s) for s in reference]
    all_functions = current + reference
    prototypes = [s[:s.index("{")].strip() + ";" for s in all_functions]
    (temp / "speed_driving.inc").write_text("\n".join(prototypes + all_functions))
    geometry = (lib / "src/geom.c").read_text()
    math = (lib / "src/math.c").read_text()
    (temp / "speed_geometry.inc").write_text("#define ROUND_ERROR 1e-10\n" +
        "\n".join(function(geometry, n) for n in ["is_on_arc", "vect2_abs",
         "vect2_set_abs", "vect2_add", "vect2_sub", "vect2_scmul",
         "vect2_dotprod", "vect2_norm", "vect2_rot", "vect2_neg", "hdg2dir",
         "dir2hdg"]) + "\n" + function(math, "quadratic_solve") + "\n" +
        function(math, "fx_lin") + "\n" +
        function((lib / "src/helpers.c").read_text(), "rel_hdg_impl"))
    telemetry = bp[bp.index("    metadata.effective_max_fwd_speed_mps ="):
                   bp.index("    metadata.push_start_accel_ramp_time_s =")]
    (temp / "speed_controller.inc").write_text("\n\n".join([
        function(bp, "automatic_push_speed_limits"),
        function(bp, "bp_run_push"), function(bp, "turn_nosewheel"),
        function(bp, "bp_start"), function(cfg, "bp_pushback_speed_factor"),
        "static void speed_metadata(void) {\n" + telemetry + "\n}",
    ]))
    (temp / "speed_refresh.inc").write_text(function(bp, "telemetry_refresh_speed_limits"))
    (temp / "speed_publication.inc").write_text("\n".join(function(bp, n) for n in [
        "telemetry_sanitize_name", "telemetry_start_impl", "telemetry_start"]))
    (temp / "speed_constants.inc").write_text("\n".join(
        re.search(r"^#define\s+" + name + r"\s+[^\n]+", bp, re.M).group()
        for name in ["BRAKE_PEDAL_THRESH", "STATE_TRANS_DELAY"]))
    for name in ["BRAKE_PEDAL_THRESH", "STATE_TRANS_DELAY"]:
        assert re.search(r"^#define\s+" + name + r"\s+[^\n]+", bp, re.M).group() == re.search(
            r"^#define\s+" + name + r"\s+[^\n]+", base_bp, re.M).group()
    binary = temp / "speed-tests"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c99", "-Wall", "-Wextra",
        "-Werror", "-DDEBUG", "-D_GNU_SOURCE", "-DAPL=0", "-DIBM=0", "-DLIN=1",
        "-DXPLM200=1", "-DXPLM300=1", "-DXPLM400=1",
        "-DBP_ENABLE_RUNTIME_TELEMETRY=1",
        f"-I{temp}", f"-I{repo / 'src'}", f"-I{lib / 'src'}",
        f"-I{lib / 'SDK/CHeaders/XPLM'}",
        str(repo / "tests/pushback_speed_test.c"), str(lib / "src/list.c"),
        str(repo / "src/telemetry.c"),
        "-lm", "-o", str(binary)], check=True)
    subprocess.run([str(binary), str(temp)], check=True)
print("Protected core functions, vehicle limits, tug, planner and brake source: unchanged")
