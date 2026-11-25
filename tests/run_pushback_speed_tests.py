#!/usr/bin/env python3
"""Compile the production movement path twice and compare exact F1 traces."""
import filecmp
import os
import re
import subprocess
import tempfile
from pathlib import Path

repo = Path(__file__).resolve().parents[1]
lib = repo.parent / "libacfutils"
controller_ref = "72a329537104c93b432859cc1600136f653aa187"
driving_ref = "c202b4377869a1fe370ab1ac9760709f911621ce"


def source(file, ref=None):
    if ref is None:
        return (repo / "src" / file).read_text()
    return subprocess.check_output(["git", "show", f"{ref}:src/{file}"],
                                   cwd=repo, text=True)


def function(src, name):
    match = re.search(r"^(?:static\s+)?(?:inline\s+)?\w+\s+" +
                      re.escape(name) + r"\([^;{}]*\)\s*\{", src, re.M)
    assert match, name
    depth = 1
    for t in re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|[{}]',
                         src[match.end():], re.S):
        depth += (t.group() == "{") - (t.group() == "}")
        if not depth:
            return src[match.start():match.end() + t.end()]
    raise AssertionError(name)


def definitions(src, names):
    bodies = [function(src, n) for n in names]
    return "\n".join([s[:s.index("{")].strip() + ";" for s in bodies] + bodies)


def struct(src, name):
    return re.search(r"^static struct \{[^{}]*\} " + name +
                     r"(?: = \{0\})?;", src, re.M).group()


def constants(src, names):
    return "\n".join(re.search(r"^#define\s+" + n + r"\s+[^\n]+", src,
                               re.M).group() for n in names.split())


def protection():
    now, base = source("bp.c"), source("bp.c", controller_ref)
    # Explicit movement owners may change. All other original core functions
    # stay protected, including the entire STOPPED-to-release sequence.
    changed = set("""automatic_push_speed_limits bp_gather bp_start bp_run
        bp_fini bp_stop bp_complete bp_state_init recon_handler bp_run_push
        bp_request_pause bp_request_resume pb_step_pushing pb_step_stopping
        turn_nosewheel push_at_speed tug_pos_update draw_tugs telemetry_record
        telemetry_begin_frame telemetry_start_impl telemetry_refresh_speed_limits""".split())
    for name in re.findall(r"^(?:static\s+)?(?:void|bool_t|double|float|int|"
                           r"vect2_t|vehicle_pos_t)\s+(\w+)\([^;{}]*\)\s*\{",
                           base, re.M):
        if name not in changed:
            assert function(now, name) == function(base, name), name
    for path in ("fast_brake_handoff.h", "bp_cam.c"):
        assert source(path) == source(path, controller_ref), path
    writer = r"\b(?:brakes_set|bp_fast_brake_handoff_reset|dr_set\w*)\s*\([^;]*;"
    for name in ("bp_run", "pb_step_stopping", "recon_handler", "bp_complete"):
        protected = lambda s: [x for x in re.findall(writer, function(s, name))
                               if re.search(r"brakes_set|brake_handoff|drs\.(?:lbrake|rbrake|pbrake)", x)]
        assert protected(now) == protected(base), name
    save = "SettingsWindow::SaveConfig"
    old_save = function(source("cfg.cpp", controller_ref), save)
    new_save = function(source("cfg.cpp"), save)
    assert new_save == old_save.replace('"pushback_speed_factor"', '"pushback_time_factor"')
    assert source("cfg.cpp").count('conf_set_d(bp_conf, "pushback_time_factor"') == 1
    for name in ("BRAKE_PEDAL_THRESH", "STATE_TRANS_DELAY"):
        assert constants(now, name) == constants(base, name)
    for name in ("bp_run_push", "motion_factor"):
        assert not re.search(r"bp_(?:fast_ground_handling|interface_mode)", function(now, name))
    vehicle = r"typedef struct \{\s+double wheelbase;.*?\} vehicle_t;"
    assert re.search(vehicle, source("driving.h"), re.S).group() == re.search(
        vehicle, source("driving.h", driving_ref), re.S).group()


def prepare(temp, ref, seeded):
    bp, bh, tug, th, drv = (source(n, ref) for n in
                             ("bp.c", "bp.h", "tug.c", "tug.h", "driving.c"))
    if seeded:
        # Only this uninitialized caller local is seeded in the separate
        # full-trace oracle. Both frozen driving producers stay pristine.
        body = function(bp, "bp_run_push")
        assert ref == controller_ref and body.count("bool_t decel;") == 1
        bp = bp.replace(body, body.replace("bool_t decel;", "bool_t decel = B_FALSE;"))
    for header in ("driving.h", "pushback_speed.h", "telemetry.h"):
        (temp / header).write_text(source(header, ref))
    (temp / "telemetry.c").write_text(source("telemetry.c", ref))
    names = """veh_pos2fixed_pos drive_on_line next_seg_speed straight_run_speed
        turn_run_speed turn_run drive_segs ang_vel_speed_limit""".split()
    historical = definitions(source("driving.c", driving_ref), names)
    historical = re.sub(r"\b(" + "|".join(names) + r")\b",
                        lambda m: "reference_" + m.group(), historical)
    # Include the real helper closure, including route lookahead, not a
    # simplified test implementation. Geometry & persistence are separate.
    pattern = r"^(?:static\s+)?(?:inline\s+)?\w+\s+(\w+)\([^;{}]*\)\s*\{"
    available = set(re.findall(pattern, drv, re.M))
    current = set(names)
    current.update(available & {"drive_route_envelope", "drive_segs_with_speed_limits"})
    while True:
        called = set(re.findall(r"\b(\w+)\s*\(", definitions(drv, sorted(current))))
        extra = (called & available) - current
        if not extra:
            break
        current.update(extra)
    (temp / "speed_driving.inc").write_text(definitions(drv, sorted(current)) + "\n" + historical)
    parts = []
    for file, funcs in (("geom", "is_on_arc vect2_abs vect2_dist vect2_set_abs vect2_add vect2_sub vect2_scmul vect2_dotprod vect2_norm vect2_rot vect2_neg hdg2dir dir2hdg"),
                        ("math", "quadratic_solve fx_lin"), ("helpers", "rel_hdg_impl")):
        parts.append(definitions((lib / "src" / (file + ".c")).read_text(), funcs.split()))
    (temp / "speed_geometry.inc").write_text("#define ROUND_ERROR 1e-10\n" + "\n".join(parts))
    fixture = [bh[bh.index("typedef struct"):bh.index("} bp_state_t;") + len("} bp_state_t;")],
               bh[bh.index("typedef struct {", bh.index("} bp_long_state_t;")):bh.index("} push_manual_t;") + len("} push_manual_t;")],
               th[th.index("typedef enum"):th.index("} tug_t;") + len("} tug_t;")],
               struct(bp, "drs").replace("dr_t ", "double "), struct(bp, "bp_telem")]
    if not ref:
        fixture += ['#include "pushback_motion_force.h"', '#include "motion_envelope.h"', struct(bp, "motion")]
    (temp / "speed_fixture.inc").write_text("\n".join(fixture))
    (temp / "speed_constants.inc").write_text(constants(bp, """
        BRAKE_PEDAL_THRESH STATE_TRANS_DELAY FORCE_PER_TON BREAKAWAY_THRESH
        MAX_SPEED_MED_FRICTION MAX_SPEED_POOR_FRICTION SPEED_COMPLETE_THRESH
        MIN_STEP_TIME PB_CRADLE_DELAY PB_START_DELAY MAX_DRIVING_AWAY_DELAY
        NEARING_END_THRESHOLD TOW_COMPLETE_TUG_STEER_THRESH TOW_COMPLETE_ACF_STEER_THRESH""")
        + "\n" + constants(tug, "TUG_MAX_ACCEL TUG_STEER_RATE TUG_FAST_STEER_RATE")
        + "\n" + constants((lib / "src/acfutils/perf.h").read_text(),
                           r"EARTH_GRAVITY MASS2GFORCE\(m\)"))
    roots = """bp_run bp_start bp_stop bp_delete_all_segs bp_run_push
        turn_nosewheel push_at_speed tug_speed tug_pos_update tug_rear2acf_nw reorient_aircraft
        bp_gather draw_tugs pb_step_pushing pb_step_stopping bp_request_pause
        bp_request_resume recon_handler corr_acf_pos nearing_end telemetry_begin_frame
        telemetry_start telemetry_start_impl telemetry_sanitize_name telemetry_refresh_speed_limits""".split()
    if ref:
        roots += ["automatic_push_speed_limits"]
    else:
        roots += ["stopping_alignment"]
        roots += [n for n in re.findall(pattern, bp, re.M) if n.startswith(
            ("motion_", "bp_motion_", "turn_nosewheel_fast", "push_at_speed_fast"))]
    tug_names = "tug_run tug_set_pos tug_set_steering tug_lift_wall_off tug_is_stopped".split()
    if not ref:
        tug_names += ["tug_run_attached", "tug_run_impl"]
    (temp / "speed_controller.inc").write_text("\n".join((
        function(source("cfg.cpp", ref), "bp_pushback_speed_factor"),
        definitions(tug, tug_names), definitions(bp, roots))))


def run():
    protection()
    with tempfile.TemporaryDirectory(prefix="bpb-motion-") as tmp:
        traces = []
        prefixes = []
        for label, ref, seeded in (("72a3295-raw", controller_ref, False),
                                   ("72a3295-crawl-defined", controller_ref, True),
                                   ("candidate", None, False)):
            temp = Path(tmp) / label
            temp.mkdir()
            prepare(temp, ref, seeded)
            binary, trace = temp / "controller", temp / "f1.trace"
            extra_sources = []
            if not ref:
                # Exercise the owner's optional route-list tests against the
                # same production lookahead and list library as the controller.
                route_test = temp / "route_envelope_cases.c"
                route_test.write_text('#define BP_TEST_ROUTE_ENVELOPE\n'
                    '#define main route_envelope_cases\n'
                    '#include "motion_envelope_test.c"\n')
                extra_sources.append(str(route_test))
            subprocess.run([os.environ.get("CC", "cc"), "-std=c99", "-Wall", "-Wextra",
                "-Werror", "-DDEBUG", "-D_GNU_SOURCE", "-DAPL=0", "-DIBM=0", "-DLIN=1",
                "-DXPLM200=1", "-DXPLM300=1", "-DXPLM400=1", "-DBP_ENABLE_RUNTIME_TELEMETRY=1",
                f"-DFROZEN={int(bool(ref))}", f"-I{temp}", f"-I{repo / 'src'}",
                f"-I{lib / 'src'}", f"-I{lib / 'SDK/CHeaders/XPLM'}",
                str(repo / "tests/pushback_speed_test.c"), str(lib / "src/list.c"),
                str(temp / "telemetry.c"), f"-I{repo / 'tests'}", *extra_sources,
                "-lm", "-o", str(binary)], check=True)
            prefix = temp / "defined-prefix.trace"
            subprocess.run([str(binary), str(temp), str(trace), str(prefix)], check=True)
            prefixes.append(prefix)
            if label != "72a3295-raw":
                traces.append(trace.read_bytes())
        assert prefixes[0].stat().st_size and all(filecmp.cmp(prefixes[0], p, shallow=False)
            for p in prefixes[1:]), (
            "1x differs on defined prefixes from untouched 72a3295")
        assert traces[0] and traces[1]
        if traces[0] != traces[1]:
            rows = [t.decode().splitlines() for t in traces]
            for i, (original, candidate) in enumerate(zip(*rows), 1):
                assert original == candidate, (
                    f"1x differs from 72a3295 at sample {i}:\n{original}\n{candidate}")
            raise AssertionError("1x trace length differs from 72a3295")


if __name__ == "__main__":
    run()
