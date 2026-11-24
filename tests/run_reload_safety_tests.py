#!/usr/bin/env python3
"""Exercise real preferences-save and deferred-reload code with SDK stubs."""

import os
from pathlib import Path
import re
import subprocess
import tempfile


def function(source, name):
    match = re.search(r"^(?:static\s+)?(?:void|bool_t|float)\s+"
                      + re.escape(name) + r"\([^)]*\)\s*\{", source, re.M)
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


repo = Path(__file__).resolve().parent.parent
cfg = (repo / "src/cfg.cpp").read_text()
xp = (repo / "src/xplane.c").read_text()
assert "save_disabled(bp_started)" in cfg
assert "const bool save_blocked = !preferences_save_allowed(save_disabled);" in cfg
assert cfg.count("if (save_blocked)") == 2
assert "SaveConfig();" in cfg
with tempfile.TemporaryDirectory(prefix="bpb-reload-safety-") as temp:
    temp = Path(temp)
    (temp / "reload_safety.inc").write_text("\n\n".join([
        function(cfg, "preferences_save_allowed"),
        function(xp, "bp_do_reload"), function(xp, "bp_sched_reload"),
        function(cfg, "bp_conf_set_save_enabled"),
        function(cfg, "SettingsWindow::SaveConfig"),
    ]))
    binary = temp / "reload-safety"
    subprocess.run([os.environ.get("CXX", "c++"), "-x", "c++", "-std=c++11",
                    "-Wall", "-Wextra", "-Werror", f"-I{temp}",
                    f"-I{repo / 'src'}",
                    str(repo / "tests/reload_safety_test.c"), "-o", str(binary)],
                   check=True)
    subprocess.run([str(binary)], check=True)
