#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(CDPATH= cd -- "$test_dir/.." && pwd)
test_bin=$(mktemp "${TMPDIR:-/tmp}/betterpushback-release-version.XXXXXX")
trap 'rm -f "$test_bin"' EXIT HUP INT TERM

# cfg.cpp consumes the same header; verify both C and C++ compilation.
cc -std=c99 -Wall -Wextra -Werror -I"$repo_dir/src" \
    "$test_dir/release_version_test.c" -o "$test_bin"
"$test_bin"
c++ -x c++ -std=c++11 -Wall -Wextra -Werror -I"$repo_dir/src" \
    "$test_dir/release_version_test.c" -o "$test_bin"
"$test_bin"
