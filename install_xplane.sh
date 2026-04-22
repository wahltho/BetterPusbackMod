#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
XPLANE_PLUGIN_DIR="$HOME/X-Plane 12/Resources/plugins/BetterPushback"
ARTIFACT_ROOT="${SCRIPT_DIR}"

if [[ -d "${BPB_BUILD_ROOT:-/Users/wahltho/dev/BPB}/BetterPusbackMod-main/BetterPushback" ]]; then
	ARTIFACT_ROOT="${BPB_BUILD_ROOT:-/Users/wahltho/dev/BPB}/BetterPusbackMod-main"
fi

if [[ ! -d "${ARTIFACT_ROOT}/BetterPushback" ]]; then
	echo "BetterPushback build output not found under ${ARTIFACT_ROOT}" >&2
	exit 1
fi

case "$(uname)" in
Linux)
    echo "copying lin_x64/BetterPushback.xpl to ${XPLANE_PLUGIN_DIR}"
	cp -r "${ARTIFACT_ROOT}/BetterPushback/lin_x64" "${XPLANE_PLUGIN_DIR}"
    echo "copying win_x64/BetterPushback.xpl to ${XPLANE_PLUGIN_DIR}"
	cp -r "${ARTIFACT_ROOT}/BetterPushback/win_x64" "${XPLANE_PLUGIN_DIR}"
	;;
Darwin)
    echo "copying mac_x64/BetterPushback.xpl to ${XPLANE_PLUGIN_DIR}"
	cp -r "${ARTIFACT_ROOT}/BetterPushback/mac_x64" "${XPLANE_PLUGIN_DIR}"
    echo "removing quarantine"
    xattr -dr com.apple.quarantine "${XPLANE_PLUGIN_DIR}"
	;;
*)
	echo "Unsupported platform" >&2
	exit 1
	;;
esac
