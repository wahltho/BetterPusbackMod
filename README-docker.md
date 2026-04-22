# Cross-compile to linux,windows from a mac

## Pre-requisites

- docker for mac 

that's it 

## Settings
Keep the source checkout under `~/Documents/Projects/BPB`, but stage builds
into the local workspace at `~/dev/BPB`:

```
cd ~/Documents/Projects/BPB
./workbench/sync_local_workspace.sh
```

Then run Docker against `~/dev/BPB` instead of the iCloud-backed source tree:

```
docker run --rm --platform linux/amd64 \
  -e BPB_BUILD_ROOT=/xpl_dev \
  -e BPB_LOCAL_WORKSPACE=/xpl_dev \
  -v ~/dev/BPB:/xpl_dev \
  -v ~/dev/BPB:/work \
  -w /xpl_dev/BetterPusbackMod-main \
  bpb-cross \
  bash -lc "./build_xpl.sh"
```
