#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="$ROOT/third_party/3D-Eye-Tracker"
REVISION=a8e2180577308596b6ac8078558df1e430789191
if [[ ! -d "$SOURCE/.git" ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/YutaItoh/3D-Eye-Tracker.git "$SOURCE"
  git -C "$SOURCE" sparse-checkout init --cone
  git -C "$SOURCE" sparse-checkout set main singleeyefitter
  git -C "$SOURCE" checkout "$REVISION"
fi
git -C "$SOURCE" show HEAD:LICENSE > "$SOURCE/LICENSE"
python3 "$ROOT/tools/itoh/build.py"
