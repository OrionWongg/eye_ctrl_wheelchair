#!/usr/bin/env bash
set -e
root="$(cd "$(dirname "$0")/.." && pwd)"
src="$root/projects/EYETLE/third_party/PyPupilEXT/src/pupil-detection-methods"
mkdir -p "$root/.local/lib"
g++ -O3 -std=c++17 -shared -fPIC -I"$src" \
  "$root/runtime/native/purest.cpp" "$src/PuRe.cpp" "$src/PuReST.cpp" \
  "$src/PupilDetectionMethod.cpp" $(pkg-config --cflags --libs opencv4) \
  -o "$root/.local/lib/libwhill_purest.so"
