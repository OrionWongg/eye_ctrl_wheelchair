#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
COMMIT=6431c57ce7bf0eda935fb6178b926ae9440b50bf
if [[ ! -d third_party/RITnet/.git ]]; then
  git clone https://github.com/AayushKrChaudhary/RITnet.git third_party/RITnet
  git -C third_party/RITnet checkout "$COMMIT"
fi
[[ "$(git -C third_party/RITnet rev-parse HEAD)" == "$COMMIT" ]] || { echo 'Unexpected RITnet revision' >&2; exit 1; }
git -C third_party/RITnet diff --exit-code HEAD -- densenet.py best_model.pkl
[[ -x .venv-model/bin/python ]] || python3 -m venv --system-site-packages .venv-model
.venv-model/bin/python -m pip install 'torch==2.6.0' 'onnx==1.17.0'
mkdir -p models/ritnet model-results
for SHAPE in 640x480 320x240; do
  WIDTH="${SHAPE%x*}"
  HEIGHT="${SHAPE#*x}"
  .venv-model/bin/python tools/export_ritnet.py --width "$WIDTH" --height "$HEIGHT"
  /usr/src/tensorrt/bin/trtexec --onnx="models/ritnet/ritnet_${SHAPE}.onnx" \
    --saveEngine="models/ritnet/ritnet_${SHAPE}.engine" --fp16 --builderOptimizationLevel=0 --skipInference
done
.venv-model/bin/python tools/export_ritnet.py --width 320 --height 240 --batch 2
/usr/src/tensorrt/bin/trtexec --onnx=models/ritnet/ritnet_320x240_b2.onnx \
  --saveEngine=models/ritnet/ritnet_320x240_b2.engine --fp16 --builderOptimizationLevel=0 --skipInference
cp third_party/RITnet/License.md models/ritnet/LICENSE.txt
