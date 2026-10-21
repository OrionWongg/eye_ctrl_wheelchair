#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
[[ -x .venv-model/bin/python ]] || python3 -m venv --system-site-packages .venv-model
.venv-model/bin/python -m pip install -r requirements-teyed-export.txt
python3 - <<'PY'
import hashlib,json,urllib.request
from pathlib import Path
root=Path('models/teyed')
sources=json.loads((root/'sources.json').read_text())
for name,record in sources.items():
    path=root/name
    if not path.exists():urllib.request.urlretrieve(record['url'],path)
    if hashlib.sha256(path.read_bytes()).hexdigest()!=record['sha256']:
        raise RuntimeError('Model checksum mismatch: '+name)
PY
mkdir -p model-results
.venv-model/bin/python tools/export_teyed.py
for MODEL in teyed meye; do
  /usr/src/tensorrt/bin/trtexec --onnx="models/teyed/${MODEL}.onnx" \
    --saveEngine="models/teyed/${MODEL}.engine" --fp16 --builderOptimizationLevel=0 --skipInference
done
