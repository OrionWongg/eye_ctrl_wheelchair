"""Build pinned PuReST sources as a minimal Linux C ABI without Pybind/vcpkg."""
from pathlib import Path
import base64
import hashlib
import json
import subprocess
import urllib.request

ROOT=Path(__file__).resolve().parents[2]
REV='a2999de1dee62243e3be764312556be2e4986068'
NAMES=['PuRe.cpp','PuRe.h','PuReST.cpp','PuReST.h','Pupil.h','PupilDetectionMethod.cpp','PupilDetectionMethod.h']

def main():
    source=ROOT/'third_party/PyPupilEXT';folder=source/'src/pupil-detection-methods'
    folder.mkdir(parents=True,exist_ok=True)
    paths=['src/pupil-detection-methods/'+n for n in NAMES]+['LICENSE','Third_Party_Licenses.html']
    manifest=Path(__file__).with_name('sources.json')
    expected=json.loads(manifest.read_text()) if manifest.exists() else {}
    for name in paths:
        path=source/name
        if not path.exists():
            with urllib.request.urlopen(f'https://api.github.com/repos/openPupil/PyPupilEXT/contents/{name}?ref={REV}',timeout=30) as f:
                path.write_bytes(base64.b64decode(json.load(f)['content']))
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if name in expected and digest!=expected[name]:raise RuntimeError('Source checksum mismatch: '+name)
        expected[name]=digest
    manifest.write_text(json.dumps(expected,indent=2)+'\n')
    out=ROOT/'build/purest';out.mkdir(parents=True,exist_ok=True)
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True).split()
    cmd=['g++','-std=c++14','-O2','-shared','-fPIC','-include','opencv2/opencv.hpp','-I'+str(folder),
         str(Path(__file__).with_name('bridge.cpp')),*[str(folder/n) for n in NAMES if n.endswith('.cpp')],
         '-o',str(out/'libpurest.so'),*flags]
    subprocess.run(cmd,check=True)
    (out/'UPSTREAM.txt').write_text('https://github.com/openPupil/PyPupilEXT\n'+REV+'\nPuRe/PuReST: non-commercial research license; see source headers.\n')
    print(out/'libpurest.so')

if __name__=='__main__':main()
