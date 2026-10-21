#!/usr/bin/python3
"""User-local build headers for the native Gazebo lidar GUI plugin; no sudo."""
import apt_pkg,gzip,hashlib,json,subprocess
from pathlib import Path
apt_pkg.init()
workspace=Path(__file__).resolve().parents[1]
base=workspace/'vendor';records={}
(base/'debs').mkdir(parents=True,exist_ok=True)
for block in gzip.open(base/'gazebo-packages.gz','rt').read().split('\n\n'):
    if not block.strip():continue
    r=apt_pkg.TagSection(block+'\n\n');records[r['Package']]={k:r[k] for k in r.keys()}
names=['libignition-gazebo6-dev','libsdformat12-dev','libignition-cmake2-dev','libignition-common4-core-dev','libignition-common4-events-dev','libignition-common4-graphics-dev','libignition-math6-dev','libignition-msgs8-dev','libignition-transport11-core-dev','libignition-plugin-dev','libignition-utils1-dev','libprotobuf-dev','qtquickcontrols2-5-dev']
manifest={}
for name in names:
    if name in records:
        r=records[name];origin='https://packages.osrfoundation.org/gazebo/ubuntu-stable/'
    else:
        out=subprocess.check_output(['apt-cache','show',name],text=True)
        tag=apt_pkg.TagSection(out.split('\n\n')[0]+'\n\n');r={k:tag[k] for k in tag.keys()};origin='https://ports.ubuntu.com/ubuntu-ports/'
    target=base/'debs'/Path(r['Filename']).name
    if not target.exists() or hashlib.sha256(target.read_bytes()).hexdigest()!=r['SHA256']:
        subprocess.run(['curl','-fsSL','--retry','2',origin+r['Filename'],'-o',str(target)],check=True)
    assert hashlib.sha256(target.read_bytes()).hexdigest()==r['SHA256']
    subprocess.run(['dpkg-deb','-x',str(target),str(base/'runtime')],check=True)
    manifest[name]={'version':r['Version'],'url':origin+r['Filename'],'sha256':r['SHA256']}
    print(name,flush=True)
(workspace/'artifacts/scan-viz-build-dependencies.json').write_text(json.dumps(manifest,indent=2)+'\n')
