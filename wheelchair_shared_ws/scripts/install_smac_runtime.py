#!/usr/bin/env python3
"""Verified workspace-local Humble Smac/OMPL runtime; leaves the default Nav2 intact."""
import gzip
import hashlib
import json
import re
import subprocess
import tempfile
import urllib.request
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'vendor/smac'
BASE='http://packages.ros.org/ros2/ubuntu/'
PACKAGES=('ros-humble-nav2-smac-planner','ros-humble-ompl')


def fetch(url):return urllib.request.urlopen(url,timeout=60).read()


def main():
    arch=subprocess.check_output(['dpkg','--print-architecture'],text=True).strip()
    with tempfile.TemporaryDirectory(prefix='wheelchair-smac-') as tmp:
        tmp=Path(tmp)
        pinned_key=ROOT.parent/'nav2/ros.key'
        key=pinned_key.read_bytes() if pinned_key.exists() else fetch('https://raw.githubusercontent.com/ros/rosdistro/master/ros.key')
        (tmp/'ros.key').write_bytes(key)
        subprocess.run(['gpg','--batch','--yes','--dearmor','--output',str(tmp/'ros.gpg'),str(tmp/'ros.key')],check=True)
        release=fetch(BASE+'dists/jammy/InRelease');(tmp/'InRelease').write_bytes(release)
        subprocess.run(['gpgv','--keyring',str(tmp/'ros.gpg'),str(tmp/'InRelease')],check=True)
        index=f'main/binary-{arch}/Packages.gz'
        match=re.search(r'^ ([a-f0-9]{64})\s+(\d+)\s+'+re.escape(index)+r'$',release.decode(),re.M)
        if not match:raise RuntimeError('Signed package index missing')
        payload=fetch(BASE+'dists/jammy/'+index)
        if len(payload)!=int(match[2]) or hashlib.sha256(payload).hexdigest()!=match[1]:raise RuntimeError('Index hash mismatch')
        packages={}
        for block in gzip.decompress(payload).decode().split('\n\n'):
            row=dict(line.split(': ',1) for line in block.splitlines() if ': ' in line and not line.startswith(' '))
            if row.get('Package') in PACKAGES:packages[row['Package']]=row
        if not packages['ros-humble-nav2-smac-planner']['Version'].startswith('1.1.20-'):
            raise RuntimeError('Review ABI before changing the pinned Nav2 1.1.20 runtime')
        ROOT.mkdir(parents=True,exist_ok=True);manifest=[]
        for name in PACKAGES:
            row=packages[name];data=fetch(BASE+row['Filename'])
            if hashlib.sha256(data).hexdigest()!=row['SHA256']:raise RuntimeError('Package hash mismatch: '+name)
            deb=tmp/(name+'.deb');deb.write_bytes(data)
            subprocess.run(['dpkg-deb','-x',str(deb),str(ROOT)],check=True)
            manifest.append({k:row[k] for k in ('Package','Version','SHA256','Filename')})
            print('Installed locally:',name,row['Version'],flush=True)
        (ROOT/'COLCON_IGNORE').touch();(ROOT/'InRelease').write_bytes(release)
        (ROOT/'manifest.json').write_text(json.dumps(dict(repository=BASE,architecture=arch,packages=manifest),indent=2)+'\n')
        (ROOT/'ros.key').write_bytes(key)


if __name__=='__main__':main()
