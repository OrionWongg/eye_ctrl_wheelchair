#!/usr/bin/env python3
"""Install a verified, workspace-local subset of ROS Humble/Nav2 (no sudo)."""
import gzip,hashlib,json,re,subprocess,tempfile,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'vendor/nav2'
BASE='http://packages.ros.org/ros2/ubuntu/'
ARCH=subprocess.check_output(['dpkg','--print-architecture'],text=True).strip()
def fetch(url):return urllib.request.urlopen(url,timeout=60).read()
def main(requested=None):
    ROOT.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='wheelchair-nav2-') as tmp:
        temp=Path(tmp)
        key=fetch('https://raw.githubusercontent.com/ros/rosdistro/master/ros.key')
        (temp/'ros.key').write_bytes(key)
        subprocess.run(['gpg','--batch','--yes','--dearmor','--output',str(temp/'ros.gpg'),str(temp/'ros.key')],check=True)
        release=fetch(BASE+'dists/jammy/InRelease');(temp/'InRelease').write_bytes(release)
        subprocess.run(['gpgv','--keyring',str(temp/'ros.gpg'),str(temp/'InRelease')],check=True)
        relpath=f'main/binary-{ARCH}/Packages.gz'
        match=re.search(r'^ ([a-f0-9]{64})\s+(\d+)\s+'+re.escape(relpath)+r'$',release.decode(),re.M)
        if not match:raise RuntimeError('No signed package index hash')
        data=fetch(BASE+'dists/jammy/'+relpath)
        assert len(data)==int(match[2]) and hashlib.sha256(data).hexdigest()==match[1]
        packages={}
        for block in gzip.decompress(data).decode().split('\n\n'):
            fields=dict(line.split(': ',1) for line in block.splitlines() if ': ' in line and not line.startswith(' '))
            if 'Package' in fields:packages[fields['Package']]=fields
        installed={}
        for row in subprocess.check_output(['dpkg-query','-W','-f=${Package}\t${Version}\t${Status}\n'],text=True).splitlines():
            name,version,status=row.split('\t',2)
            if status=='install ok installed':installed[name]=version
        queue=list(requested or ['ros-humble-dwb-core','ros-humble-nav2-costmap-2d','ros-humble-nav2-lifecycle-manager'])
        selected={}
        while queue:
            name=queue.pop()
            if name in selected:continue
            entry=packages[name];selected[name]=entry
            for dependency in entry.get('Depends','').split(','):
                alternative=dependency.strip().split(' | ')[0]
                m=re.match(r'([a-z0-9+.-]+)(?: \((>=|=) ([^)]+)\))?',alternative)
                if not m:raise RuntimeError('Unsupported dependency: '+dependency)
                dep,op,ver=m.groups()
                present=dep in installed and (not op or subprocess.run(['dpkg','--compare-versions',installed[dep],op,ver]).returncode==0)
                if present:continue
                if not dep.startswith('ros-humble-'):raise RuntimeError('Missing system dependency: '+alternative)
                queue.append(dep)
        manifest=[]
        for name,entry in sorted(selected.items()):
            payload=fetch(BASE+entry['Filename'])
            assert hashlib.sha256(payload).hexdigest()==entry['SHA256'],name
            dest=temp/(name+'.deb');dest.write_bytes(payload)
            subprocess.run(['dpkg-deb','-x',str(dest),str(ROOT)],check=True)
            manifest.append({k:entry[k] for k in ('Package','Version','SHA256','Filename')})
            print('Installed locally:',name,flush=True)
        (ROOT/'COLCON_IGNORE').touch()
        (ROOT/'manifest.json').write_text(json.dumps(dict(repository=BASE,architecture=ARCH,packages=manifest),indent=2)+'\n')
        (ROOT/'InRelease').write_bytes(release)
        (ROOT/'ros.key').write_bytes(key)
        relocate()

def relocate():
    # ROS binary CMake exports include absolute paths to dependency headers.
    # Relocate only paths actually provided by this local subset, retaining
    # system-underlay references for already installed dependencies.
    prefix=ROOT/'opt/ros/humble'
    for path in prefix.glob('share/*/cmake/*.cmake'):
        source=path.read_text()
        def local(match):
            original=match[0];candidate=prefix/original.removeprefix('/opt/ros/humble/')
            return str(candidate) if candidate.exists() else original
        path.write_text(re.sub(r'/opt/ros/humble/[^;"\s)]+',local,source))
if __name__=='__main__':main()
