from setuptools import setup
from glob import glob
from pathlib import Path
model_data=[]
for folder in [Path('models'),*Path('models').rglob('*')]:
    if folder.is_dir():
        files=[str(p) for p in folder.iterdir() if p.is_file()]
        if files:model_data.append(('share/wheelchair_sim/'+str(folder),files))
setup(name='wheelchair_sim',version='0.1.0',packages=['wheelchair_sim'],data_files=[('share/ament_index/resource_index/packages',['resource/wheelchair_sim']),('share/wheelchair_sim',['package.xml','SIM_README.md']),('share/wheelchair_sim/launch',glob('launch/*.py')),('share/wheelchair_sim/config',glob('config/*'))]+model_data,entry_points={'console_scripts':['obstacle_map = wheelchair_sim.map_node:main','drive_watchdog = wheelchair_sim.watchdog:main','first_person_view = wheelchair_sim.first_person_view:main']})
