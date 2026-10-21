#!/usr/bin/python3
"""Read-only audit of physical mesh projections versus the conservative office map."""
import json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import shapely
from shapely.geometry import Polygon,LineString
from wheelchair_control.planner import Config
from wheelchair_sim.scenes import map_obstacles
root=Path(__file__).resolve().parents[1]
mesh=root/'src/wheelchair_sim/models/clearpath_office/meshes/office_collision.dae'
ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
r=ET.parse(mesh).getroot()
vertices=np.fromstring(r.find(".//c:source[@id='positions']/c:float_array",ns).text,sep=' ').reshape(-1,3)
shapes=[]
for tri in vertices.reshape(-1,3,3):
    xy=tri[:,:2];p=Polygon(xy)
    # Vertical surfaces project to lines; retaining them matters for doorway sides.
    shapes.append(p if p.area>1e-12 else LineString(xy))
physical=shapely.make_valid(shapely.union_all([shapely.make_valid(p) for p in shapes]))
raster=shapely.union_all([Polygon(b.polygon()) for b in map_obstacles('clearpath_office')])
def boundaries(shape,x,y):
    hit=shape.intersection(LineString([(x,y-2),(x,y+2)]))
    parts=list(hit.geoms) if hasattr(hit,'geoms') else [hit];intervals=[]
    for p in parts:
        if p.is_empty or not hasattr(p,'coords'):continue
        yy=[q[1] for q in p.coords];intervals.append((min(yy),max(yy)))
    above=min((a for a,b in intervals if a>y),default=None)
    below=max((b for a,b in intervals if b<y),default=None)
    return dict(lower=below,upper=above,width=None if above is None or below is None else above-below)
rows=[]
for x in [2.8,3.0,3.5,4.0,4.5]:
    rows.append(dict(x=x,y=1.,collision_mesh=boundaries(physical,x,1.),map=boundaries(raster,x,1.)))
old=Config();new=Config(footprint=((-0.45,-.30),(.55,-.30),(.55,.30),(-.45,.30)),margin=.06)
report={'physical_envelope_m':[1.,.60,.85],'old_straight_hard_width_m':.70+2*old.padding,
        'new_straight_hard_width_m':.60+2*new.padding,'new_hard_padding_m':new.padding,
        'office_cross_sections':rows,'map_modified':False,
        'note':'Projected physical surfaces in the existing collision height band. Widths are cross sections, not a guarantee of swept passage feasibility.'}
(root/'artifacts/narrow-geometry-audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
