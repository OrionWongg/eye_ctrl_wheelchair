#!/usr/bin/python3
"""Import BSD Clearpath Office; derive conservative map from its real mesh.
Flat-floor wheelchair test: keep collision surfaces in z=[0.04, 1.30] m.
Source vertices/transforms are preserved; original textured visual is unchanged.
"""
import hashlib,json,shutil,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import cv2
ROOT=Path(__file__).resolve().parents[1]
UP=ROOT/'vendor/cpr_gazebo/cpr_office_gazebo'
OUT=ROOT/'src/wheelchair_sim/models/clearpath_office'
OUT.mkdir(parents=True,exist_ok=True)
ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
r=ET.parse(UP/'meshes/office.dae').getroot()
assert r.findtext('c:asset/c:up_axis',namespaces=ns)=='Z_UP'
assert float(r.find('c:asset/c:unit',ns).get('meter'))==1.
meshes={}
for g in r.findall('c:library_geometries/c:geometry',ns):
    mesh=g.find('c:mesh',ns);sources={}
    for s in mesh.findall('c:source',ns):
        accessor=s.find('c:technique_common/c:accessor',ns)
        if s.find('c:float_array',ns) is not None:
            sources[s.get('id')]=np.fromstring(s.findtext('c:float_array',namespaces=ns),sep=' ').reshape(-1,int(accessor.get('stride','1')))
    vertices={v.get('id'):v.find("c:input[@semantic='POSITION']",ns).get('source')[1:] for v in mesh.findall('c:vertices',ns)}
    faces=[]
    assert not mesh.findall('c:polylist',ns),'Unexpected mesh primitive'
    for t in mesh.findall('c:triangles',ns):
        inputs=t.findall('c:input',ns);stride=1+max(int(i.get('offset','0')) for i in inputs)
        v=next(i for i in inputs if i.get('semantic')=='VERTEX')
        indices=np.fromstring(t.findtext('c:p',namespaces=ns),sep=' ',dtype=int).reshape(-1,3,stride)[:,:,int(v.get('offset','0'))]
        faces.append(sources[vertices[v.get('source')[1:]]][indices,:3])
    meshes[g.get('id')]=np.concatenate(faces) if faces else np.empty((0,3,3))
all_faces=[]
def walk(node,parent):
    transform=parent.copy()
    for m in node.findall('c:matrix',ns):
        local=np.fromstring(m.text,sep=' ').reshape(4,4)
        assert np.allclose(local[3],[0,0,0,1]),'Unsupported matrix convention'
        transform=transform@local
    assert not node.findall('c:translate',ns)+node.findall('c:rotate',ns)+node.findall('c:scale',ns)
    for instance in node.findall('c:instance_geometry',ns):
        xyz=meshes[instance.get('url')[1:]]
        world=xyz@transform[:3,:3].T+transform[:3,3]
        all_faces.extend(world)
    for child in node.findall('c:node',ns):walk(child,transform)
for node in r.findall('c:library_visual_scenes/c:visual_scene/c:node',ns):walk(node,np.eye(4))
all_faces=np.array(all_faces)
def clip(poly,height,above):
    result=[]
    for a,b in zip(poly,poly[1:]+poly[:1]):
        ia=a[2]>=height if above else a[2]<=height
        ib=b[2]>=height if above else b[2]<=height
        if ia:result.append(a)
        if ia!=ib:result.append(a+(b-a)*((height-a[2])/(b[2]-a[2])))
    return result
polys=[];triangles=[]
for tri in all_faces:
    poly=clip(list(tri),.04,True)
    if len(poly)<3:continue
    poly=clip(poly,1.30,False)
    if len(poly)<3:continue
    polys.append(np.array(poly))
    for i in range(1,len(poly)-1):triangles.append([poly[0],poly[i],poly[i+1]])
lo=np.floor(all_faces[:,:,:2].min(axis=(0,1)))-1
hi=np.ceil(all_faces[:,:,:2].max(axis=(0,1)))+1
res=.05
shape=np.ceil((hi-lo)/res).astype(int)
grid=np.zeros((shape[1],shape[0]),np.uint8)
for poly in polys:
    # Include every projected surface, including degenerate vertical faces.
    points=np.floor((poly[:,:2]-lo)/res).astype(np.int32)
    cv2.fillConvexPoly(grid,points,1)
# Cover cells crossed by oblique edges despite raster rounding.
grid=cv2.dilate(grid,np.ones((3,3),np.uint8))
rectangles=[];active={}
for row in range(grid.shape[0]+1):
    values=grid[row] if row<grid.shape[0] else np.zeros(grid.shape[1],np.uint8)
    delta=np.diff(np.pad(values.astype(int),(1,1)))
    runs=set(zip(np.flatnonzero(delta==1),np.flatnonzero(delta==-1)))
    for run,start in list(active.items()):
        if run not in runs:
            x0,x1=run;rectangles.append([float(lo[0]+x0*res),float(lo[1]+start*res),float(lo[0]+x1*res),float(lo[1]+row*res)]);del active[run]
    for run in runs:
        if run not in active:active[run]=row
# Keep original model and texture license/provenance beside adapted collision mesh.
(OUT/'meshes').mkdir(exist_ok=True)
for filename in ['office.dae']+[Path(e.text).name for e in r.findall('c:library_images/c:image/c:init_from',ns)]:
    shutil.copy2(UP/'meshes'/filename,OUT/'meshes'/filename)
shutil.copy2(UP/'docs/LICENSE.txt',OUT/'LICENSE.txt')
triangles_array=np.array(triangles)
normals=np.cross(triangles_array[:,1]-triangles_array[:,0],triangles_array[:,2]-triangles_array[:,0])
lengths=np.linalg.norm(normals,axis=1)
valid=lengths>1e-10
triangles_array=triangles_array[valid];normals=normals[valid]/lengths[valid,None]
triangles=list(triangles_array)
verts=triangles_array.reshape(-1,3)
normals=np.repeat(normals,3,axis=0)
collada=f'''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset><library_geometries><geometry id="office"><mesh><source id="positions"><float_array id="positions-array" count="{verts.size}">{' '.join(format(v,'.9g') for v in verts.flat)}</float_array><technique_common><accessor source="#positions-array" count="{len(verts)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices><triangles count="{len(triangles)}"><input semantic="VERTEX" source="#vertices" offset="0"/><p>{' '.join(map(str,range(len(verts))))}</p></triangles></mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node id="office"><instance_geometry url="#office"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>'''
normal_source=f'<source id="normals"><float_array id="normals-array" count="{normals.size}">'+ ' '.join(format(v,'.9g') for v in normals.flat)+f'</float_array><technique_common><accessor source="#normals-array" count="{len(normals)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>'
collada=collada.replace('<vertices id="vertices">',normal_source+'<vertices id="vertices">')
collada=collada.replace('source="#vertices" offset="0"/><p>', 'source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><p>')
indices=' '.join(map(str,range(len(verts))))
collada=collada.replace('<p>'+indices+'</p>','<p>'+' '.join(str(i) for i in range(len(verts)) for _ in range(2))+'</p>')
(OUT/'meshes/office_collision.dae').write_text(collada)
(OUT/'model.config').write_text('<model><name>Clearpath Office (Fortress adapter)</name><version>1.0</version><sdf version="1.8">model.sdf</sdf><author><name>Clearpath Robotics</name></author><description>BSD licensed Clearpath Office; wheelchair collision-height adaptation.</description></model>')
(OUT/'model.sdf').write_text('''<?xml version="1.0"?><sdf version="1.8"><model name="clearpath_office"><static>true</static><link name="office"><visual name="original_visual"><geometry><mesh><uri>model://clearpath_office/meshes/office.dae</uri></mesh></geometry><cast_shadows>false</cast_shadows></visual><collision name="office_collision"><geometry><mesh><uri>model://clearpath_office/meshes/office_collision.dae</uri></mesh></geometry></collision><sensor name="contact" type="contact"><always_on>true</always_on><update_rate>20</update_rate><contact><collision>office_collision</collision></contact></sensor></link></model></sdf>''')
metadata={'source':'https://github.com/clearpathrobotics/cpr_gazebo','commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=UP,text=True).strip(),'license':'BSD-3-Clause','source_mesh_sha256':hashlib.sha256((UP/'meshes/office.dae').read_bytes()).hexdigest(),'bounds':[lo.tolist(),hi.tolist()],'mesh_dimensions_m':(all_faces.max(axis=(0,1))-all_faces.min(axis=(0,1))).tolist(),'resolution_m':res,'height_band_m':[.04,1.30],'raster_dilation_cells':1,'source_triangles':len(all_faces),'collision_triangles':len(triangles),'rectangles':rectangles}
(OUT/'obstacle_map.json').write_text(json.dumps(metadata,indent=2)+'\n')
cv2.imwrite(str(ROOT/'artifacts/office-occupancy.png'),255-grid[::-1]*255)
print({k:v for k,v in metadata.items() if k!='rectangles'})
print('Map rectangles:',len(rectangles))
