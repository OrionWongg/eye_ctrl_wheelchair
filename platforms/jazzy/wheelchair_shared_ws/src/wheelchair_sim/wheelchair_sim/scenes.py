"""Unique geometry source for physical obstacles and simulation-only map (metres)."""
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import json
@dataclass(frozen=True)
class Box:
    name: str
    x: float
    y: float
    sx: float
    sy: float
    height: float = 1.5
    z_bottom: float = 0.0
    color: str = "0.7 0.35 0.15 1"
    def polygon(self):
        return [(self.x-self.sx/2,self.y-self.sy/2),(self.x+self.sx/2,self.y-self.sy/2),(self.x+self.sx/2,self.y+self.sy/2),(self.x-self.sx/2,self.y+self.sy/2)]
SCENES = ('turn_adjust_bay','turn_adjust_blocked','straight_wall','rotation_wall','narrow','slight_offset','both_blocked','dynamic','complex_indoor','clearpath_office','passage_080','passage_090','passage_100','passage_110','door_080','door_090','door_100','door_blocked_exit','passage_branch','side_door','intent_left','intent_two_left','intent_blocked_first','intent_narrow_turn','intent_behind','path_obstacle','uturn_right_clear')
BOUNDARY = [(-4.,-4.),(6.,-4.),(6.,4.),(-4.,4.)]
MESH_SCENES = ('clearpath_office', 'willow_office')
SCENES = (*SCENES, 'willow_office')
def obstacles(scene):
    if scene not in SCENES: raise ValueError('Unknown scene: '+scene)
    if scene in MESH_SCENES:return [] # Physical obstacles are one imported collision mesh.
    outer = [Box('west',-4.1,0,.2,8.4),Box('east',6.1,0,.2,8.4),Box('north',1,4.1,10,.2),Box('south',1,-4.1,10,.2)]
    if scene in ('turn_adjust_bay','turn_adjust_blocked'):
        result=outer+[Box('adjust_left',-.6,.49,.8,.12),Box('adjust_right',-.6,-.49,.8,.12)]
        if scene=='turn_adjust_blocked':result.append(Box('adjust_front',.85,0,.12,1.8))
        return result
    if scene=='uturn_right_clear':return outer+[Box('uturn_upper_obstacle',.05,.74,.7,.12)]
    if scene=='path_obstacle':return outer+[Box('path_obstacle',1.05,0.,.25,.35)]
    if scene.startswith('intent_'):
        width=.9 if scene=='intent_narrow_turn' else 1.4
        gaps=((-2.4,-1.),) if scene=='intent_behind' else ((.7,1.7),(2.,3.4)) if scene in ('intent_two_left','intent_blocked_first') else ((1.,2.4),)
        result=outer+[Box('intent_opposite',2.,-width/2-.06,8.,.12)]
        previous=-3.5
        for i,(start,end) in enumerate(gaps):
            result.append(Box('intent_wall_'+str(i),(previous+start)/2,width/2+.06,start-previous,.12))
            previous=end
        result.append(Box('intent_wall_end',(previous+6)/2,width/2+.06,6-previous,.12))
        if scene=='intent_blocked_first':result.append(Box('intent_first_blocker',1.2,1.,1.,.3))
        return result
    if scene=='side_door':
        return outer+[Box('continuous_left',2.5,.76,7.,.12),Box('right_before',0.,-.76,2.,.12),Box('right_after',4.2,-.76,3.6,.12)]
    if scene.startswith('passage_') and scene!='passage_branch':
        width=int(scene.split('_')[1])/100
        return outer+[Box('left_passage',2.5,width/2+.06,7,.12),Box('right_passage',2.5,-width/2-.06,7,.12)]
    if scene.startswith('door_'):
        width=.90 if scene=='door_blocked_exit' else int(scene.split('_')[1])/100
        result=outer+[Box('left_jamb',1.2,(4+width/2)/2,.30,4-width/2),Box('right_jamb',1.2,-(4+width/2)/2,.30,4-width/2)]
        if scene=='door_blocked_exit':result.append(Box('blocked_exit',2.,0,.15,2.))
        return result
    if scene=='passage_branch':
        return outer+[Box('left_before',.2,.51,2.4,.12),Box('right_before',.2,-.51,2.4,.12),Box('branch_end',2.2,0,.12,.9)]
    specific = {
      'straight_wall':[Box('front_wall',2,0,.2,4)],
      'rotation_wall':[Box('side_wall',0,.55,2,.12)],
      'narrow':[Box('left_corridor',2,.56,5,.2),Box('right_corridor',2,-.56,5,.2)],
      'slight_offset':[Box('offset_obstacle',1.8,.54,.22,.22,.28)],
      'both_blocked':[Box('front_wall',1.1,0,.2,2),Box('left_wall',0,.52,2,.2),Box('right_wall',0,-.52,2,.2)],
      'dynamic':[Box('moving_obstacle',2,2,.3,.6)],
      'complex_indoor':complex_furniture(),
    }
    return outer+specific[scene]
def map_obstacles(scene):
    if scene in MESH_SCENES:
        return [Box('office_map_'+str(i),(a+c)/2,(b+d)/2,c-a,d-b,1.26,.04)
                for i,(a,b,c,d) in enumerate(office_metadata(scene)['rectangles'])]
    boxes=obstacles(scene)
    # Conservatively reserve the complete motion corridor before moving geometry.
    if scene=='dynamic': boxes[-1]=Box('moving_obstacle_swept',2,0,.3,4.6)
    return boxes


def complex_furniture():
    """Indoor course: 1.8 m doorway, offset passages, furnishings and low obstacle.
    Each entry is an actual physical box and part of the collision oracle.
    """
    walls='0.60 0.69 0.76 1'
    wood='0.62 0.38 0.19 1'
    metal='0.24 0.30 0.34 1'
    blue='0.10 0.40 0.62 1'
    items=[
        Box('door_south',2.7,-2.45,.18,3.1,1.55,color=walls),
        Box('door_north',2.7,2.45,.18,3.1,1.55,color=walls),
        Box('offset_left',1.3,.80,.55,.45,1.40,color='0.85 0.53 0.12 1'),
        Box('offset_right',3.8,-.80,.55,.45,1.40,color='0.85 0.53 0.12 1'),
        Box('north_column',.55,2.45,.40,.40,1.8,color=walls),
        Box('south_column',4.6,-2.65,.40,.40,1.8,color=walls),
        Box('storage_west',-2.6,-2.95,1.6,.60,1.70,color=blue),
        Box('storage_east',4.6,2.95,1.6,.60,1.70,color=blue),
        Box('low_box',.65,-2.30,.65,.50,.25,color='0.72 0.24 0.15 1'),
        Box('small_crate',4.7,1.25,.60,.55,.65,color=wood),
        Box('desk_top',-2.,1.75,1.5,.85,.08,.74,wood),
    ]
    for i,(dx,dy) in enumerate([(-.62,-.30),(-.62,.30),(.62,-.30),(.62,.30)]):
        items.append(Box('desk_leg_'+str(i),-2.+dx,1.75+dy,.09,.09,.74,color=metal))
    for name,y,back_y in [('south',.75,.47),('north',2.75,3.03)]:
        items += [Box('chair_'+name+'_seat',-2.,y,.50,.50,.09,.43,blue),
                  Box('chair_'+name+'_back',-2.,back_y,.50,.08,.55,.48,blue),
                  Box('chair_'+name+'_base',-2.,y,.30,.30,.43,color=metal)]
    return items


@lru_cache(maxsize=2)
def office_metadata(scene='clearpath_office'):
    if scene not in MESH_SCENES:raise ValueError('Not a mesh scene: '+scene)
    directory=Path(__file__).resolve().parents[1]/'models'/scene
    if not directory.exists():
        from ament_index_python.packages import get_package_share_directory
        directory=Path(get_package_share_directory('wheelchair_sim'))/'models'/scene
    return json.loads((directory/'obstacle_map.json').read_text())

def boundary(scene):
    if scene not in MESH_SCENES:return BOUNDARY
    lo,hi=office_metadata(scene)['bounds']
    return [lo,[hi[0],lo[1]],hi,[lo[0],hi[1]]]
