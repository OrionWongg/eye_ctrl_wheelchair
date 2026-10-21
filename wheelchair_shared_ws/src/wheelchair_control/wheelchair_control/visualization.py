"""Decision-sequenced RViz overlays; every frame replaces the previous markers."""
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from shapely.geometry import Polygon
from .geometry import transform
import math

def safety_markers(stamp,sequence,state,cfg,diagnostics):
    output=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;output.markers.append(clear)
    def line(name,points,color,width=.012):
        m=Marker();m.header.frame_id='odom';m.header.stamp=stamp;m.ns=name;m.id=len(output.markers)
        m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=width
        m.color.r,m.color.g,m.color.b,m.color.a=color
        m.points=[Point(x=float(p[0]),y=float(p[1]),z=.04) for p in points]
        m.lifetime.sec=1;output.markers.append(m)
    footprint=transform(list(cfg.footprint),state.pose)
    for name,polygon,color in [('sim_body',Polygon(transform([(-.45,-.30),(.55,-.30),(.55,.30),(-.45,.30)],state.pose)),(.2,.8,.2,1.)),
            ('planning_footprint',Polygon(footprint),(.3,.8,.5,.7)),
            ('hard',Polygon(footprint).buffer(cfg.padding),(1.,.2,.1,1.)),
            ('comfort',Polygon(footprint).buffer(max(cfg.padding,cfg.comfort_distance)),(.8,.8,.2,.7))]:
        line(name,list(polygon.exterior.coords),color)
    candidate=diagnostics.get('candidate',{})
    poses=candidate.get('poses',[])
    for pose in poses[::max(1,len(poses)//15)]:
        outline=transform(list(cfg.footprint),pose);line('sweep',outline+outline[:1],(.1,.5,1.,.25))
    if poses:
        outline=transform(list(cfg.footprint),poses[-1]);line('predicted_stop' if candidate.get('safe') else 'prediction_end',outline+outline[:1],(.8,.1,.8,1.))
    failure=candidate.get('failure',diagnostics.get('current_geometry',{}))
    if 'body_point' in failure:line('first_violation',[failure['body_point'],failure['obstacle_point']],(1.,0.,0.,1.),.045)
    p=diagnostics.get('passage')
    if p:
        c,s=math.cos(p['heading']),math.sin(p['heading']);o=p['origin']
        points=[]
        for x,y in [(p['entry'],p['center']-p['width']/2),(p['end'],p['center']-p['width']/2),
                    (p['end'],p['center']+p['width']/2),(p['entry'],p['center']+p['width']/2)]:
            points.append((o[0]+c*x-s*y,o[1]+s*x+c*y))
        line('passage_band',points+points[:1],(.1,.9,.9,1.))
    text=Marker();text.header.frame_id='odom';text.header.stamp=stamp;text.ns='decision';text.id=1000
    text.type=Marker.TEXT_VIEW_FACING;text.action=Marker.ADD;text.pose.position.x=state.x;text.pose.position.y=state.y;text.pose.position.z=1.2
    text.pose.orientation.w=1.;text.scale.z=.12;text.color.r=text.color.g=text.color.b=text.color.a=1.
    text.text=f'decision {sequence}';text.lifetime.sec=1;output.markers.append(text)
    return output
