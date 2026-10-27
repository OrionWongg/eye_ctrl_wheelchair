"""Decision-sequenced RViz overlays; every frame replaces the previous markers."""
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from shapely.geometry import Polygon
from .geometry import transform

def safety_markers(stamp,sequence,state,cfg,diagnostics):
    output=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;output.markers.append(clear)
    def line(name,points,color,width=.012):
        m=Marker();m.header.frame_id='odom';m.header.stamp=stamp;m.ns=name;m.id=len(output.markers)
        m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=width
        m.color.r,m.color.g,m.color.b,m.color.a=color
        m.points=[Point(x=float(p[0]),y=float(p[1]),z=.04) for p in points]
        m.lifetime.sec=1;output.markers.append(m)
    # The only safety outline: footprint + margin used by the braking check.
    footprint=transform(list(cfg.footprint),state.pose)
    line('hard',list(Polygon(footprint).buffer(cfg.padding).exterior.coords),(1.,.2,.1,1.))
    text=Marker();text.header.frame_id='odom';text.header.stamp=stamp;text.ns='decision';text.id=1000
    text.type=Marker.TEXT_VIEW_FACING;text.action=Marker.ADD;text.pose.position.x=state.x;text.pose.position.y=state.y;text.pose.position.z=1.2
    text.pose.orientation.w=1.;text.scale.z=.12;text.color.r=text.color.g=text.color.b=text.color.a=1.
    text.text=f'decision {sequence}';text.lifetime.sec=1;output.markers.append(text)
    return output
