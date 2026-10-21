"""Map-derived rectangular wall openings. No scene names or door coordinates.

First release supports orthogonal straight corridors, like the passage detector.
Unknown/non-orthogonal topology is not interpreted as a reachable doorway.
"""
from dataclasses import dataclass
import hashlib
import math
import shapely
from shapely.affinity import translate
from shapely.geometry import Polygon, MultiPoint
from .geometry import transform, wrap


@dataclass(frozen=True)
class Entry:
    identity: str
    side: int
    heading: float
    start: float
    end: float
    inner: float
    outer: float
    approach: tuple
    exit: tuple


def coordinates(x, y, heading):
    c, s = math.cos(heading), math.sin(heading)
    return x*c+y*s, -x*s+y*c


def world(x, y, heading):
    c, s = math.cos(heading), math.sin(heading)
    return x*c-y*s, x*s+y*c


def openings(state, obstacles, passage, side, cfg):
    if passage is None:
        return []
    h = passage.heading
    along, lateral = coordinates(state.x, state.y, h)
    _, origin_lateral = coordinates(*passage.origin, h)
    center = origin_lateral+passage.center
    wall = center+side*passage.width/2
    intervals = []
    for obstacle in obstacles:
        points = [coordinates(x, y, h) for x, y in obstacle]
        xs, ys = zip(*points)
        lo, hi, bottom, top = min(xs), max(xs), min(ys), max(ys)
        # Reject nonrectangular/sloping walls instead of using their AABB as fact.
        if any(min(abs(x-lo), abs(x-hi)) > .005 or min(abs(y-bottom), abs(y-top)) > .005 for x,y in points):
            continue
        inner, outer = (bottom, top) if side > 0 else (top, bottom)
        if abs(inner-wall) < .03:
            intervals.append((lo, hi, outer))
    merged = []
    for lo, hi, outer in sorted(intervals):
        if merged and lo <= merged[-1][1]+.01:
            a,b,o = merged[-1]
            merged[-1] = (a,max(b,hi),max(o,outer) if side>0 else min(o,outer))
        else:
            merged.append((lo,hi,outer))
    result = []
    for before, after in zip(merged, merged[1:]):
        start,end = before[1],after[0]
        middle = (start+end)/2
        if not 0 <= middle-along <= cfg.entry_lookahead:
            continue
        if end-start <= 2*(max(abs(y) for x,y in cfg.footprint)+cfg.padding+cfg.nav_position_tolerance):
            continue
        outer = max(before[2],after[2]) if side>0 else min(before[2],after[2])
        # Retain the current corridor track; passage assistance may center it en route.
        approach = (*world(middle, center, h), h)
        target_heading = wrap(h+side*math.pi/2)
        rear = -min(x for x,y in cfg.footprint)
        exit_lateral = outer+side*(rear+cfg.padding+cfg.entry_exit_clearance)
        exit_pose = (*world(middle, exit_lateral, h), target_heading)
        endpoints = sorted(tuple(round(v,4) for v in world(x,wall,h)) for x in (start,end))
        identity = hashlib.sha256(repr(endpoints).encode()).hexdigest()[:16]
        result.append(Entry(identity,side,h,start,end,wall,outer,approach,exit_pose))
    return sorted(result,key=lambda e:(round((e.start+e.end)/2-along,4),e.identity))


def route_clear(state, entry, obstacles, boundary, cfg, reserve_position=True):
    """Conservative complete straight/rotate/straight sweep including exit stop room.

Sampled rotation hulls are inflated by the maximum circular chord deviation.
Execution still checks the real momentum/lease-loss envelope each cycle.
"""
    a = entry.approach
    rotation = [(*a[:2],a[2]+entry.side*math.pi/2*i/45) for i in range(46)]
    extra_steps=max(1,math.ceil(cfg.nav_turn_envelope/math.radians(2)))
    overshoot=[(*a[:2],rotation[-1][2]+entry.side*cfg.nav_turn_envelope*i/extra_steps)
               for i in range(1,extra_steps+1)]
    forward_exit = cfg.forward_speed*(cfg.reaction+cfg.action_ttl+cfg.transport_delay+cfg.period)
    forward_exit += cfg.forward_speed**2/(2*cfg.linear_brake)
    end = (entry.exit[0]+math.cos(entry.exit[2])*forward_exit,
           entry.exit[1]+math.sin(entry.exit[2])*forward_exit,entry.exit[2])
    approach = MultiPoint(transform(cfg.footprint,state.pose)+transform(cfg.footprint,a)).convex_hull
    turn = [MultiPoint(transform(cfg.footprint,p)+transform(cfg.footprint,q)).convex_hull
            for p,q in zip(rotation+overshoot,(rotation+overshoot)[1:])]
    depart = MultiPoint(transform(cfg.footprint,rotation[-1])+transform(cfg.footprint,end)).convex_hull
    sweep = shapely.union_all([approach,depart,*turn])
    # Stopping tolerance is along the corridor, not a lateral map uncertainty.
    tolerance=cfg.nav_position_tolerance if reserve_position else 0.
    dx=tolerance*math.cos(entry.heading)
    dy=tolerance*math.sin(entry.heading)
    sweep=shapely.union_all([sweep,translate(sweep,dx,dy),translate(sweep,-dx,-dy)]).buffer(
        cfg.padding+cfg.radius*(1-math.cos(math.pi/180)))
    known = Polygon(boundary)
    if not known.covers(sweep):
        return False
    return not any(sweep.intersects(getattr(o,'native',None) or Polygon(o)) for o in obstacles)
