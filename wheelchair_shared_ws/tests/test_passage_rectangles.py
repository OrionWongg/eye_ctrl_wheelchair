"""Differential check against the original GEOS-union passage detector."""
import math
import random
import pytest
from wheelchair_control import passage
from wheelchair_control.geometry import IndexedObstacles
from wheelchair_control.planner import Config, State


def box(a,b,c,d):return [(a,c),(b,c),(b,d),(a,d)]


@pytest.mark.parametrize('heading',[0., math.pi/2, math.pi, -math.pi/2])
def test_rectangle_detector_matches_general_polygon_union(monkeypatch,heading):
    rng=random.Random(32)
    cfg=Config(mode='corridor_assist')
    cases=[[],[box(-2,5,.45,.6),box(-2,5,-.6,-.45)],
           [box(0,.3,.45,3),box(0,.3,-3,-.45)],
           [box(-2,1,.45,.6),box(1.5,5,.45,.6),box(-2,5,-.6,-.45)],
           [box(-1,1,-.2,.2),box(0,2,-.2,.2)],
           [[(0,0),(1,0),(.5,.2)]]]
    for _ in range(30):
        shapes=[box(-2,5,.45,.6),box(-2,5,-.6,-.45)]
        for _ in range(40):
            x=rng.uniform(-4,4);y=rng.uniform(-4,4)
            shapes.append(box(x,x+rng.uniform(.02,.5),y,y+rng.uniform(.02,.5)))
        cases.append(shapes)
    fast=passage.rectangle_sections
    for shapes in cases:
        obstacles=IndexedObstacles(shapes)
        state=State(x=.017,y=-.013,yaw=heading+.01)
        monkeypatch.setattr(passage,'rectangle_sections',fast)
        actual,why=passage.detect(state,obstacles,cfg)
        monkeypatch.setattr(passage,'rectangle_sections',lambda *args:None)
        expected,reason=passage.detect(state,obstacles,cfg)
        assert why==reason
        assert (actual is None)==(expected is None)
        if expected:
            for key,value in expected.as_dict().items():
                assert actual.as_dict()[key]==(pytest.approx(value,abs=1e-9) if isinstance(value,(float,tuple)) else value)
            # Compare extension of a previously locked corridor as well.
            monkeypatch.setattr(passage,'rectangle_sections',fast)
            locked_actual,why=passage.detect(state,obstacles,cfg,actual)
            monkeypatch.setattr(passage,'rectangle_sections',lambda *args:None)
            locked_expected,reason=passage.detect(state,obstacles,cfg,expected)
            assert why==reason
            assert (locked_actual is None)==(locked_expected is None)
