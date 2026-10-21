"""Vectorized bounds must preserve scalar collision decisions and safe clearance."""
import math
import random
import pytest
from wheelchair_control.geometry import IndexedObstacles,clearance,transform

@pytest.mark.parametrize('seed',[19,41,73])
def test_indexed_matches_scalar_for_rotated_footprints_and_convex_obstacles(seed):
    rng=random.Random(seed);boundary=[(-10,-10),(10,-10),(10,10),(-10,10)]
    obstacles=[]
    for i in range(40):
        w,h=rng.uniform(.03,2),rng.uniform(.03,2)
        shape=[(-w,-h),(w,-h),(w,h),(-w,h)] if i%2 else [(-w,-h),(w,-h),(0,h)]
        obstacles.append(transform(shape,(rng.uniform(-8,8),rng.uniform(-8,8),rng.uniform(-math.pi,math.pi))))
    indexed=IndexedObstacles(obstacles)
    for _ in range(100):
        poly=transform([(-.45,-.35),(.55,-.35),(.55,.35),(-.45,.35)],(rng.uniform(-9,9),rng.uniform(-9,9),rng.uniform(-math.pi,math.pi)))
        scalar=clearance(poly,obstacles,boundary,.09);fast=clearance(poly,indexed,boundary,.09)
        assert (scalar>.09)==(fast>.09)
        if scalar>.09:assert fast==pytest.approx(scalar,abs=1e-10)

@pytest.mark.parametrize('gap',[0.,.09-1e-11,.09,.09+1e-11,1.])
def test_touching_and_margin_threshold(gap):
    poly=[(-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5)]
    obs=[[(.5+gap,-2),(2+gap,-2),(2+gap,2),(.5+gap,2)]]
    boundary=[(-10,-10),(10,-10),(10,10),(-10,10)]
    a=clearance(poly,obs,boundary,.09);b=clearance(poly,IndexedObstacles(obs),boundary,.09)
    assert a==pytest.approx(b,abs=1e-12)
    assert (a>.09)==(b>.09)
