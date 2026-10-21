"""The experimental control set must be executable without reverse or arcs."""
import importlib.util
import json
import math
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[1]
DATA=json.loads((ROOT/'src/wheelchair_nav2/config/wheelchair_lattice.json').read_text())


def test_checked_in_lattice_is_reproducible():
    spec=importlib.util.spec_from_file_location('wheelchair_lattice_generator',ROOT/'scripts/generate_wheelchair_lattice.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assert module.generate()==DATA


def test_lattice_is_connected_and_has_both_in_place_turn_directions():
    m=DATA['lattice_metadata'];angles=m['heading_angles'];n=len(angles)
    assert n==16 and len(DATA['primitives'])==m['number_of_trajectories']
    for i in range(n):
        turns=[p for p in DATA['primitives'] if p['start_angle_index']==i and p['trajectory_length']==0]
        assert {p['end_angle_index'] for p in turns}=={(i-1)%n,(i+1)%n}
        for p in turns:
            assert all(x==0 and y==0 for x,y,h in p['poses'])
            last=p['poses'][-1][2];expected=angles[p['end_angle_index']]
            assert abs(math.atan2(math.sin(last-expected),math.cos(last-expected)))<1e-9


def test_all_translations_are_forward_straight_and_land_on_the_costmap_grid():
    m=DATA['lattice_metadata'];resolution=m['grid_resolution']
    for p in DATA['primitives']:
        if p['trajectory_length']==0:continue
        heading=m['heading_angles'][p['start_angle_index']]
        assert p['start_angle_index']==p['end_angle_index'] and p['arc_length']==0
        previous=(0,0,heading)
        for point in p['poses']:
            dx,dy=point[0]-previous[0],point[1]-previous[1]
            assert dx*math.cos(heading)+dy*math.sin(heading)>0
            assert abs(-dx*math.sin(heading)+dy*math.cos(heading))<1e-10
            assert point[2]==heading
            previous=point
        assert math.hypot(previous[0],previous[1])==pytest.approx(p['trajectory_length'])
        for v in previous[:2]:assert v/resolution==pytest.approx(round(v/resolution))


@pytest.mark.parametrize('backend,service,health',[
    ('nav2_dwb','/wheelchair/compute_discrete_path','/wheelchair/path_planner_health'),
    ('smac_lattice','/wheelchair/smac/compute_discrete_path','/wheelchair/smac/path_planner_health')])
def test_backend_selection_initializes_before_parameter_lock(backend,service,health):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.parameter import Parameter
    from wheelchair_control.nodes import PlannerNode
    rclpy.init(args=['--ros-args','-p','navigation_backend:=nav2','-p','path_planner_backend:='+backend,
                     '-r','/wheelchair/compute_discrete_path:='+service,
                     '-r','/wheelchair/path_planner_health:='+health],signal_handler_options=SignalHandlerOptions.NO)
    node=None
    try:
        node=PlannerNode()
        assert node.path_client.backend==backend
        assert node.resolve_service_name(node.path_client.client.srv_name)==service
        assert any(s.topic_name==health for s in node.subscriptions)
        assert not node.set_parameters([Parameter('margin',value=.01)])[0].successful
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()
