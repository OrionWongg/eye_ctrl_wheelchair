"""Run with the ROS/workspace environment sourced; no simulator or hardware."""
import pytest


@pytest.mark.parametrize('mode', ['gaze_joystick', 'intent_confirm', 'directional_intent'])
def test_runtime_configuration_is_fixed_until_stationary_restart(mode, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    interfaces = pytest.importorskip('wheelchair_interfaces.msg')
    if not hasattr(interfaces, 'CandidateStatus'):
        pytest.skip('build and source wheelchair_interfaces first')
    from rclpy.parameter import Parameter
    from eye_shared_control.node import EyeIntentNode
    monkeypatch.setenv('ROS_DOMAIN_ID', '87')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=['--ros-args', '-p', 'interaction_mode:=' + mode,
                    '-p', 'use_sim_time:=false'])
    node = None
    try:
        node = EyeIntentNode()
        other = 'intent_confirm' if mode == 'gaze_joystick' else 'gaze_joystick'
        for name, value in [('interaction_mode', other), ('dwell', .5), ('use_sim_time', True)]:
            result = node.set_parameters_atomically([Parameter(name, value=value)])
            assert not result.successful
            assert 'stationary' in result.reason and 'restart' in result.reason
        assert node.set_parameters_atomically([Parameter('interaction_mode', value=mode)]).successful
        assert node.get_parameter('interaction_mode').value == mode
        assert node.get_parameter('dwell').value == .25
        assert not node.get_parameter('use_sim_time').value
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
