import time
from types import SimpleNamespace
import pytest

def test_detector_restart_keeps_health_counter_monotonic_and_retires_old_sensor():
    pytest.importorskip('rclpy')
    from eye_shared_control.confirm_node import ConfirmTransport
    from wheelchair_interfaces.msg import EyeSample
    transport=ConfirmTransport.__new__(ConfirmTransport)
    transport.sensor_session='old';transport.retired_sensors=set()
    transport.sensor_sequence=100;transport.sample_sequence=100
    faults=[];received=[]
    transport.machine=SimpleNamespace(invalidate=lambda reason,now:faults.append(reason))
    transport.receive=lambda *args:received.append(args)
    msg=EyeSample();msg.source_session_id='new';msg.sequence=1;msg.valid=True
    msg.header.stamp.sec=int(time.time())
    transport.stamped(msg)
    assert transport.sample_sequence==101 and transport.sensor_sequence==1
    assert faults==['SENSOR_SESSION_CHANGED'] and len(received)==1
    transport.stamped(msg)  # duplicate new frame is not health
    msg.source_session_id='old';msg.sequence=101;transport.stamped(msg)
    assert transport.sample_sequence==101 and len(received)==1
