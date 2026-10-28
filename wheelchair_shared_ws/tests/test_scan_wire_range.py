"""The advertised scan range must survive ROS float32 wire rounding."""
import ast
from pathlib import Path

from sensor_msgs.msg import LaserScan
from rclpy.serialization import deserialize_message, serialize_message


def test_cloud_scan_range_survives_wire_roundtrip():
    source = Path(__file__).parents[1] / 'src/wheelchair_hardware_bridge/wheelchair_hardware_bridge/cloud_to_scan.py'
    tree = ast.parse(source.read_text())
    assignments = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Attribute) and t.attr == 'range_max'
                for t in node.targets)
    ]
    assert len(assignments) == 1
    scan = LaserScan()
    scan.range_max = ast.literal_eval(assignments[0].value)
    scan.ranges = [float('inf'), 0.5, 200.0]
    received = deserialize_message(serialize_message(scan), LaserScan)
    assert received.range_max == scan.range_max
    assert received.ranges == scan.ranges
