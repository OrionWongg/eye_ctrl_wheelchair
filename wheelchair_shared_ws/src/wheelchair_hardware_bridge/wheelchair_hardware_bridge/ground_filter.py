"""Ground separation in base_link using a measured, fixed level-floor reference.

Never fit the floor to an obstacle top. Missing ground returns do not prove free
space. The 3 cm tolerance is a detection limit, not a traversable-step claim.
"""
import math
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def classify_ground(points, ground_z, tolerance, obstacle_height):
    if (not all(math.isfinite(v) for v in (ground_z, tolerance, obstacle_height))
            or not 0 < tolerance < obstacle_height):
        raise ValueError('ground tolerance must be positive and below obstacle height')
    points = np.asarray(points)
    finite = np.isfinite(points).all(axis=1)
    height = points[:, 2] - ground_z
    ground = finite & (np.abs(height) <= tolerance)
    # Below-floor returns may indicate an edge, hole or incorrect calibration.
    # Preserve them as blocking evidence instead of silently discarding them.
    obstacles = finite & ~ground & (height <= obstacle_height)
    return points[ground], points[obstacles]


def xyz_cloud(points, stamp, frame):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 12, 12*len(points)
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg
