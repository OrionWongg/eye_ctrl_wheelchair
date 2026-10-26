"""Synthetic checks; does not start sensor hardware."""
import math
import struct
import unittest
import numpy as np
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2, PointField
from body_cloud_filter import filter_cloud, BodyCloudFilter
import rclpy


def cloud(points, endian='<', padded=False):
    msg = PointCloud2()
    msg.header.frame_id = 'test_lidar'
    msg.header.stamp.sec = 42
    msg.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(('x', 'y', 'z'))]
    msg.fields.append(PointField(name='tag', offset=12, datatype=PointField.UINT32, count=1))
    msg.point_step = 16
    msg.height = len(points) if padded else 1
    msg.width = 1 if padded else len(points)
    msg.row_step = msg.width*16 + (4 if padded else 0)
    msg.is_bigendian = endian == '>'
    msg.data = b''.join(struct.pack(endian+'fffI', *p, i) + (b'PAD!' if padded else b'')
                        for i, p in enumerate(points))
    return msg


class FilterChecks(unittest.TestCase):
    def test_boundaries_and_field_preservation(self):
        t = Transform(); t.rotation.w = 1.0
        points = [(0,0,100), (-.5,0,0), (.5,0,0), (0,-.25,0), (0,.25,0),
                  (.51,0,0), (0,-.26,0), (float('nan'),0,0)]
        for endian in ('<','>'):
            msg = cloud(points,endian,padded=True)
            result = filter_cloud(msg,t,(-.5,.5,-.25,.25))
            self.assertEqual(result.width,2)
            self.assertEqual(result.data.tobytes(), b''.join(
                struct.pack(endian+'fffI',*points[i],i) for i in (5,6)))
            self.assertEqual(result.header,msg.header)
            self.assertEqual(result.fields,msg.fields)
            self.assertEqual(result.row_step,32)

    def test_both_inverted_tilted_mounts(self):
        pitch=math.radians(-1)
        rotation=np.array([[math.cos(pitch),0,-math.sin(pitch)],
                           [0,-1,0],[-math.sin(pitch),0,-math.cos(pitch)]])
        base=np.array([[0,0,.5],[.7,0,.5],[0,.4,.5],[-.5,0,.5]])
        for side in (-.21,.21):
            translation=np.array([.28,side,-.1])
            local=(base-translation) @ rotation
            t=Transform();t.translation.x=.28;t.translation.y=side;t.translation.z=-.1
            t.rotation.w=0.0;t.rotation.x=math.cos(pitch/2);t.rotation.z=-math.sin(pitch/2)
            result=filter_cloud(cloud(local),t,(-.42,.63,-.31,.31))
            self.assertEqual(result.width,3)
            tags=[struct.unpack_from('<I',result.data,i*16+12)[0] for i in range(3)]
            self.assertEqual(tags,[1,2,3])

    def test_node_initializes(self):
        rclpy.init()
        node=BodyCloudFilter()
        self.assertEqual(len(node.publishers_),2)
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
