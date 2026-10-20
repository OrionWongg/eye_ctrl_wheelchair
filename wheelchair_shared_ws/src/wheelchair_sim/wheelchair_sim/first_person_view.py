"""Read-only seated camera display; no motion publishers or camera hardware."""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


def decode_image(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('Expected rgb8 or bgr8, got '+msg.encoding)
    if msg.step < msg.width*3 or len(msg.data) != msg.step*msg.height:
        raise ValueError('Invalid camera image dimensions')
    rows=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
    frame=rows[:,:msg.width*3].reshape(msg.height,msg.width,3)
    return cv2.cvtColor(frame,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else frame.copy()


def main():
    rclpy.init()
    node=Node('wheelchair_first_person_view')
    title='Wheelchair - First Person (F: fullscreen, Esc: close)'
    latest=None
    received=0.
    def receive(msg):
        nonlocal latest,received
        try:
            latest=decode_image(msg)
            received=time.monotonic()
        except ValueError as exc:
            node.get_logger().error(str(exc))
    node.create_subscription(Image,'/wheelchair/first_person/image_raw',receive,
        QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    fullscreen=False
    try:
        cv2.namedWindow(title,cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(title,960,540)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            # Never present an old still frame as a live driving view.
            if latest is None or time.monotonic()-received>1.:
                frame=np.zeros((540,960,3),np.uint8)
                text='Waiting for camera...' if latest is None else 'Camera feed interrupted'
                cv2.putText(frame,text,(100,270),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
            else:
                frame=latest
            cv2.imshow(title,frame)
            key=cv2.waitKey(1)&0xff
            # GTK reports VISIBLE=-1 even for an open window; AUTOSIZE is
            # supported by both GTK and Qt and becomes -1 when it is closed.
            if key in (27,ord('q')) or cv2.getWindowProperty(title,cv2.WND_PROP_AUTOSIZE)<0:
                break
            if key in (ord('f'),ord('F')):
                fullscreen=not fullscreen
                cv2.setWindowProperty(title,cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
