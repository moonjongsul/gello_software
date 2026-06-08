import rclpy
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CompressedImage, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np


class RealSenseWrapper:
    def __init__(self, cfg, node):
        self.cfg = cfg
        self._node = node
        self._logger = node.get_logger()
        
        self.current_color_frame = None
        self.current_depth_frame = None
        self.intrinsics = None

        self.fx_default, self.fy_default = 651.066, 650.289
        self.cx_default, self.cy_default = 645.814, 359.238

        self.sub_topics = self._to_dict(self.cfg.subscribe_list)
        
        self.get_intrinsics()
        
    def _to_dict(self, items):
            return dict(item.split(":", 1) for item in items)
        
    def get_data(self):
        color = self.get_color()
        depth = self.get_depth()
        
        return color, depth

    def get_color(self, timeout=1.0):
        _, msg = wait_for_message(CompressedImage, self._node, self.sub_topics['color'], time_to_wait=timeout)
        if not _:
            return None
        arr = np.frombuffer(msg.data, np.uint8)
        color = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return color
        
    def get_depth(self, timeout=1.0):
        _, msg = wait_for_message(CompressedImage, self._node, self.sub_topics['depth'], time_to_wait=timeout)
        if not _:
            return None
        depth_header_size = 12
        raw_data = msg.data[depth_header_size:]
        arr = np.frombuffer(raw_data, np.uint8)
        depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if depth is None:
            return
        return depth
    
    def get_intrinsics(self, timeout=1.0):
        _, msg = wait_for_message(CameraInfo, self._node, self.sub_topics['info'], time_to_wait=timeout)        
        if not _:
            return None
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        self.intrinsics = [fx, fy, cx, cy]
        
    def get_3d_coordinate(self, x, y, depth):
        try:
            #depth_mm = self.current_depth_frame[int(pixel_y), int(pixel_x)]
            #if depth_mm == 0: 
            #    return None
            z = depth[y, x]
            z /= 1000
            if not z:
                return
            
            fx, fy, cx, cy = self.intrinsics
            
            x_cam = (x - cx) * z / fx
            y_cam = (y - cy) * z / fy
            return np.array([x_cam, y_cam, z])
        
        except Exception:
            return None
