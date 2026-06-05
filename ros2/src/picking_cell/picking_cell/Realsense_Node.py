import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

class Realsense_Node(Node):
    def __init__(self):
        super().__init__('realsense_vision_node')
        self.bridge = CvBridge()
        
        self.current_color_frame = None
        self.current_depth_frame = None
        self.intrinsics = None

        self.fx_default, self.fy_default = 651.066, 650.289
        self.cx_default, self.cy_default = 645.814, 359.238

        # QoS 센서 데이터 프로파일 적용 구독
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, qos_profile_sensor_data)

        self.get_logger().info('📸 RealSense 데이터 수신 대기 중...')

    # --- [콜백 함수들] ---
    def color_callback(self, msg):
        """컬러 영상 수신 및 저장"""
        self.current_color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        """뎁스 영상 수신 및 저장"""
        self.current_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def info_callback(self, msg):
        """카메라 정보(인트린직) 수신"""
        self.intrinsics = msg

    # --- [Getter 함수들] ---
    def get_color_image(self):
        return self.current_color_frame

    def get_intrinsics(self):
        if self.intrinsics is not None:
            fx = self.intrinsics.k[0]
            fy = self.intrinsics.k[4]
            cx = self.intrinsics.k[2]
            cy = self.intrinsics.k[5]
            return fx, fy, cx, cy
        return self.fx_default, self.fy_default, self.cx_default, self.cy_default

    def get_3d_coordinate(self, pixel_x, pixel_y, depth):
        if self.current_depth_frame is None:
            return None
        try:
            #depth_mm = self.current_depth_frame[int(pixel_y), int(pixel_x)]
            #if depth_mm == 0: 
            #    return None
            depth_z = depth
            fx, fy, cx, cy = self.get_intrinsics()
            X_cam = (pixel_x - cx) * depth_z / fx
            Y_cam = (pixel_y - cy) * depth_z / fy
            return np.array([X_cam, Y_cam, depth_z])
        except Exception:
            return None

def main(args=None):
    rclpy.init(args=args)
    vision_node = Realsense_Node()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()