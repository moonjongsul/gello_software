from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo
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

        # Subscribe to the camera streams and cache the latest frame. We must
        # NOT use rclpy.wait_for_message() from inside a service callback: it
        # spins a temporary executor on this node and then destroys a temporary
        # subscription, which collides with the MultiThreadedExecutor already
        # spinning the node and raises "cannot use Destroyable ...". Caching via
        # normal subscriptions keeps get_data() a cheap, non-blocking read.
        self.sub_color = self._node.create_subscription(
            CompressedImage, self.sub_topics['color'],
            self._color_callback, qos_profile_sensor_data,
        )
        self.sub_depth = self._node.create_subscription(
            CompressedImage, self.sub_topics['depth'],
            self._depth_callback, qos_profile_sensor_data,
        )
        self.sub_info = self._node.create_subscription(
            CameraInfo, self.sub_topics['info'],
            self._info_callback, qos_profile_sensor_data,
        )

    def _to_dict(self, items):
            return dict(item.split(":", 1) for item in items)

    def _color_callback(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, np.uint8)
        self.current_color_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _depth_callback(self, msg: CompressedImage):
        # compressedDepth carries a 12-byte header before the PNG payload.
        depth_header_size = 12
        raw_data = msg.data[depth_header_size:]
        arr = np.frombuffer(raw_data, np.uint8)
        depth = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if depth is not None:
            self.current_depth_frame = depth

    def _info_callback(self, msg: CameraInfo):
        if self.intrinsics is None:
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self.intrinsics = [fx, fy, cx, cy]

    def get_data(self):
        """Return the latest cached (color, depth) frames (non-blocking)."""
        return self.current_color_frame, self.current_depth_frame

    def get_color(self):
        return self.current_color_frame

    def get_depth(self):
        return self.current_depth_frame

    def get_3d_coordinate(self, x, y, z):
        try:
            #depth_mm = self.current_depth_frame[int(pixel_y), int(pixel_x)]
            #if depth_mm == 0:
            #    return None
            if self.intrinsics is not None:
                fx, fy, cx, cy = self.intrinsics
            else:
                fx, fy = self.fx_default, self.fy_default
                cx, cy = self.cx_default, self.cy_default

            x_cam = (x - cx) * z / fx
            y_cam = (y - cy) * z / fy
            return np.array([x_cam, y_cam, z])

        except Exception:
            return None
