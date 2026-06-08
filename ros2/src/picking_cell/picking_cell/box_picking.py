from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
import cv2
import numpy as np

from picking_cell.utils.realsense_wrapper import RealSenseWrapper
from picking_cell.utils.ur10_wrapper import Ur10ControlWrapper

class BoxPicking(Node):
    def __init__(self):
        super().__init__(
            'box_picking',
            automatically_declare_parameters_from_overrides=True,
        )
        # Load the parameters passed in from config.yaml into a nested dict.
        self.cfg = self._load_cfg()
        # self.get_logger().info(f"Loaded config: {self.cfg.robot}")

        self.ur10 = Ur10ControlWrapper(self.cfg.robot, node=self)
        self.camera = RealSenseWrapper(self.cfg.camera, node=self)
        
        self.T_cam2tcp = np.array(self.cfg.camera.T_cam2tcp).reshape(4, 4)
        # Trigger service: any call kicks off a full box-picking cycle. No
        # request data; the response just reports success/failure.
        self.pick_srv = self.create_service(
            Trigger, self.cfg.vision.service_name, self.srv_pick_cb
        )
        
        self.get_logger().info(f"{self.cfg.vision.publish_list[0].split(':')[1]}")
        
        self.pub_vis_output = self.create_publisher(CompressedImage, self.cfg.vision.publish_list[0].split(':')[1], 10)
        self.get_logger().info(
            f"Box-picking service ready at '{self.cfg.vision.service_name}'"
        )

    def srv_pick_cb(self, request, response):
        """Run one box-picking cycle. response.success = True on success."""
        try:
            ok = self._run_pick_cycle()
            response.success = bool(ok)
            response.message = 'pick done' if ok else 'pick failed'
        except Exception as e:
            self.get_logger().error(f"Pick cycle error: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _run_pick_cycle(self):
        pick_success = False
        """Object detection + robot motion for one pick. Returns bool success."""
        # TODO: wire up real vision + robot motion here.
        self.get_logger().info("HAHAHAHHAHA")
        
        self.ur10.movej('home')
        
        color, depth = self.camera.get_data()
        
        circles, vis_output = self.process_vision(color, depth)
        
        vis_output_msg = self.arr_to_compressed_msg(vis_output)
        if vis_output_msg is not None:
            self.pub_vis_output.publish(vis_output_msg)
        
        if circles is not None:
            target_pixel = circles[0]
            self.get_logger().info(f"{target_pixel}")
               
                
            return pick_success
        else:
            return pick_success


    def process_vision(self, color, depth):
        if color is None or depth is None: 
            return None
        
        h, w = color.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        roi_xy_min = self.cfg.camera.box_roi.xy_min
        roi_xy_max = self.cfg.camera.box_roi.xy_max
        mask = cv2.rectangle(mask, roi_xy_min, roi_xy_max, 255, -1)
        
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        gray = cv2.bitwise_and(gray, mask)
        blurred = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 
                                   dp=1, 
                                   minDist=55, 
                                   param1=60, 
                                   param2=24, 
                                   minRadius=14, 
                                   maxRadius=17)
        
        res_circles = np.uint16(np.around(circles[0, :])) if circles is not None else None
        
        # output image
        vis_output = color.copy()
        cv2.rectangle(vis_output, roi_xy_min, roi_xy_max, (255, 0, 0), 2)
        try:
            for i in res_circles:
                cv2.circle(vis_output, (i[0], i[1]), i[2], (0, 255, 0), 2)
        except:
            pass
        
        return res_circles, vis_output
        
    def arr_to_compressed_msg(self, arr):
        """Encode a BGR image array as a JPEG CompressedImage message."""
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        ok, buf = cv2.imencode('.jpg', arr)
        if not ok:
            self.get_logger().warn('Failed to encode vis_output to JPEG')
            return None
        msg.data = buf.tobytes()
        return msg

    def _load_cfg(self):
        """Rebuild the nested config dict from the flattened ROS parameters.

        config.yaml is passed to this node via the launch file, so its keys
        arrive as dot-separated parameter names (e.g.
        ``camera.launch_args.serial_no``). This reconstructs the original
        nested structure.
        """
        cfg = {}
        for name, param in self.get_parameters_by_prefix('').items():
            keys = name.split('.')
            node = cfg
            for key in keys[:-1]:
                node = node.setdefault(key, {})
            node[keys[-1]] = param.value
        return self._to_namespace(cfg)

    @classmethod
    def _to_namespace(cls, obj):
        """Recursively turn nested dicts into SimpleNamespace for dot access.

        e.g. ``self.cfg.camera.launch_args.serial_no`` instead of
        ``self.cfg['camera']['launch_args']['serial_no']``.
        """
        if isinstance(obj, dict):
            return SimpleNamespace(
                **{key: cls._to_namespace(value) for key, value in obj.items()}
            )
        if isinstance(obj, list):
            return [cls._to_namespace(item) for item in obj]
        return obj


def main(args=None):
    rclpy.init(args=args)
    
    node = BoxPicking()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info(f"Launch error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == "__main__":
    main()
    
    