from types import SimpleNamespace
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from picking_cell_interfaces.srv import MoveJ, MoveL, GripperWidth
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
        
        self.robot_acc = self.cfg.robot.speed.acc
        self.robot_vel = self.cfg.robot.speed.vel
        
        self.cb_group = ReentrantCallbackGroup()

        # Trigger service: any call kicks off a full box-picking cycle. No
        # request data; the response just reports success/failure.
        self.pick_srv = self.create_service(
            Trigger, self.cfg.vision.service_name[0], self.srv_pick_cb,
            callback_group=self.cb_group,
        )
        # MoveJ service: request carries a target pose name (string) such as
        # 'home', 'place', 'place_wp'; the response reports success/failure.
        self.movej_srv = self.create_service(
            MoveJ, self.cfg.vision.service_name[1], self.srv_movej_cb,
            callback_group=self.cb_group,
        )
        # self.movel_srv = self.create_service(
        #     MoveL, self.cfg.vision.service_name[2], self.srv_movel_cb
        # )
        
        self.gripper_srv = self.create_service(
            GripperWidth, self.cfg.vision.service_name[4], self.srv_gripper_cb,
            callback_group=self.cb_group,
        )
        
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

    def srv_movej_cb(self, request, response):
        """Move the robot to the requested named pose (request.location)."""
        location = request.location
        try:
            self.ur10.movej(location, wait=False)
            response.success = True
            response.message = f"moved to '{location}'"
        except Exception as e:
            self.get_logger().error(f"MoveJ error: {e}")
            response.success = False
            response.message = str(e)
        return response
    
    def srv_gripper_cb(self, request, response):
        width = request.width
        try:
            width = int(width)
        except:
            width = str(width)
        try:
            self.ur10.set_gripper(width)
            response.success = True
            response.message = f"gripper width: {width}"
        except Exception as e:
            self.get_logger().error(f"Gripper error: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _run_pick_cycle(self):
        """ home 위치로 이동 """
        self.ur10.movej('home', acc=self.robot_acc * 1.5, vel=self.robot_vel * 2, wait=True)
        time.sleep(0.5)

        """ 물체 인식 """
        color, depth = self.camera.get_data()
        circles, vis_output = self.detect_circle(color)

        vis_output_msg = self.arr_to_compressed_msg(vis_output)
        if vis_output_msg is not None:
            self.pub_vis_output.publish(vis_output_msg)

        if circles is None:
            self.get_logger().info("No circle detected; aborting pick cycle.")
            return False

        """ 좌표변환 """
        target_pixel = circles[0]
        coord_cam = self.camera.get_3d_coordinate(
            target_pixel[0], target_pixel[1], self.cfg.vision.obj2cam
        )
        if coord_cam is None:
            self.get_logger().warn("get_3d_coordinate returned None; aborting.")
            return False
        
        P_cam = np.array([*coord_cam, 1.0])
        P_tcp = self.T_cam2tcp @ P_cam
        dx, dy = float(P_tcp[0]), float(P_tcp[1])
        self.get_logger().info(
            f"pixel={target_pixel[:2]} P_cam={coord_cam} -> dx={dx:.4f} dy={dy:.4f}"
        )
        
        """ 그리퍼 조금 열기 """
        self.ur10.set_gripper(self.cfg.robot.gripper.pick_width)
        
        """ 물체 위치로 이동 (하강 X)"""        
        if not self.ur10.movel_rel([dx+self.cfg.robot.gripper.pick_offset.x,
                                    dy+self.cfg.robot.gripper.pick_offset.y,
                                    0.0,
                                    0, 0, 1.5708], frame='tool',
                                   acc=self.robot_acc, vel=self.robot_vel, wait=True):
            self.get_logger().warn("XY centering move did not converge.")
            return False

        """ 피킹하러 내려감 """        
        descend = -float(self.cfg.vision.obj2tool)
        if not self.ur10.movel_rel([0.0, 0.0, -descend+0.005, 0, 0, 0], frame='tool',
                                   acc=self.robot_acc, vel=self.robot_vel, wait=True):
            self.get_logger().warn("Descent move did not converge.")
            return False
        
        """ 그리퍼 파지 """
        self.ur10.set_gripper(1)
        time.sleep(1)

        """ 그리퍼 파지 후 z축 상승 + drop 위치 이동 """
        self.ur10.movel_rel([0.0, 0.0, descend-0.05, 0, 0, 0], frame='tool',
                            acc=self.robot_acc*0.8, vel=self.robot_vel*0.8, wait=True)
        self.ur10.move_blend([
            ('cs', [0.400, -0.335, 0.750, 2.221, 2.221, 0]),
            ('cs', [0.667, -0.280, 0.470, 1.939, 1.939, 0.498])
            ], acc=self.robot_acc, vel=self.robot_vel, wait=True)

        self.ur10.set_gripper(self.cfg.robot.gripper.pick_width)
        time.sleep(0.3)
        
        """ home 복귀. blending 모션 or movej -> movej가 빠름"""
        self.ur10.move_blend([
            ('cs', [0.400, -0.335, 0.750, 2.221, 2.221, 0]),
            ('js', 'home')
            ],acc=self.robot_acc*2, vel=self.robot_vel*2, wait=True)
        # self.ur10.movej('home', acc=self.robot_acc, vel=self.robot_vel, wait=True)
        return True


    def detect_circle(self, color):
        if color is None:
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
            for idx, i in enumerate(res_circles):
                if idx == 0:
                    cv2.circle(vis_output, (i[0], i[1]), i[2], (0, 0, 255), 2)
                else:
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
    # MultiThreadedExecutor so the long-running pick service can block on
    # movel_rel waits while pose/joint subscription callbacks keep running on
    # other threads (see the ReentrantCallbackGroup in __init__).
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().info(f"Launch error: {e}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == "__main__":
    main()
    
    