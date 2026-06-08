import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import time
import serial
import minimalmodbus
from scipy.spatial.transform import Rotation as R

GRP_INIT = 101
GRP_OPEN = 102
GRP_CLOSE = 103
GRP_POS_CTRL = 104

class Ur10ControlWrapper:
    def __init__(self, cfg, node):
        self.cfg = cfg
        # Owning rclpy Node, borrowed for logging and spinning. This wrapper is
        # a plain helper, not a Node itself.
        self._node = node
        self._logger = node.get_logger()

        self.gripper = None
        self._load_gripper()
        
        # self._logger.info(f"{self.cfg.publish_list[0].split(':')[1]}")
        
        self.set_subscriber_publisher()
        
        self.current_pose = np.zeros(6)
        self.current_joint = np.zeros(6)
        
        # self.publisher_move_msg = self._node.create_publisher(String, self.cfg.publish_list)

    def _to_dict(self, items):
            return dict(item.split(":", 1) for item in items)

    def set_subscriber_publisher(self):
        sub_topics = self._to_dict(self.cfg.subscribe_list)
        pub_topics = self._to_dict(self.cfg.publish_list)
        
        self.sub_pose = self._node.create_subscription(PoseStamped, sub_topics['tcp_pose'], self.pose_callback, 10)
        self.sub_joint = self._node.create_subscription(JointState, sub_topics['joint_state'], self.joint_callback, 10)
        self.sub_state = self._node.create_subscription(Bool, sub_topics['state'], self.state_callback, 10)
        
        self.pub_script = self._node.create_publisher(String, pub_topics['script_pub'], 10)

    def pose_callback(self, msg: PoseStamped):
        _p = msg.pose.position
        _q = msg.pose.orientation
        pose = [_p.x, _p.y, _p.z]
        quat = [_q.x, _q.y, _q.z, _q.w]
        
        euler = R.from_quat(quat).as_rotvec()
        
        pose.extend(euler)
        
        self.current_pose = np.array(pose)
        # self._logger.info(f"CS: {self.current_pose}")
    
    def joint_callback(self, msg: JointState):
        # JointState doesn't guarantee an order, so reorder positions to match
        # the configured joint_list (e.g. shoulder_pan ... wrist_3).
        joint_order = self.cfg.joint_list
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.current_joint = np.array([name_to_pos[name] for name in joint_order])
        except KeyError as e:
            self._logger.warn(f"Joint {e} missing from JointState; names={msg.name}")
            return
        # self._logger.info(f"JS: {self.current_joint}")
    
    def state_callback(self, msg: Bool):
        self.running = msg.data
        self._logger.info(f"{self.running}")
    
    def _publish(self, script):
        msg = String(); msg.data = script.strip() + "\n"
        self.pub_script.publish(msg)
    
    def _load_gripper(self):
        try:
            self.gripper = minimalmodbus.Instrument(self.cfg.gripper_usb_port, 1, 'rtu')
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.bytesize = 8
            self.gripper.serial.parity = serial.PARITY_NONE
            self.gripper.serial.stopbits = 1
            self.gripper.serial.timeout = 0.5
            self.gripper.address = 1
            self.gripper.mode = minimalmodbus.MODE_RTU
            self.gripper.clear_buffers_before_each_transaction = True
            self._init_gripper()
            self._logger.info(f"Gripper initialized on {self.cfg.gripper_usb_port}")
        except Exception as e:
            self._logger.warn(f"Gripper init failed: {e}")
            self.gripper = None
    
    def _init_gripper(self):
        if self.gripper is None: return False
        success = self._send_gripper([GRP_INIT])
        time.sleep(1)
        return success

    def _send_gripper(self, opt):
        if self.gripper is None: return False
        try:
            self.gripper.write_registers(0, opt)
            return True
        except: return False

    def set_gripper(self, state):
        if self.gripper is None: return
        if isinstance(state, str):
            if state.lower() == 'open': self._send_gripper([GRP_OPEN])
            elif state.lower() == 'close': self._send_gripper([GRP_CLOSE])
        elif isinstance(state, (int, float)):
            pos = int(state * 5.0)
            self._send_gripper([GRP_POS_CTRL, pos])
    
    def movej(self, loc=None, acc=1.0, vel=0.5, r=0, wait=True):
        target = []
        for pose, joints in vars(self.cfg.pose).items():
            # self._logger.info(f"{pose}, {joints}")
            if loc == pose:
                target = joints
        if not target:
            target = loc
        
        verify = np.mean(np.array(target))
        if verify > 5 or verify < -5:
            # degree -> radian
            rads = [math.radians(d) for d in target]
        else:
            rads = target
            
        script = f"movej([{', '.join(map(str, rads))}], a={acc}, v={vel})"
        self._publish(script=script)
        # self._logger.info(f"{script}")
            
    def move_home(self, wait=True):
        home_deg = [57.0, -124.0, 98.0, -64.0, -90.0, -33.0]
        rads = [math.radians(d) for d in home_deg]
        script = f"movej([{', '.join(map(str, rads))}], a=1.0, v=0.5)"
        self._publish(script)
        if wait: return self.wait_until_joint_reached(rads)
        return True

    def move_to(self, target_pose, acc=0.2, vel=0.1, gripper_state=None, wait=True):
        safe_pose = self._apply_safety_boundary(target_pose)
        script = f"movel(p[{','.join(map(str, safe_pose))}], a={acc}, v={vel}, r=0)"
        self._publish(script)
        if gripper_state is not None: self.set_gripper(gripper_state)
        if wait: return self.wait_until_reached(safe_pose)
        return True

    def wait_until_reached(self, target_pose, pos_tol=0.002, rot_tol=0.02, timeout=8.0):
        start_time = time.time()
        stable_start_time = None
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self._node, timeout_sec=0)
            current = self.get_pose()
            if np.all(current[:3] == 0): continue
            pos_diff = np.linalg.norm(target_pose[:3] - current[:3])
            rot_diff = np.linalg.norm(target_pose[3:] - current[3:])
            if pos_diff < pos_tol and rot_diff < rot_tol:
                if stable_start_time is None: stable_start_time = time.time()
                if (time.time() - stable_start_time) >= 0.2: return True
            else: stable_start_time = None
            time.sleep(0.005)
        return False

    def wait_until_joint_reached(self, target_joints, tol=0.01, timeout=10.0):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self._node, timeout_sec=0)
            current = self.get_joint()
            if np.all(current == 0): continue
            diff = np.linalg.norm(target_joints - current)
            if diff < tol: return True
            time.sleep(0.01)
        return False