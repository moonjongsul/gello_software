import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
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

class Ur10_Controller_Node(Node):
    def __init__(self):
        super().__init__('ur_base_controller')
        self.script_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        try:
            self.instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1, 'rtu')
            self.instrument.serial.baudrate = 115200
            self.instrument.serial.bytesize = 8
            self.instrument.serial.parity = serial.PARITY_NONE
            self.instrument.serial.stopbits = 1
            self.instrument.serial.timeout = 0.5
            self.instrument.address = 1
            self.instrument.mode = minimalmodbus.MODE_RTU
            self.instrument.clear_buffers_before_each_transaction = True
            self.gripper_init()
        except Exception as e:
            self.instrument = None

        self.current_tcp_pose = np.zeros(6)
        self.current_joint_positions = np.zeros(6)
        self.ur_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.SAFETY_LIMITS = {'z_min': 0.18, 'z_max': 0.90, 'x_min': -0.750, 'x_max': 0.350, 'y_min': -0.700, 'y_max': 0.100}

    def tf_callback(self, msg):
        for transform in msg.transforms:
            child_id = transform.child_frame_id
            if any(target in child_id for target in ['tool0', 'wrist_3', 'flange']):
                t = transform.transform.translation
                r = transform.transform.rotation
                rotvec = R.from_quat([r.x, r.y, r.z, r.w]).as_rotvec()
                self.current_tcp_pose = np.array([t.x, t.y, t.z, rotvec[0], rotvec[1], rotvec[2]])

    def joint_state_callback(self, msg):
        positions = []
        for name in self.ur_joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        if len(positions) == 6:
            self.current_joint_positions = np.array(positions)

    def get_pose(self): return np.copy(self.current_tcp_pose)
    def get_joint(self): return np.copy(self.current_joint_positions)

    def _publish(self, script):
        msg = String(); msg.data = script.strip() + "\n"
        self.script_pub.publish(msg)

    def _apply_safety_boundary(self, pose):
        safe_pose = np.copy(pose)
        safe_pose[0] = np.clip(safe_pose[0], self.SAFETY_LIMITS['x_min'], self.SAFETY_LIMITS['x_max'])
        safe_pose[1] = np.clip(safe_pose[1], self.SAFETY_LIMITS['y_min'], self.SAFETY_LIMITS['y_max'])
        safe_pose[2] = np.clip(safe_pose[2], self.SAFETY_LIMITS['z_min'], self.SAFETY_LIMITS['z_max'])
        return safe_pose

    def gripper_init(self):
        if self.instrument is None: return False
        success = self._send_gripper([GRP_INIT])
        time.sleep(5)
        return success

    def _send_gripper(self, opt):
        if self.instrument is None: return False
        try:
            self.instrument.write_registers(0, opt)
            return True
        except: return False

    def set_gripper(self, state):
        if self.instrument is None: return
        if isinstance(state, str):
            if state.lower() == 'open': self._send_gripper([GRP_OPEN])
            elif state.lower() == 'close': self._send_gripper([GRP_CLOSE])
        elif isinstance(state, (int, float)):
            pos = int(state * 5.0)
            self._send_gripper([GRP_POS_CTRL, pos])

    def move_home(self, wait=True):
        home_deg = [57.0, -124.0, 98.0, -64.0, -90.0, -33.0]
        rads = [math.radians(d) for d in home_deg]
        script = f"movej([{','.join(map(str, rads))}], a=1.0, v=0.5)"
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
            rclpy.spin_once(self, timeout_sec=0)
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
            rclpy.spin_once(self, timeout_sec=0)
            current = self.get_joint()
            if np.all(current == 0): continue
            diff = np.linalg.norm(target_joints - current)
            if diff < tol: return True
            time.sleep(0.01)
        return False

def main(args=None):
    rclpy.init(args=args)
    node = Ur10_Controller_Node()
    
    try:
        # 1. 그리퍼 연결 상태 확인 및 초기 동작 테스트
        if node.instrument is not None:
            node.get_logger().info("✊ 그리퍼 테스트 시작...")
            node.set_gripper('open')
            time.sleep(2.0)
            node.set_gripper('close')
            time.sleep(2.0)
            node.get_logger().info("✅ 그리퍼 테스트 완료")
        else:
            node.get_logger().error("❌ 그리퍼가 연결되지 않았습니다. 포트 및 전원을 확인하세요.")

        # 2. 로봇 홈 위치로 이동 (안전 확인용)
        node.get_logger().info("🏠 로봇 홈 위치로 이동합니다...")
        node.move_home(wait=True)
        node.get_logger().info("✅ 홈 도착 완료")

        # 3. 추가적인 시퀀스가 필요하다면 여기에 작성 (예: 특정 위치 이동 테스트)
        # target_pose = [0.1, -0.4, 0.5, 0.0, 3.14, 0.0]
        # node.move_to(target_pose, wait=True)

        node.get_logger().info("📡 현재 상태 유지 및 데이터 수신 대기 중 (Spin)...")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("🛑 사용자에 의해 종료되었습니다.")
    finally:
        # 종료 전 안전하게 그리퍼 열기 (선택 사항)
        # node.set_gripper('open')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    