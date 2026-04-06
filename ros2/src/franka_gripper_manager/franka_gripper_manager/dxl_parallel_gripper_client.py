import time
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

try:
    from dynamixel_sdk import PortHandler, PacketHandler  # type: ignore
except Exception:  # pragma: no cover
    PortHandler = None
    PacketHandler = None

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 57600
DEFAULT_JOINT_STATES_TOPIC = "dxl_parallel_gripper/joint_states"
DEFAULT_GRIPPER_COMMAND_TOPIC = "gripper/gripper_client/target_gripper_width_percent"
MIN_SEND_INTERVAL = 0.1   # 10Hz

# control_mode: "continuous" → 0~1을 그리퍼 위치에 직접 매핑
#               "binary"     → threshold 초과 시 열기, 이하 시 닫기
DEFAULT_CONTROL_MODE = "continuous"
DEFAULT_BINARY_THRESHOLD = 0.8

DXL_ID = 1
DXL_MIN_POSITION = 1200
DXL_MAX_POSITION = 2200
DXL_MAX_CURRENT = 600 # mA

class DynamixelController:
    """
    Minimal controller for Dynamixel XH540 serie (Protocol 2.0)
    """
    ADDR_TORQUE_ENABLE = 64
    ADDR_OPERATING_MODE = 11
    ADDR_CURRENT_LIMIT = 38
    ADDR_GOAL_CURRENT = 102
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_CURRENT = 126
    ADDR_PRESENT_VELOCITY = 128
    ADDR_PRESENT_POSITION = 132

    OPERATING_MODE_POSITION = 3
    OPERATING_MODE_CURRENT_BASED_POSITION = 5

    def __init__(self, port_name: str, baudrate: int, dxl_id: int, max_current_ma: int):
        if PortHandler is None or PacketHandler is None:
            raise RuntimeError("dynamixel_sdk is not available. Please install 'dynamixel-sdk'.")
        self.port_name = port_name
        self.baudrate = baudrate
        self.dxl_id = dxl_id
        self.max_current_ma = max_current_ma
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(2.0)
        self._connected = False
        self._last_send_time = 0.0
        self.state_cache = dict()

    def connect(self):
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {self.port_name}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate: {self.baudrate}")
        self._connected = True
        # Ensure torque disabled before changing modes/limits
        try:
            self.enable_torque(False)
        except Exception:
            pass
        # Current-Based Position mode
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_OPERATING_MODE, self.OPERATING_MODE_CURRENT_BASED_POSITION
        )
        if dxl_comm_result != 0 or dxl_error != 0:
            raise RuntimeError(f"Failed to set operating mode: comm={dxl_comm_result}, err={dxl_error}")
        # Set current limit (mA)
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_CURRENT_LIMIT, int(self.max_current_ma)
        )
        if dxl_comm_result != 0 or dxl_error != 0:
            raise RuntimeError(f"Failed to set current limit {self.max_current_ma}mA: comm={dxl_comm_result}, err={dxl_error}")
        # Small delay to ensure settings take effect
        time.sleep(0.02)
        # Torque on
        self.enable_torque(True)

    def disconnect(self):
        try:
            self.enable_torque(False)
        finally:
            if self._connected:
                self.port_handler.closePort()
                self._connected = False

    def enable_torque(self, enable: bool) -> None:
        value = 1 if enable else 0
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_TORQUE_ENABLE, value
        )
        if dxl_comm_result != 0 or dxl_error != 0:
            raise RuntimeError(f"Failed to {'enable' if enable else 'disable'} torque: comm={dxl_comm_result}, err={dxl_error}")

    def set_goal_position(self, position: int) -> None:
        now = time.time()
        if now - self._last_send_time < MIN_SEND_INTERVAL:
            return
        self._last_send_time = now
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_GOAL_POSITION, position
        )
        if dxl_comm_result != 0 or dxl_error != 0:
            raise RuntimeError(f"Failed to set goal position {position}: comm={dxl_comm_result}, err={dxl_error}")

    def read_status(self):
        try:
            pos_result, pos_comm, pos_err = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, self.ADDR_PRESENT_POSITION)
            vel_result, vel_comm, vel_err = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, self.ADDR_PRESENT_VELOCITY)
            cur_result, cur_comm, cur_err = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, self.ADDR_PRESENT_CURRENT)
            if any([pos_err, vel_err, cur_err]) or any([pos_comm != 0, vel_comm != 0, cur_comm != 0]):
                return None
        except Exception:
            return None

        def to_int16(val):
            return struct.unpack('<h', struct.pack('<H', val & 0xFFFF))[0]
        def to_int32(val):
            return struct.unpack('<i', struct.pack('<I', val & 0xFFFFFFFF))[0]

        self.state_cache['motor_position'] = to_int32(pos_result)
        self.state_cache['motor_current'] = to_int16(cur_result)
        self.state_cache['motor_velocity'] = to_int32(vel_result)
        self.state_cache['finger_position'] = to_int32(pos_result)
        return self.state_cache

class GripperClient(Node):
    def __init__(self):
        super().__init__("gripper_client")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC)
        self.declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC)
        # Dynamixel parameters (overridable via launch)
        self.declare_parameter("dxl_id", DXL_ID)
        self.declare_parameter("dxl_min_position", DXL_MIN_POSITION)
        self.declare_parameter("dxl_max_position", DXL_MAX_POSITION)
        self.declare_parameter("dxl_max_current", DXL_MAX_CURRENT)
        # Control mode parameters
        self.declare_parameter("control_mode", DEFAULT_CONTROL_MODE)
        self.declare_parameter("binary_threshold", DEFAULT_BINARY_THRESHOLD)
        self.declare_parameter("position_snap_threshold", 0.02)

        gripper_command_topic = self.get_parameter("gripper_command_topic").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.dxl_id = self.get_parameter("dxl_id").get_parameter_value().integer_value
        self.dxl_min = self.get_parameter("dxl_min_position").get_parameter_value().integer_value
        self.dxl_max = self.get_parameter("dxl_max_position").get_parameter_value().integer_value
        self.dxl_max_current = self.get_parameter("dxl_max_current").get_parameter_value().integer_value
        self.control_mode = self.get_parameter("control_mode").get_parameter_value().string_value
        self.binary_threshold = self.get_parameter("binary_threshold").get_parameter_value().double_value
        self.position_snap_threshold = self.get_parameter("position_snap_threshold").get_parameter_value().double_value

        if self.control_mode not in ("continuous", "binary"):
            self.get_logger().warn(
                f"Unknown control_mode '{self.control_mode}', falling back to 'continuous'."
            )
            self.control_mode = "continuous"

        self.get_logger().info(
            f"Gripper control_mode: '{self.control_mode}'"
            + (f", binary_threshold: {self.binary_threshold}" if self.control_mode == "binary" else "")
        )

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self.gripper_client = DynamixelController(port, baud, self.dxl_id, self.dxl_max_current)
        self.gripper_client.connect()
        self.gripper_client.set_goal_position(self.dxl_min)
        time.sleep(1)
        self.gripper_client.set_goal_position(self.dxl_max)

        self.joint_states_publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.gripper_command_subscriber = self.create_subscription(Float32, gripper_command_topic, self._trigger_cb, 10, callback_group=self.group1)

        timer_period = 1.0 / 200.0  # 30Hz
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.group2)

    def _trigger_cb(self, msg: Float32) -> None:
        raw = float(msg.data)
        raw = max(0.0, min(1.0, raw))

        if self.control_mode == "binary":
            target = self.dxl_max if raw > self.binary_threshold else self.dxl_min
        else:  # "continuous"
            target = int(round(self.dxl_min + raw * (self.dxl_max - self.dxl_min)))

        try:
            self.gripper_client.set_goal_position(target)
        except Exception as e:
            self.get_logger().error(f"Failed to send goal position {target}: {e}")

    def timer_callback(self) -> None:
        gripper_state = self.gripper_client.read_status()
        # self.get_logger().info("Gripper state: {0}".format(gripper_state))
        if gripper_state is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['finger_joint']
        raw_pos = float(gripper_state['finger_position'])
        normalized = (raw_pos - self.dxl_min) / (self.dxl_max - self.dxl_min)
        normalized = max(0.0, min(1.0, normalized))
        if normalized >= 1.0 - self.position_snap_threshold:
            normalized = 1.0
        elif normalized <= self.position_snap_threshold:
            normalized = 0.0
        msg.position = [normalized]
        msg.velocity = [float(gripper_state['motor_velocity'])]
        msg.effort = [-float(gripper_state['motor_current'])]
        self.joint_states_publisher.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = GripperClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
