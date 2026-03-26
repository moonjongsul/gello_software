import time
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from pymodbus.client import ModbusSerialClient

DEFAULT_PORT = "/dev/ttyKoras"
DEFAULT_BAUD = 115200
DEFAULT_FORCE = 40.0    # 0 ~ 100 %
DEFAULT_JOINT_STATES_TOPIC = "koras_gripper/joint_states"
DEFAULT_GRIPPER_COMMAND_TOPIC = "gripper/gripper_client/target_gripper_width_percent"
MIN_SEND_INTERVAL = 0.1   # 10Hz

class GripperControllerWrapper:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        
        # Initialize the pymodbus
        self.client = ModbusSerialClient(self.port, baudrate=self.baud)
        self.gripper_state = dict()

    def connect_grip(self):
        self.client.connect()

    def disconnect_grip(self):
        self.client.close()

    def gripper_initialize(self):
        self.client.write_register(0,101,slave=1)
        time.sleep(0.1)
        self.client.write_register(0,213,slave=1)
        self.client.write_register(1,100,slave=1)
        time.sleep(0.1)

    def set_finger_position(self, position):
        self.client.write_register(0, 104, slave=1)
        self.client.write_register(1, position, slave=1)  # 목표 값 설정

    def set_motor_torque(self,ratio):
        self.client.write_register(0,212, slave=1)
        self.client.write_register(1,ratio,slave=1)


    def read_status(self):
        """
        모터 상태를 읽어 GripperState 메시지 형태로 반환하는 함수
        """
        try:
            # Modbus 요청 시 `slave=1`을 키워드 인자로 전달
            registers = self.client.read_holding_registers(10, count=5, slave=1)
            # 요청 실패 처리
            if not registers or registers.isError():
                print("⚠️ 모터 상태 읽기 실패!")
                return None
        except Exception as e:
            print(f"❌ 모터 상태 읽기 오류: {e}")
            return None

        # GripperState 메시지에 매핑
        def to_int16(val):
            return struct.unpack('>h', struct.pack('>H', val))[0]
        
        self.gripper_state['motor_position'] = to_int16(registers.registers[1])
        self.gripper_state['motor_current'] = to_int16(registers.registers[2])
        self.gripper_state['motor_velocity'] = to_int16(registers.registers[3])
        self.gripper_state['finger_position'] = to_int16(registers.registers[4])

        return self.gripper_state

class GripperClient(Node):
    def __init__(self):
        super().__init__("gripper_client")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("force", DEFAULT_FORCE)
        self.declare_parameter("gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC)
        self.declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC)

        gripper_command_topic = self.get_parameter("gripper_command_topic").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.force = self.get_parameter("force").get_parameter_value().integer_value

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self.gripper_client = GripperControllerWrapper(port, baud)
        self.gripper_client.connect_grip()
        self.gripper_client.gripper_initialize()

        self.joint_states_publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.gripper_command_subscriber = self.create_subscription(Float32, gripper_command_topic, self._trigger_cb, 10, callback_group=self.group1)

        timer_period = 1.0 / 30.0  # 30Hz
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.group2)

    def _trigger_cb(self, msg: Float32) -> None:
        trigger_command = float(msg.data)

        self.gripper_client.set_motor_torque(self.force)
        self.gripper_client.set_finger_position(trigger_command)

    def timer_callback(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['finger_joint']
        msg.position = [self.gripper_client.gripper_state['finger_position']]
        msg.velocity = [self.gripper_client.gripper_state['motor_velocity']]
        msg.effort = [self.gripper_client.gripper_state['motor_current']]
        self.joint_states_publisher.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = GripperClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()