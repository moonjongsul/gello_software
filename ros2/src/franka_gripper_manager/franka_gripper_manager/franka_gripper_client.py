import rclpy
import time
from rclpy.node import Node
from franka_msgs.action import Move, Grasp, Homing
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from std_msgs.msg import Float32

DEFAULT_MOVE_ACTION_TOPIC = "franka_gripper/move"
DEFAULT_GRASP_ACTION_TOPIC = "franka_gripper/grasp"
DEFAULT_HOMING_ACTION_TOPIC = "franka_gripper/homing"
DEFAULT_JOINT_STATES_TOPIC = "franka_gripper/joint_states"
DEFAULT_GRIPPER_COMMAND_TOPIC = "gripper/gripper_client/target_gripper_width_percent"

OPEN_THRESHOLD = 0.8      # trigger 이 이상이면 open
GRASP_FORCE = 10.0        # N — 조절 가능
GRASP_SPEED = 0.1         # m/s
OPEN_SPEED = 0.3          # m/s
EPSILON_INNER = 0.08      # 3cm — 물건 두께 오차 허용 (넉넉하게)
EPSILON_OUTER = 0.08
MIN_SEND_INTERVAL = 0.1   # 10Hz


class GripperClient(Node):
    def __init__(self):
        super().__init__("gripper_client")

        self.declare_parameter("move_action_topic", DEFAULT_MOVE_ACTION_TOPIC)
        self.declare_parameter("grasp_action_topic", DEFAULT_GRASP_ACTION_TOPIC)
        self.declare_parameter("homing_action_topic", DEFAULT_HOMING_ACTION_TOPIC)
        self.declare_parameter("gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC)
        self.declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC)

        move_action_topic = self.get_parameter("move_action_topic").get_parameter_value().string_value
        grasp_action_topic = self.get_parameter("grasp_action_topic").get_parameter_value().string_value
        homing_action_topic = self.get_parameter("homing_action_topic").get_parameter_value().string_value
        gripper_command_topic = self.get_parameter("gripper_command_topic").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value

        self._ACTION_SERVER_TIMEOUT = 10.0
        self._max_width = 0.0
        self._current_goal_handle = None
        self._last_state = None        # "open" or "grasp"
        self._pending_trigger = None
        self._last_send_time = 0.0

        self.get_logger().info("Initializing gripper client...")
        self._home_gripper(homing_action_topic)
        self._get_max_gripper_width(joint_states_topic)

        self._move_client = ActionClient(self, Move, move_action_topic)
        self._grasp_client = ActionClient(self, Grasp, grasp_action_topic)

        if not self._move_client.wait_for_server(timeout_sec=self._ACTION_SERVER_TIMEOUT):
            raise RuntimeError("Move action server not available!")
        if not self._grasp_client.wait_for_server(timeout_sec=self._ACTION_SERVER_TIMEOUT):
            raise RuntimeError("Grasp action server not available!")

        self._sub = self.create_subscription(
            Float32, gripper_command_topic, self._trigger_cb, 10)

        self.create_timer(MIN_SEND_INTERVAL, self._process_pending)
        self.get_logger().info("Gripper client ready.")

    # ── homing / max_width (기존과 동일) ─────────────────────────────────

    def _home_gripper(self, homing_action_topic):
        client = ActionClient(self, Homing, homing_action_topic)
        if not client.wait_for_server(timeout_sec=self._ACTION_SERVER_TIMEOUT):
            raise RuntimeError("Homing server not available!")
        future = client.send_goal_async(Homing.Goal())
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        time.sleep(2)

    def _get_max_gripper_width(self, joint_states_topic):
        done = rclpy.task.Future()
        def cb(msg):
            self._max_width = 2 * msg.position[0]
            done.set_result(True)
        sub = self.create_subscription(JointState, joint_states_topic, cb, 10)
        rclpy.spin_until_future_complete(self, done)
        self.destroy_subscription(sub)
        self.get_logger().info(f"Max width: {self._max_width:.4f} m")

    # ── 핵심 로직 ─────────────────────────────────────────────────────────

    def _trigger_cb(self, msg: Float32) -> None:
        self._pending_trigger = float(msg.data)

    def _process_pending(self) -> None:
        if self._pending_trigger is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_send_time < MIN_SEND_INTERVAL:
            return

        trigger = self._pending_trigger
        self._pending_trigger = None

        want_open = trigger >= OPEN_THRESHOLD

        # 상태가 바뀔 때만 새 goal 전송
        new_state = "open" if want_open else "grasp"
        if new_state == self._last_state:
            return

        # 이전 goal cancel
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        if want_open:
            self._send_open()
        else:
            self._send_grasp(trigger)

        self._last_state = new_state
        self._last_send_time = now

    def _send_open(self) -> None:
        goal = Move.Goal()
        goal.width = self._max_width
        goal.speed = OPEN_SPEED
        self.get_logger().info("Gripper: OPEN")
        f = self._move_client.send_goal_async(goal)
        f.add_done_callback(self._on_goal_response)

    def _send_grasp(self, trigger: float) -> None:
        goal = Grasp.Goal()
        # trigger 0~0.5 → width 0~max/2 (선택적: 고정값 0.0 써도 됨)
        goal.width = 0.0
        goal.speed = GRASP_SPEED
        goal.force = GRASP_FORCE
        goal.epsilon.inner = EPSILON_INNER
        goal.epsilon.outer = EPSILON_OUTER
        self.get_logger().info(f"Gripper: GRASP (force={GRASP_FORCE}N)")
        f = self._grasp_client.send_goal_async(goal)
        f.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Goal rejected")
            self._last_state = None  # 재시도 허용
            return
        self._current_goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        self._current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = GripperClient()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()