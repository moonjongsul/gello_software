from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import IsInRemoteControl
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import time
import threading
import socket
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
        self._node = node
        self._logger = node.get_logger()

        self.current_pose = np.zeros(6, dtype=float)
        self.current_joint = np.zeros(6, dtype=float)
        self.running = False
        self._state_received = False
        self._running_event = threading.Event()
        self._motion_lock = threading.Lock()
        self._robot_ip = self._infer_robot_ip()
        self._script_port = 30002

        self.gripper = None
        self._load_gripper()
        self.set_subscriber_publisher()
        self._create_dashboard_clients()

    def _to_dict(self, items):
        return dict(item.split(":", 1) for item in items)

    def _normalize_namespace(self, namespace):
        namespace = str(namespace).strip()
        if not namespace:
            return "/ur10e"
        return "/" + namespace.strip("/")

    def _infer_robot_ip(self):
        for attr in ("robot_ip", "ur_ip", "ip", "host"):
            value = getattr(self.cfg, attr, None)
            if value:
                return str(value)
        return "172.16.0.5"

    def _infer_robot_namespace(self, script_topic):
        configured_namespace = getattr(self.cfg, "robot_namespace", None)
        if configured_namespace is None:
            configured_namespace = getattr(self.cfg, "ur_namespace", None)
        if configured_namespace:
            return self._normalize_namespace(configured_namespace)

        parts = str(script_topic).strip("/").split("/")
        if parts and parts[0]:
            return f"/{parts[0]}"
        return "/ur10e"

    def set_subscriber_publisher(self):
        sub_topics = self._to_dict(self.cfg.subscribe_list)
        pub_topics = self._to_dict(self.cfg.publish_list)

        self._script_topic = pub_topics["script_pub"]
        self._robot_namespace = self._infer_robot_namespace(self._script_topic)

        self.sub_pose = self._node.create_subscription(
            PoseStamped,
            sub_topics["tcp_pose"],
            self.pose_callback,
            10,
        )
        self.sub_joint = self._node.create_subscription(
            JointState,
            sub_topics["joint_state"],
            self.joint_callback,
            10,
        )
        self.sub_state = self._node.create_subscription(
            Bool,
            sub_topics["state"],
            self.state_callback,
            10,
        )
        self.pub_script = self._node.create_publisher(
            String,
            self._script_topic,
            10,
        )

    def _create_dashboard_clients(self):
        dashboard_ns = f"{self._robot_namespace}/dashboard_client"
        self.dashboard_quit_client = self._node.create_client(
            Trigger,
            f"{dashboard_ns}/quit",
        )
        self.dashboard_connect_client = self._node.create_client(
            Trigger,
            f"{dashboard_ns}/connect",
        )
        self.dashboard_play_client = self._node.create_client(
            Trigger,
            f"{dashboard_ns}/play",
        )
        self.dashboard_remote_client = self._node.create_client(
            IsInRemoteControl,
            f"{dashboard_ns}/is_in_remote_control",
        )
        self._logger.info(
            f"Dashboard auto-restore services: {dashboard_ns}/quit, "
            f"{dashboard_ns}/connect, {dashboard_ns}/play"
        )

    def pose_callback(self, msg: PoseStamped):
        position = msg.pose.position
        orientation = msg.pose.orientation
        quat = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        rotvec = R.from_quat(quat).as_rotvec()
        self.current_pose = np.array(
            [
                position.x,
                position.y,
                position.z,
                rotvec[0],
                rotvec[1],
                rotvec[2],
            ],
            dtype=float,
        )

    def joint_callback(self, msg: JointState):
        joint_order = self.cfg.joint_list
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.current_joint = np.array(
                [name_to_pos[name] for name in joint_order],
                dtype=float,
            )
        except KeyError as error:
            self._logger.warn(
                f"Joint {error} missing from JointState; names={msg.name}"
            )

    def state_callback(self, msg: Bool):
        new_running = bool(msg.data)
        state_changed = not self._state_received or new_running != self.running
        self.running = new_running
        self._state_received = True

        if new_running:
            self._running_event.set()
        else:
            self._running_event.clear()

        if state_changed:
            self._logger.info(f"External Control running: {self.running}")

    def _publish(self, script):
        # Kept for backward compatibility. Motion methods below use a fresh
        # direct TCP connection to port 30002 so they do not depend on a stale
        # launch-time urscript_interface connection.
        msg = String()
        msg.data = script.strip() + "\n"
        self.pub_script.publish(msg)

    def _build_program(self, name, commands):
        safe_name = "".join(
            char if char.isalnum() or char == "_" else "_"
            for char in str(name)
        )
        body = "\n".join(f"  {command.strip()}" for command in commands)
        return f"def {safe_name}():\n{body}\nend\n"

    def _ensure_remote_control(self, timeout=3.0):
        service_name = (
            f"{self._robot_namespace}/dashboard_client/is_in_remote_control"
        )
        if not self.dashboard_remote_client.wait_for_service(
            timeout_sec=timeout
        ):
            self._logger.warn(
                f"Remote-control check unavailable: {service_name}; "
                "continuing with direct URScript send"
            )
            return True

        future = self.dashboard_remote_client.call_async(
            IsInRemoteControl.Request()
        )
        result, error = self._wait_future(future, timeout)
        if result is None:
            self._logger.error(
                f"Remote-control check failed: {error}"
            )
            return False

        if not result.success or not result.remote_control:
            self._logger.error(
                "Robot is not in Remote Control mode; direct URScript motion "
                "was not sent"
            )
            return False

        return True

    def _send_program_direct(self, program, timeout=3.0):
        if not self._ensure_remote_control(timeout=timeout):
            return False

        payload = program.encode("utf-8")
        try:
            with socket.create_connection(
                (self._robot_ip, self._script_port),
                timeout=timeout,
            ) as connection:
                connection.settimeout(timeout)
                connection.sendall(payload)
                try:
                    connection.shutdown(socket.SHUT_WR)
                except OSError:
                    pass

            self._logger.info(
                f"Direct URScript sent to "
                f"{self._robot_ip}:{self._script_port}:\n"
                f"{program.rstrip()}"
            )
            return True
        except OSError as error:
            self._logger.error(
                f"Direct URScript send failed to "
                f"{self._robot_ip}:{self._script_port}: {error}"
            )
            return False

    def _wait_future(self, future, timeout_sec):
        start_time = time.monotonic()
        while not future.done():
            if time.monotonic() - start_time >= timeout_sec:
                return None, "timeout"
            time.sleep(0.01)

        exception = future.exception()
        if exception is not None:
            return None, str(exception)
        return future.result(), ""

    def _call_trigger(
        self,
        client,
        service_name,
        service_wait_timeout=3.0,
        response_timeout=5.0,
    ):
        if not client.wait_for_service(timeout_sec=service_wait_timeout):
            self._logger.error(
                f"Dashboard service unavailable: {service_name}"
            )
            return False, "service unavailable"

        future = client.call_async(Trigger.Request())
        result, error = self._wait_future(future, response_timeout)
        if result is None:
            self._logger.error(
                f"Dashboard service failed: {service_name}: {error}"
            )
            return False, error

        message = getattr(result, "message", "")
        if result.success:
            self._logger.info(
                f"Dashboard service success: {service_name}: {message}"
            )
            return True, message

        self._logger.warn(
            f"Dashboard service returned false: {service_name}: {message}"
        )
        return False, message

    def _wait_external_control_running(self, timeout=15.0, stable_time=0.3):
        start_time = time.monotonic()
        stable_start = None

        while time.monotonic() - start_time < timeout:
            if self._running_event.is_set():
                if stable_start is None:
                    stable_start = time.monotonic()
                elif time.monotonic() - stable_start >= stable_time:
                    return True
            else:
                stable_start = None
            time.sleep(0.02)

        return False

    def restore_external_control(self, timeout=15.0):
        """Reconnect Dashboard port 29999 and restart External Control."""
        self._logger.info("External Control automatic restore started")

        quit_ok, quit_message = self._call_trigger(
            self.dashboard_quit_client,
            f"{self._robot_namespace}/dashboard_client/quit",
            service_wait_timeout=3.0,
            response_timeout=5.0,
        )
        if not quit_ok:
            self._logger.warn(
                "Dashboard quit did not succeed; continuing with reconnect: "
                f"{quit_message}"
            )

        time.sleep(0.25)

        connect_ok = False
        for attempt in range(1, 4):
            connect_ok, connect_message = self._call_trigger(
                self.dashboard_connect_client,
                f"{self._robot_namespace}/dashboard_client/connect",
                service_wait_timeout=3.0,
                response_timeout=5.0,
            )
            if connect_ok:
                break
            self._logger.warn(
                f"Dashboard reconnect attempt {attempt}/3 failed: "
                f"{connect_message}"
            )
            time.sleep(0.5)

        if not connect_ok:
            self._logger.error("External Control restore failed at Dashboard reconnect")
            return False

        time.sleep(0.25)
        self._running_event.clear()

        play_ok, play_message = self._call_trigger(
            self.dashboard_play_client,
            f"{self._robot_namespace}/dashboard_client/play",
            service_wait_timeout=3.0,
            response_timeout=5.0,
        )
        if not play_ok:
            self._logger.error(
                f"External Control restore failed at Dashboard play: {play_message}"
            )
            return False

        if not self._wait_external_control_running(timeout=timeout):
            self._logger.error(
                "Dashboard play succeeded, but reverse interface did not become "
                f"ready within {timeout:.1f} seconds"
            )
            return False

        self._logger.info("External Control automatic restore completed")
        return True

    def _load_gripper(self):
        try:
            self.gripper = minimalmodbus.Instrument(
                self.cfg.gripper.usb_port,
                1,
                "rtu",
            )
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.bytesize = 8
            self.gripper.serial.parity = serial.PARITY_NONE
            self.gripper.serial.stopbits = 1
            self.gripper.serial.timeout = 0.5
            self.gripper.address = 1
            self.gripper.mode = minimalmodbus.MODE_RTU
            self.gripper.clear_buffers_before_each_transaction = True
            self._init_gripper()
            self._logger.info(
                f"Gripper initialized on {self.cfg.gripper.usb_port}"
            )
        except Exception as error:
            self._logger.warn(f"Gripper init failed: {error}")
            self.gripper = None

    def _init_gripper(self):
        if self.gripper is None:
            return False
        success = self._send_gripper([GRP_INIT])
        time.sleep(1)
        return success

    def _send_gripper(self, opt):
        if self.gripper is None:
            return False
        try:
            self.gripper.write_registers(0, opt)
            return True
        except Exception as error:
            self._logger.warn(f"Gripper command failed: {error}")
            return False

    def set_gripper(self, state):
        if self.gripper is None:
            return False

        if isinstance(state, str):
            state_lower = state.lower()
            if state_lower == "open":
                return self._send_gripper([GRP_OPEN])
            if state_lower == "close":
                return self._send_gripper([GRP_CLOSE])
            if state_lower == "init":
                return self._send_gripper([GRP_INIT])
            self._logger.warn(f"Unknown gripper state: {state}")
            return False

        if isinstance(state, (int, float)):
            pos = int(state * 5.0)
            return self._send_gripper([GRP_POS_CTRL, pos])

        self._logger.warn(f"Unsupported gripper state: {state!r}")
        return False

    def movej(self, loc=None, acc=1.0, vel=0.5, r=0, wait=False):
        if not self._motion_lock.acquire(blocking=False):
            self._logger.warn("movej rejected: robot is already moving")
            return False

        try:
            target = self._resolve_joint(loc)
            if target is None:
                self._logger.warn(
                    f"movej: expected 6-element joint target, got {loc}"
                )
                return False

            command = (
                f"movej([{', '.join(map(str, target))}], "
                f"a={float(acc)}, v={float(vel)}, r={float(r)})"
            )
            program = self._build_program("external_movej", [command])
            if not self._send_program_direct(program):
                return False

            if not wait:
                self._logger.warn(
                    "movej wait=False requested, but the direct-script backend "
                    "waits so External Control can be restored safely"
                )

            reached = self.wait_until_joint_reached(
                np.asarray(target, dtype=float),
                timeout=30.0,
            )
            if not reached:
                if not self.running:
                    self.restore_external_control()
                return False

            return self.restore_external_control()
        finally:
            self._motion_lock.release()

    def movel(self, loc, acc=1.0, vel=0.5, wait=False):
        """Move linearly using a fresh port-30002 connection."""
        if not self._motion_lock.acquire(blocking=False):
            self._logger.warn("movel rejected: robot is already moving")
            return False

        try:
            target = self._resolve_cart(loc)
            if target is None:
                self._logger.warn(
                    f"movel: expected [x, y, z, rx, ry, rz], got {loc!r}"
                )
                return False

            acc = float(acc)
            vel = float(vel)
            if acc <= 0.0 or vel <= 0.0:
                self._logger.warn(
                    f"movel: acceleration and velocity must be positive: "
                    f"a={acc}, v={vel}"
                )
                return False

            command = (
                f"movel(p[{','.join(map(str, target))}], "
                f"a={acc}, v={vel}, r=0)"
            )
            program = self._build_program("external_movel", [command])
            if not self._send_program_direct(program):
                return False

            if not wait:
                self._logger.warn(
                    "movel wait=False requested, but the direct-script backend "
                    "waits so External Control can be restored safely"
                )

            reached = self.wait_until_reached(
                target,
                pos_tol=0.002,
                rot_tol=0.02,
                timeout=30.0,
            )
            if not reached:
                if not self.running:
                    self._logger.warn(
                        "MoveL target was not reached and External Control is not "
                        "running; attempting recovery"
                    )
                    self.restore_external_control()
                return False

            restored = self.restore_external_control(timeout=15.0)
            if not restored:
                self._logger.error(
                    "MoveL reached the target, but External Control restore failed"
                )
                return False

            return True
        finally:
            self._motion_lock.release()

    def _resolve_joint(self, loc):
        target = []
        for pose, joints in vars(self.cfg.pose).items():
            if loc == pose:
                target = joints
                break

        if not target:
            target = loc

        try:
            target = list(np.asarray(target, dtype=float))
        except Exception:
            return None

        if len(target) != 6:
            return None

        verify = np.mean(np.asarray(target, dtype=float))
        if verify > 5 or verify < -5:
            return [math.radians(value) for value in target]
        return target

    def _resolve_cart(self, loc):
        target = []
        for pose, value in vars(self.cfg.pose).items():
            if loc == pose:
                target = value
                break

        if not (
            len(target)
            if isinstance(target, (list, tuple, np.ndarray))
            else target
        ):
            target = loc

        try:
            target = np.asarray(target, dtype=float)
        except Exception:
            return None

        if target.shape != (6,):
            return None
        return target

    def move_blend(self, waypoints, acc=1.0, vel=0.5, blend=0.05, wait=False):
        if not self._motion_lock.acquire(blocking=False):
            self._logger.warn("move_blend rejected: robot is already moving")
            return False

        try:
            parsed = []
            for waypoint in waypoints:
                if (
                    isinstance(waypoint, (tuple, list))
                    and len(waypoint) == 2
                    and isinstance(waypoint[0], str)
                    and waypoint[0] in ("cs", "js")
                ):
                    kind, raw_target = waypoint
                else:
                    kind, raw_target = "cs", waypoint

                if kind == "js":
                    target = self._resolve_joint(raw_target)
                else:
                    target = self._resolve_cart(raw_target)

                if target is None:
                    self._logger.warn(
                        f"move_blend: bad {kind}-waypoint {waypoint!r}"
                    )
                    return False
                parsed.append((kind, target))

            if not parsed:
                self._logger.warn("move_blend: empty waypoint list")
                return False

            count = len(parsed)
            if isinstance(blend, (list, tuple, np.ndarray)):
                radii = [float(value) for value in blend]
                if len(radii) != count:
                    self._logger.warn(
                        f"move_blend: blend list len {len(radii)} != "
                        f"{count} waypoints"
                    )
                    return False
            else:
                radii = [float(blend)] * count
            radii[-1] = 0.0

            lines = []
            for (kind, target), radius in zip(parsed, radii):
                values = ",".join(map(str, target))
                if kind == "js":
                    lines.append(
                        f"  movej([{values}], a={acc}, v={vel}, r={radius})"
                    )
                else:
                    lines.append(
                        f"  movel(p[{values}], a={acc}, v={vel}, r={radius})"
                    )

            script = "def blended_path():\n" + "\n".join(lines) + "\nend\n"
            if not self._send_program_direct(script):
                return False

            if not wait:
                self._logger.warn(
                    "move_blend wait=False requested, but the direct-script "
                    "backend waits so External Control can be restored safely"
                )

            last_kind, last_target = parsed[-1]
            if last_kind == "js":
                reached = self.wait_until_joint_reached(
                    np.asarray(last_target, dtype=float),
                    timeout=30.0,
                )
            else:
                reached = self.wait_until_reached(
                    last_target,
                    timeout=30.0,
                )

            if not reached:
                if not self.running:
                    self.restore_external_control()
                return False
            return self.restore_external_control()
        finally:
            self._motion_lock.release()

    def movel_rel(
        self,
        delta,
        frame="base",
        acc=0.2,
        vel=0.1,
        gripper_state=None,
        wait=True,
    ):
        if not self._motion_lock.acquire(blocking=False):
            self._logger.warn("movel_rel rejected: robot is already moving")
            return False

        try:
            current = self.current_pose.copy()
            if np.all(current[:3] == 0):
                self._logger.warn("movel_rel: current_pose not received yet")
                return False

            delta = np.asarray(delta, dtype=float)
            if delta.shape != (6,):
                self._logger.warn(
                    f"movel_rel: expected 6-element delta, got {delta}"
                )
                return False

            current_rotation = R.from_rotvec(current[3:])
            delta_rotation = R.from_rotvec(delta[3:])

            if frame == "tool":
                target_position = (
                    current[:3] + current_rotation.apply(delta[:3])
                )
                target_rotation = current_rotation * delta_rotation
            elif frame == "base":
                target_position = current[:3] + delta[:3]
                target_rotation = delta_rotation * current_rotation
            else:
                self._logger.warn(
                    f"movel_rel: frame must be 'base' or 'tool', got {frame!r}"
                )
                return False

            target_pose = np.concatenate(
                [target_position, target_rotation.as_rotvec()]
            )
            command = (
                f"movel(p[{','.join(map(str, target_pose))}], "
                f"a={float(acc)}, v={float(vel)}, r=0)"
            )
            program = self._build_program("external_movel_rel", [command])
            if not self._send_program_direct(program):
                return False

            if gripper_state is not None:
                self.set_gripper(gripper_state)

            if not wait:
                self._logger.warn(
                    "movel_rel wait=False requested, but the direct-script "
                    "backend waits so External Control can be restored safely"
                )

            reached = self.wait_until_reached(
                target_pose,
                timeout=30.0,
            )
            if not reached:
                if not self.running:
                    self.restore_external_control()
                return False
            return self.restore_external_control()
        finally:
            self._motion_lock.release()

    def movet(
        self,
        delta,
        acc=0.2,
        vel=0.1,
        wait=True,
        frame="tool",
        gripper_state=None,
    ):
        """Move the TCP by a relative Cartesian offset.

        MoveT defaults to the current tool/TCP coordinate frame:
        [dx, dy, dz, drx, dry, drz]. Translation is in metres and
        rotation is a rotation vector in radians. Set frame="base" only
        when a base-frame relative move is explicitly required.

        The actual motion uses the same fresh port-30002 connection and
        External Control auto-restore sequence as MoveL.
        """
        frame = str(frame).strip().lower()
        if frame not in ("tool", "base"):
            self._logger.warn(
                f"movet: frame must be 'tool' or 'base', got {frame!r}"
            )
            return False

        return self.movel_rel(
            delta=delta,
            frame=frame,
            acc=acc,
            vel=vel,
            gripper_state=gripper_state,
            wait=wait,
        )

    def wait_until_reached(
        self,
        target_pose,
        pos_tol=0.002,
        rot_tol=0.02,
        timeout=8.0,
    ):
        target_pose = np.asarray(target_pose, dtype=float)
        start_time = time.monotonic()
        stable_start_time = None
        last_log_time = 0.0
        target_rotation = R.from_rotvec(target_pose[3:])

        while time.monotonic() - start_time < timeout:
            current = self.current_pose.copy()
            if np.all(current[:3] == 0):
                time.sleep(0.005)
                continue

            current = np.asarray(current, dtype=float)
            pos_diff = float(
                np.linalg.norm(target_pose[:3] - current[:3])
            )
            current_rotation = R.from_rotvec(current[3:])
            relative_rotation = target_rotation.inv() * current_rotation
            rot_diff = float(relative_rotation.magnitude())

            if pos_diff < pos_tol and rot_diff < rot_tol:
                if stable_start_time is None:
                    stable_start_time = time.monotonic()
                if time.monotonic() - stable_start_time >= 0.2:
                    self._logger.info(
                        f"Target reached: position_error={pos_diff:.6f} m, "
                        f"rotation_error={rot_diff:.6f} rad"
                    )
                    return True
            else:
                stable_start_time = None

            if time.monotonic() - last_log_time >= 1.0:
                self._logger.info(
                    f"Waiting for target: position_error={pos_diff:.6f} m, "
                    f"rotation_error={rot_diff:.6f} rad"
                )
                last_log_time = time.monotonic()

            time.sleep(0.005)

        current = np.asarray(self.current_pose.copy(), dtype=float)
        self._logger.error(
            f"Target reach timeout: target={target_pose.tolist()}, "
            f"current={current.tolist()}"
        )
        return False

    def wait_until_joint_reached(
        self,
        target_joints,
        tol=0.01,
        timeout=10.0,
    ):
        target_joints = np.asarray(target_joints, dtype=float)
        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout:
            current = self.current_joint.copy()
            if np.all(current == 0):
                time.sleep(0.01)
                continue

            diff = float(np.linalg.norm(target_joints - current))
            if diff < tol:
                return True
            time.sleep(0.01)

        self._logger.error(
            f"Joint target timeout: target={target_joints.tolist()}, "
            f"current={self.current_joint.tolist()}"
        )
        return False