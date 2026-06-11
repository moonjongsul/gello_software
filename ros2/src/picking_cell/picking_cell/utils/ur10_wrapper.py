from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import time
import threading
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

        # Motion guard. The pick and movej services share a ReentrantCallbackGroup,
        # so the executor may dispatch a second motion command while one is still
        # running. _motion_lock is acquired non-blockingly at the start of every
        # motion command (movej/movel_rel); if it's already held, the new command
        # is rejected so we never stream two URScript moves at once.
        self._motion_lock = threading.Lock()

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
            self.gripper = minimalmodbus.Instrument(self.cfg.gripper.usb_port, 1, 'rtu')
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.bytesize = 8
            self.gripper.serial.parity = serial.PARITY_NONE
            self.gripper.serial.stopbits = 1
            self.gripper.serial.timeout = 0.5
            self.gripper.address = 1
            self.gripper.mode = minimalmodbus.MODE_RTU
            self.gripper.clear_buffers_before_each_transaction = True
            self._init_gripper()
            self._logger.info(f"Gripper initialized on {self.cfg.gripper.usb_port}")
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
    
    def movej(self, loc=None, acc=1.0, vel=0.5, r=0, wait=False):
        if not self._motion_lock.acquire(blocking=False):
            self._logger.warn("movej rejected: robot is already moving")
            return False
        try:
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
            if wait: return self.wait_until_joint_reached(np.array(rads))
            return True
        finally:
            self._motion_lock.release()
    
    def movel(self, loc, acc=1.0, vel=0.5, wait=False):
        """Linear move to an absolute Cartesian TCP pose in the base frame.

        loc: either a 6-element [x, y, z, rx, ry, rz] pose (translation in
        metres, rotation as a rotation vector in rad, matching current_pose's
        format) or the name of a Cartesian pose defined under cfg.pose. Unlike
        movej, rotations are always treated as a rotation vector (rad) — the
        degree heuristic does not apply to Cartesian poses.
        """
        if not self._motion_lock.acquire(blocking=False):
            self._logger.warn("movel rejected: robot is already moving")
            return False
        try:
            target = []
            for pose, value in vars(self.cfg.pose).items():
                if loc == pose:
                    target = value
            if not target:
                target = loc

            target = np.asarray(target, dtype=float)
            if target.shape[0] != 6:
                self._logger.warn(f"movel: expected 6-element pose, got {target}")
                return False

            script = f"movel(p[{','.join(map(str, target))}], a={acc}, v={vel}, r=0)"
            self._publish(script)
            if wait: return self.wait_until_reached(target)
            return True
        finally:
            self._motion_lock.release()


    def _resolve_joint(self, loc):
        """Resolve a movej target to a radian joint vector (named pose or raw).

        Mirrors movej: named poses are looked up in cfg.pose, and a raw target
        whose mean magnitude exceeds 5 is treated as degrees and converted.
        Returns a list of 6 radians, or None if unresolved.
        """
        target = []
        for pose, joints in vars(self.cfg.pose).items():
            if loc == pose:
                target = joints
        if not target:
            target = loc
        target = list(np.asarray(target, dtype=float))
        if len(target) != 6:
            return None
        verify = np.mean(np.array(target))
        if verify > 5 or verify < -5:
            return [math.radians(d) for d in target]
        return target

    def _resolve_cart(self, loc):
        """Resolve a movel target to a 6-element Cartesian pose (named or raw).

        Named poses are looked up in cfg.pose. Returns a numpy array of 6, or
        None if unresolved.
        """
        target = []
        for pose, value in vars(self.cfg.pose).items():
            if loc == pose:
                target = value
        if not (len(target) if isinstance(target, (list, tuple)) else target):
            target = loc
        target = np.asarray(target, dtype=float)
        if target.shape[0] != 6:
            return None
        return target

    def move_blend(self, waypoints, acc=1.0, vel=0.5, blend=0.05, wait=False):
        """Blended move through a list of waypoints, mixing movel and movej.

        waypoints: a list of waypoints. Each waypoint is either

          ('cs', target) -> movel: target is a 6-element Cartesian pose
                            [x, y, z, rx, ry, rz] (metres + rotvec rad) or the
                            name of a Cartesian pose in cfg.pose.
          ('js', target) -> movej: target is a 6-element joint vector (rad, or
                            degrees if its mean magnitude > 5) or the name of a
                            joint pose in cfg.pose.

        A bare 6-element list (no type tag) is treated as ('cs', target) for
        backwards compatibility.

        All waypoints are streamed as a single URScript program so the
        controller blends across them — including movej<->movel transitions.
        Every intermediate waypoint gets blend radius r=blend so the robot
        carries speed through it; the final waypoint always uses r=0 so it stops
        exactly on target.

        blend: blend radius in metres. A single float (applied to all
        intermediate waypoints) or a per-waypoint list (len == waypoints; the
        last entry is forced to 0).

        Note: keep blend smaller than half the shortest segment length, or the
        controller will reject overlapping blends.
        """
        if not self._motion_lock.acquire(blocking=False):
            self._logger.warn("move_blend rejected: robot is already moving")
            return False
        try:
            parsed = []  # (kind, resolved_target) per waypoint
            for wp in waypoints:
                # Tagged waypoint: ('cs'|'js', target). A bare 6-vector or pose
                # name defaults to a Cartesian (linear) move.
                if isinstance(wp, (tuple, list)) and len(wp) == 2 \
                        and isinstance(wp[0], str) and wp[0] in ('cs', 'js'):
                    kind, raw = wp[0], wp[1]
                else:
                    kind, raw = 'cs', wp

                if kind == 'js':
                    target = self._resolve_joint(raw)
                else:
                    target = self._resolve_cart(raw)
                if target is None:
                    self._logger.warn(f"move_blend: bad {kind}-waypoint {wp!r}")
                    return False
                parsed.append((kind, target))

            if not parsed:
                self._logger.warn("move_blend: empty waypoint list")
                return False

            n = len(parsed)
            if isinstance(blend, (list, tuple, np.ndarray)):
                radii = [float(b) for b in blend]
                if len(radii) != n:
                    self._logger.warn(
                        f"move_blend: blend list len {len(radii)} != {n} waypoints")
                    return False
            else:
                radii = [float(blend)] * n
            # The final waypoint must stop exactly on target.
            radii[-1] = 0.0

            lines = []
            for (kind, target), r in zip(parsed, radii):
                vals = ','.join(map(str, target))
                if kind == 'js':
                    lines.append(f"  movej([{vals}], a={acc}, v={vel}, r={r})")
                else:
                    lines.append(f"  movel(p[{vals}], a={acc}, v={vel}, r={r})")
            # Wrap in a def so the whole sequence is sent and executed as one
            # program rather than line-by-line, which would break blending.
            script = "def blended_path():\n" + "\n".join(lines) + "\nend"
            self._publish(script)
            if wait:
                # Wait on the final waypoint, in its own space: Cartesian pose
                # vs. joint vector.
                last_kind, last_target = parsed[-1]
                if last_kind == 'js':
                    return self.wait_until_joint_reached(np.asarray(last_target))
                return self.wait_until_reached(last_target)
            return True
        finally:
            self._motion_lock.release()

    def movel_rel(self, delta, frame='base', acc=0.2, vel=0.1, gripper_state=None, wait=True):
        """Linear move relative to the current TCP pose (Python-computed target).

        delta: [dx, dy, dz, drx, dry, drz] offset. Translation in metres,
        rotation as a rotation vector (rad), matching current_pose's format.
        frame='base' translates/rotates in the base frame; frame='tool'
        applies the offset in the current TCP frame.

        The absolute target pose is computed here from current_pose so the
        existing wait_until_reached() can verify arrival.
        """
        if not self._motion_lock.acquire(blocking=False):
            self._logger.warn("movel_rel rejected: robot is already moving")
            return False
        try:
            current = self.current_pose
            if np.all(current[:3] == 0):
                self._logger.warn("movel_rel: current_pose not received yet")
                return False

            delta = np.asarray(delta, dtype=float)
            cur_rot = R.from_rotvec(current[3:])
            delta_rot = R.from_rotvec(delta[3:])

            if frame == 'tool':
                # Offset expressed in the TCP frame: rotate translation into base,
                # then compose rotations on the right (tool-local).
                target_pos = current[:3] + cur_rot.apply(delta[:3])
                target_rot = cur_rot * delta_rot
            else:  # 'base'
                # Offset expressed in the base frame: add translation directly,
                # compose rotations on the left (base-global).
                target_pos = current[:3] + delta[:3]
                target_rot = delta_rot * cur_rot

            target_pose = np.concatenate([target_pos, target_rot.as_rotvec()])

            script = f"movel(p[{','.join(map(str, target_pose))}], a={acc}, v={vel}, r=0)"
            self._publish(script)
            if gripper_state is not None: self.set_gripper(gripper_state)
            if wait: return self.wait_until_reached(target_pose)
            return True
        finally:
            self._motion_lock.release()

    def wait_until_reached(self, target_pose, pos_tol=0.002, rot_tol=0.02, timeout=8.0):
        # The owning node is spun by a MultiThreadedExecutor on its own
        # thread(s), so current_pose is updated concurrently. Do NOT spin_once
        # here: this runs inside a service callback and re-entering the executor
        # raises "cannot spin while already spinning". Just poll the cached pose.
        start_time = time.time()
        stable_start_time = None
        while (time.time() - start_time) < timeout:
            current = self.current_pose.copy()
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
        # Same as wait_until_reached: the executor thread keeps current_joint
        # fresh, so poll the cache instead of re-entering the executor here.
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            current = self.current_joint.copy()
            if np.all(current == 0): continue
            diff = np.linalg.norm(target_joints - current)
            if diff < tol: return True
            time.sleep(0.01)
        return False