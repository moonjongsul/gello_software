#!/usr/bin/env python3
from types import SimpleNamespace
import json, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from picking_cell_interfaces.srv import MoveJ, MoveL, MoveT, GripperWidth
from picking_cell.utils.realsense_wrapper import RealSenseWrapper
from picking_cell.utils.ur10_wrapper import Ur10ControlWrapper
from picking_cell.detector.tray_object_detector import TrayObjectDetector

APPROACH_HEIGHT = 0.05      # 처음 물체 위 5cm까지 접근
FINAL_CLEARANCE = 0.005     # 최종 파지 시 5mm 여유, 완전 하강하려면 0.0
LIFT_HEIGHT = 0.10          # 파지 후 10cm 상승
GRIP_WAIT = 1.0

class TrayPicking(Node):
    def __init__(self):
        super().__init__('tray_picking', automatically_declare_parameters_from_overrides=True)
        self.cfg = self._load_cfg()
        self.ur10 = Ur10ControlWrapper(self.cfg.robot, node=self)
        self.camera = RealSenseWrapper(self.cfg.camera, node=self)
        self.T_cam2tcp = np.array(self.cfg.camera.T_cam2tcp, dtype=np.float64).reshape(4, 4)
        self.robot_acc = self.cfg.robot.speed.acc
        self.robot_vel = self.cfg.robot.speed.vel
        self.tray_detector = TrayObjectDetector()
        self.cb_group = ReentrantCallbackGroup()

        self.pick_srv = self.create_service(Trigger, self.cfg.vision.service_name[0], self.srv_pick_cb, callback_group=self.cb_group)
        self.movej_srv = self.create_service(MoveJ, self.cfg.vision.service_name[1], self.srv_movej_cb, callback_group=self.cb_group)
        self.movel_srv = self.create_service(MoveL, self.cfg.vision.service_name[2], self.srv_movel_cb, callback_group=self.cb_group)
        self.movet_srv = self.create_service(MoveT, self.cfg.vision.service_name[3], self.srv_movet_cb, callback_group=self.cb_group)
        self.gripper_srv = self.create_service(GripperWidth, self.cfg.vision.service_name[4], self.srv_gripper_cb, callback_group=self.cb_group)

        self.pub_vis_output = self.create_publisher(CompressedImage, self.cfg.vision.publish_list[0].split(':')[1], 10)
        self.get_logger().info(f"Tray-picking service ready at '{self.cfg.vision.service_name}'")
        self.get_logger().info(f"3x4 TrayObjectDetector 초기화 완료 / 접근높이={APPROACH_HEIGHT*100:.1f}cm / 최종여유={FINAL_CLEARANCE*1000:.0f}mm")

    def srv_pick_cb(self, request, response):
        try:
            ok = self._run_pick_cycle()
            response.success = bool(ok)
            response.message = 'pick done' if ok else 'pick failed'
        except Exception as error:
            self.get_logger().error(f"Pick cycle error: {error}")
            response.success = False
            response.message = str(error)
        return response

    def srv_movej_cb(self, request, response):
        try:
            result = self.ur10.movej(request.location, wait=True)
            response.success = True if result is None else bool(result)
            response.message = f"MoveJ completed: {request.location}" if response.success else f"MoveJ failed: {request.location}"
        except Exception as error:
            self.get_logger().error(f"MoveJ error: {error}")
            response.success = False
            response.message = str(error)
        return response

    def srv_movel_cb(self, request, response):
        try:
            target_pose = json.loads(request.location)
            if not isinstance(target_pose, list) or len(target_pose) != 6:
                raise ValueError("MoveL 좌표는 [x,y,z,rx,ry,rz] 6개여야 합니다.")
            target_pose = [float(v) for v in target_pose]
            result = self.ur10.movel(target_pose, acc=self.robot_acc, vel=self.robot_vel, wait=True)
            response.success = True if result is None else bool(result)
            response.message = f"MoveL completed: {target_pose}" if response.success else f"MoveL failed: {target_pose}"
        except Exception as error:
            self.get_logger().error(f"MoveL error: {error}")
            response.success = False
            response.message = str(error)
        return response

    def srv_movet_cb(self, request, response):
        try:
            payload = json.loads(request.location)
            frame = "tool"

            if isinstance(payload, dict):
                delta = payload.get("delta", payload.get("location"))
                frame = str(payload.get("frame", "tool")).strip().lower()
            else:
                delta = payload

            if not isinstance(delta, list) or len(delta) != 6:
                raise ValueError("MoveT 변화량은 [dx,dy,dz,drx,dry,drz] 6개여야 합니다.")
            if frame not in ("tool", "base"):
                raise ValueError("MoveT frame은 'tool' 또는 'base'여야 합니다.")

            delta = [float(v) for v in delta]
            translation_distance = float(np.linalg.norm(delta[:3]))
            rotation_distance = float(np.linalg.norm(delta[3:]))

            if translation_distance > 0.50:
                raise ValueError(f"MoveT 단일 이동 거리가 너무 큽니다: {translation_distance:.3f}m")
            if rotation_distance > np.pi + 1e-6:
                raise ValueError(f"MoveT 단일 회전량이 너무 큽니다: {rotation_distance:.3f}rad")

            result = self.ur10.movet(delta, frame=frame, acc=self.robot_acc, vel=self.robot_vel, wait=True)
            response.success = True if result is None else bool(result)
            response.message = f"MoveT completed: delta={delta}, frame={frame}" if response.success else f"MoveT failed: delta={delta}, frame={frame}"
        except Exception as error:
            self.get_logger().error(f"MoveT error: {error}")
            response.success = False
            response.message = str(error)
        return response

    def srv_gripper_cb(self, request, response):
        width = request.width
        try:
            width = int(width)
        except Exception:
            width = str(width)

        try:
            self.ur10.set_gripper(width)
            response.success = True
            response.message = f"gripper width: {width}"
        except Exception as error:
            self.get_logger().error(f"Gripper error: {error}")
            response.success = False
            response.message = str(error)
        return response

    def detect_tray_objects(self, color):
        if color is None:
            raise ValueError("카메라 Color 이미지가 None입니다.")

        result_image, detections = self.tray_detector.process(color)
        detected_objects = [d for d in detections if d.get("detected", False)]
        detected_objects.sort(key=lambda d: int(d["id"]))

        direct_count = len(detected_objects)
        inferred_count = len(detections) - direct_count

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Objects: {direct_count}/12 / inferred={inferred_count}")

        for d in detections:
            source = "DIRECT" if d["detected"] else "INFERRED"
            self.get_logger().info(f'ID={d["id"]:2d} row={d["row"]} col={d["column"]} pixel=({d["image_x"]:.1f},{d["image_y"]:.1f}) {source}')

        self.get_logger().info("=" * 60)

        if not detected_objects:
            return None, result_image, detections

        target = detected_objects[0]
        self.get_logger().info(f'선택 물체: ID={target["id"]}, pixel=({target["image_x"]:.1f},{target["image_y"]:.1f})')
        return target, result_image, detections

    def _run_pick_cycle(self):
        # 1. Home
        self.get_logger().info("[1] Home 이동")
        result = self.ur10.movej('home', acc=self.robot_acc * 1.2, vel=self.robot_vel, wait=True)
        if result is False:
            self.get_logger().warn("Home 이동 실패")
            return False
        time.sleep(0.5)

        # 2. Camera
        self.get_logger().info("[2] 카메라 이미지 획득")
        color, depth = self.camera.get_data()
        if color is None:
            self.get_logger().warn("Color image가 없습니다.")
            return False

        # 3. Detection
        self.get_logger().info("[3] 3x4 트레이 물체 인식")
        target, vis_output, detections = self.detect_tray_objects(color)

        vis_output_msg = self.arr_to_compressed_msg(vis_output)
        if vis_output_msg is not None:
            self.pub_vis_output.publish(vis_output_msg)

        if target is None:
            self.get_logger().warn("직접 검출된 물체가 없습니다.")
            return False

        # 4. Target pixel
        target_x = float(target["image_x"])
        target_y = float(target["image_y"])
        target_id = int(target["id"])
        self.get_logger().info(f"[4] Pick 대상: ID={target_id}, pixel=({target_x:.1f},{target_y:.1f})")

        # 5. Pixel -> Camera XYZ
        coord_cam = self.camera.get_3d_coordinate(target_x, target_y, self.cfg.vision.obj2cam)
        if coord_cam is None:
            self.get_logger().warn("3D 좌표 계산 실패")
            return False

        # 6. Camera -> TCP
        P_cam = np.array([float(coord_cam[0]), float(coord_cam[1]), float(coord_cam[2]), 1.0], dtype=np.float64)
        P_tcp = self.T_cam2tcp @ P_cam
        dx, dy, dz = float(P_tcp[0]), float(P_tcp[1]), float(P_tcp[2])

        self.get_logger().info(f"P_cam=[{P_cam[0]:.4f},{P_cam[1]:.4f},{P_cam[2]:.4f}] P_tcp=[{dx:.4f},{dy:.4f},{dz:.4f}]")

        # 7. 그리퍼 열기
        self.get_logger().info("[5] 그리퍼 열기")
        self.ur10.set_gripper(self.cfg.robot.gripper.pick_width)
        time.sleep(0.3)

        # 8. XY centering
        move_dx = dx + float(self.cfg.robot.gripper.pick_offset.x)
        move_dy = dy + float(self.cfg.robot.gripper.pick_offset.y)

        self.get_logger().info(f"[6] 물체 중심 XY 이동: dx={move_dx:.4f}, dy={move_dy:.4f}")

        result = self.ur10.movel_rel([move_dx, move_dy, 0.0, 0.0, 0.0, 0.0], frame='tool',
                                    acc=self.robot_acc * 0.5, vel=self.robot_vel * 0.3, wait=True)
        if result is False:
            self.get_logger().warn("XY centering 이동 실패")
            return False

        # 9. 물체 위 5cm까지 접근
        obj2tool = float(self.cfg.vision.obj2tool)
        approach_descend = obj2tool - APPROACH_HEIGHT

        if approach_descend <= 0:
            self.get_logger().error(f"접근 거리 오류: obj2tool={obj2tool:.3f}, approach={APPROACH_HEIGHT:.3f}")
            return False

        self.get_logger().info(f"[7] 물체 위 {APPROACH_HEIGHT*100:.1f}cm까지 하강: {approach_descend:.3f}m")


        GRIPPER_ROTATION = -1.4708
        result = self.ur10.movel_rel([0.0, 0.0, approach_descend, 0.0, 0.0, GRIPPER_ROTATION], frame='tool',
                                    acc=self.robot_acc * 0.3, vel=self.robot_vel * 0.2, wait=True)
        if result is False:
            self.get_logger().warn("접근 하강 실패")
            return False

        # 10. 마지막 천천히 하강
        final_descend = APPROACH_HEIGHT - FINAL_CLEARANCE

        if final_descend <= 0:
            self.get_logger().error("최종 하강 거리가 0 이하입니다.")
            return False

        self.get_logger().warn("=" * 60)
        self.get_logger().warn("[8] 최종 파지 위치 접근")
        self.get_logger().warn(f"추가 하강={final_descend:.3f}m")
        self.get_logger().warn(f"남기는 여유={FINAL_CLEARANCE*1000:.0f}mm")
        self.get_logger().warn("=" * 60)

        result = self.ur10.movel_rel([0.0, 0.0, final_descend, 0.0, 0.0, 0], frame='tool',
                                    acc=self.robot_acc * 0.15, vel=self.robot_vel * 0.08, wait=True)
        if result is False:
            self.get_logger().warn("최종 하강 실패")
            return False

        # 11. Grip
        self.get_logger().info("[9] 그리퍼 파지")
        self.ur10.set_gripper(1)
        time.sleep(GRIP_WAIT)

        # 12. Lift
        self.get_logger().info(f"[10] 물체 {LIFT_HEIGHT*100:.1f}cm 상승")

        
        # 현재 tool +Z가 하강 방향이므로 상승은 -Z
        result = self.ur10.movel_rel([0.0, 0.0, -LIFT_HEIGHT, 0.0, 0.0, 0], frame='tool',
                                    acc=self.robot_acc * 0.3, vel=self.robot_vel * 0.2, wait=True)
        if result is False:
            self.get_logger().warn("파지 후 상승 실패")
            return False

        # 13. Drop 위치
        self.get_logger().info("[11] Drop 위치 이동")

        result = self.ur10.move_blend([
            ('cs', [0.400, -0.335, 0.750, 2.221, 2.221, 0.0]),
            ('cs', [0.630, -0.280, 0.550, 1.939, 1.939, 0.52])
        ], acc=self.robot_acc * 0.7, vel=self.robot_vel * 0.7, wait=True)

        if result is False:
            self.get_logger().warn("Drop 위치 이동 실패")
            return False

        # 14. Release
        self.get_logger().info("[12] 물체 놓기")
        self.ur10.set_gripper(self.cfg.robot.gripper.pick_width)
        time.sleep(0.5)

        # 15. Home
        self.get_logger().info("[13] Home 복귀")

        result = self.ur10.move_blend([
            ('cs', [0.400, -0.335, 0.750, 2.221, 2.221, 0.0]),
            ('js', 'home')
        ], acc=self.robot_acc, vel=self.robot_vel, wait=True)

        if result is False:
            self.get_logger().warn("Home 복귀 실패")
            return False

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"✅ Pick 완료: ID={target_id}")
        self.get_logger().info("=" * 60)

        return True

    def arr_to_compressed_msg(self, arr):
        if arr is None:
            return None

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        ok, buf = cv2.imencode('.jpg', arr)

        if not ok:
            self.get_logger().warn("Failed to encode detection image")
            return None

        msg.data = buf.tobytes()
        return msg

    def _load_cfg(self):
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
        if isinstance(obj, dict):
            return SimpleNamespace(**{key: cls._to_namespace(value) for key, value in obj.items()})
        if isinstance(obj, list):
            return [cls._to_namespace(item) for item in obj]
        return obj


def main(args=None):
    rclpy.init(args=args)
    node = TrayPicking()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("사용자가 종료했습니다.")
    except Exception as error:
        node.get_logger().error(f"Launch error: {error}")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()