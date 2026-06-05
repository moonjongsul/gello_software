import rclpy
from rclpy.executors import MultiThreadedExecutor
import cv2
import numpy as np
import time
import threading
import os
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import functools

from Ur10_Controller_Node import Ur10_Controller_Node
from Realsense_Node import Realsense_Node

# ==========================================
# ⚙️ 1. 전역 설정 파라미터
# ==========================================
CALIB_FILE = "T_cam2base.npy"
APPROACH_OFFSET = 0.25
FIXED_CAM_DIST = 0.59
SAVE_PATH = "captured_data" # 📸 저장 폴더 이름


FAST_ACC, FAST_VEL = 0.6, 0.4    # 빠른 이동 (공중 이동용)
MID_ACC, MID_VEL = 0.6, 0.4      # 중간 이동 (접근용)
SLOW_ACC, SLOW_VEL = 0.3, 0.15   # 정밀 이동 (피킹/플레이스용)


# 폴더가 없으면 자동 생성
if not os.path.exists(SAVE_PATH):
    os.makedirs(SAVE_PATH)

ROIS = {
    "RIGHT": ((480, 200), (740, 720)),
    "CENTER": ((480, 200), (740, 720)),
    "LEFT": ((530, 200), (700, 720))
}

HOMES = {
    "RIGHT": [0.200, -0.500, 0.823, 0.0, 3.14, 0.0],
    "CENTER": [0.000, -0.500, 0.823, 0.0, 3.14, 0.0],
    "LEFT": [-0.200, -0.500, 0.823, 0.0, 3.14, 0.0]
}

PLACE_POSE = [0.370, -0.074, 0.530, 0.0, 3.14, 0.0]

state = {
    "current_zone": "RIGHT",
    "is_moving": False,
    "is_full_auto": False,
    "detected_circles": None
}

T_cam2tcp = None

# ==========================================
# 🤖 2. 자동화 실행 스레드 (속도 최적화 버전)
# ==========================================
def execute_pick_and_place(robot, target_xyz):
    if state["is_moving"]: return
    state["is_moving"] = True
    
    # --- [사용자 설정 파라미터] ---
    GRIPPER_LENGTH = 0.300
    PICK_Y_COR = 0.01
    
    try:
        current_pose = robot.get_pose()
        r_current = R.from_rotvec(current_pose[3:])
        R_tcp2base = r_current.as_matrix()

        gripper_vector_base = R_tcp2base @ np.array([0.0, 0.0, GRIPPER_LENGTH])
        r_z90 = R.from_euler('z', np.pi / 2)
        rotated_rotvec = (r_current * r_z90).as_rotvec()

        pick_target = np.array([target_xyz[0], target_xyz[1] + PICK_Y_COR, target_xyz[2]]) - gripper_vector_base
        pick_app_target = pick_target + np.array([0, 0, APPROACH_OFFSET])

        print(f"🤖 [MOTION] 피킹 시작 (속도 상향): {pick_target}")

        # 1. 상공 접근 (비교적 빠르게 이동)
        robot.move_to([*pick_app_target, *rotated_rotvec], acc=FAST_ACC, vel=FAST_VEL, gripper_state=30, wait=True)
        
        # 2. 하강 (정밀하게 이동)
        robot.move_to([*pick_target, *rotated_rotvec], acc=SLOW_ACC, vel=SLOW_VEL, wait=True)
        
        # 3. 잡기
        robot.set_gripper('close')
        time.sleep(0.8)

        # 4. 상승 (물체를 잡았으므로 중간 속도로 안전하게)
        robot.move_to([*pick_app_target, *rotated_rotvec], acc=MID_ACC, vel=MID_VEL, wait=True)

        # 5. 적재 위치 상공 이동 (빠르게)
        place_app = [PLACE_POSE[0], PLACE_POSE[1], PLACE_POSE[2] + APPROACH_OFFSET]
        robot.move_to([*place_app, *rotated_rotvec], acc=FAST_ACC, vel=FAST_VEL, wait=True)
        
        # 6. 적재 하강 및 놓기 (정밀하게)
        robot.move_to([*PLACE_POSE[:3], *rotated_rotvec], acc=SLOW_ACC, vel=SLOW_VEL, wait=True)
        robot.set_gripper('open')
        time.sleep(0.5)

        # 7. 복귀 시퀀스 (시원하게 복귀)
        robot.move_to([*place_app, *rotated_rotvec], acc=FAST_ACC, vel=FAST_VEL, wait=True)
        robot.move_to(HOMES[state["current_zone"]], acc=FAST_ACC, vel=FAST_VEL, wait=True)
        
        print("✅ [MOTION] 고속 시퀀스 완료")
    except Exception as e:
        print(f"❌ 모션 실행 에러: {e}")
    finally:
        state["is_moving"] = False

# ==========================================
# 🖱️ 3. 마우스 콜백 함수
# ==========================================
def on_mouse_click(event, x, y, flags, param, robot, vision):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        if state["is_moving"]:
            print("⚠️ [MOUSE] 로봇이 이미 동작 중입니다.")
            return

        print(f"\n🖱️ [MOUSE] 더블클릭 감지: ({x}, {y})")
        if state["is_full_auto"]:
            state["is_full_auto"] = False

        coord_cam = vision.get_3d_coordinate(x, y, FIXED_CAM_DIST)
        if coord_cam is None:
            params = vision.get_intrinsics()
            if params is None: return
            fx, fy, cx, cy = params
            coord_cam = np.array([(x - cx) * FIXED_CAM_DIST / fx, (y - cy) * FIXED_CAM_DIST / fy, FIXED_CAM_DIST])

        try:
            P_cam = np.array([*coord_cam, 1.0])
            curr_pose = robot.get_pose()
            T_tcp2base = np.eye(4)
            T_tcp2base[:3, :3] = R.from_rotvec(curr_pose[3:]).as_matrix()
            T_tcp2base[:3, 3] = curr_pose[:3]
            P_base = T_tcp2base @ T_cam2tcp @ P_cam
            target_xyz_base = P_base[:3]
            threading.Thread(target=execute_pick_and_place, args=(robot, target_xyz_base), daemon=True).start()
        except Exception as e:
            print(f" -> ❌ 좌표 변환 실패: {e}")

# ==========================================
# 🔍 4. 비전 로직 함수 (저장 기능 추가)
# ==========================================
def process_vision(color_img, zone, depth_img=None, save=False):
    if color_img is None: return None
    h, w = color_img.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    roi = ROIS[zone]
    cv2.rectangle(mask, roi[0], roi[1], 255, -1)
    
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    gray = cv2.bitwise_and(gray, mask)
    blurred = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=55, param1=60, param2=24, minRadius=18, maxRadius=35)
    
    res_circles = np.uint16(np.around(circles[0, :])) if circles is not None else None
    
    # 📸 자동 저장 로직 (물체를 찾았을 때만 저장)
    if save and res_circles is not None:
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        res_img = color_img.copy()
        cv2.rectangle(res_img, roi[0], roi[1], (255, 0, 0), 2)
        
        for i in res_circles:
            cv2.circle(res_img, (i[0], i[1]), i[2], (0, 255, 0), 2)
            
        cv2.imwrite(f"{SAVE_PATH}/{now}_color.png", color_img)
        cv2.imwrite(f"{SAVE_PATH}/{now}_result.png", res_img)
        if depth_img is not None:
            cv2.imwrite(f"{SAVE_PATH}/{now}_depth.png", depth_img)
        print(f"📸 [AUTO] 데이터 저장 완료: {now}")
        
    return res_circles

def main():
    global T_cam2tcp
    rclpy.init()
    robot = Ur10_Controller_Node()
    vision = Realsense_Node()
    try: T_cam2tcp = np.load(CALIB_FILE)
    except: return

    executor = MultiThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(vision)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    window_name = "Integrated Robot Vision"
    cv2.namedWindow(window_name)
    mouse_callback_with_params = functools.partial(on_mouse_click, robot=robot, vision=vision)
    cv2.setMouseCallback(window_name, mouse_callback_with_params)

    robot.move_to(HOMES["RIGHT"], wait=True)

    try:
        while rclpy.ok():
            frame = vision.get_color_image()
            depth_frame = getattr(vision, 'current_depth_frame', None) # 뎁스 프레임 획득
            if frame is None: continue
            display_img = frame.copy()

            if state["is_full_auto"] and not state["is_moving"]:
                # 📸 save=True 플래그와 depth_frame 전달
                circles = process_vision(frame, state["current_zone"], depth_frame, save=True)
                
                if circles is not None:
                    target_pixel = circles[0]
                    # 원본 연산 로직 변경 없이 그대로 유지
                    coord_cam = vision.get_3d_coordinate(target_pixel[0], target_pixel[1], FIXED_CAM_DIST)
                    if coord_cam is not None:
                        P_cam = np.array([*coord_cam, 1.0])
                        curr_pose = robot.get_pose()
                        T_tcp2base = np.eye(4)
                        T_tcp2base[:3, :3] = R.from_rotvec(curr_pose[3:]).as_matrix()
                        T_tcp2base[:3, 3] = curr_pose[:3]
                        P_base = T_tcp2base @ T_cam2tcp @ P_cam
                        threading.Thread(target=execute_pick_and_place, args=(robot, P_base[:3]), daemon=True).start()
                else:
                    zones = list(HOMES.keys())
                    curr_idx = zones.index(state["current_zone"])
                    if curr_idx < len(zones) - 1:
                        state["current_zone"] = zones[curr_idx + 1]
                        robot.move_to(HOMES[state["current_zone"]], wait=True)
                    else:
                        state["is_full_auto"] = False
                        state["current_zone"] = "RIGHT"
                        robot.move_to(HOMES["RIGHT"], wait=True)

            roi = ROIS[state["current_zone"]]
            cv2.rectangle(display_img, roi[0], roi[1], (255, 0, 0), 2)
            cv2.imshow(window_name, display_img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27: break
            elif key == ord('a'): 
                state["is_full_auto"] = True
                print("▶️ 자동 모드 활성화")
            elif key == ord('c'): 
                state["is_full_auto"] = False
                print("🛑 수동 모드 전환")
                
            # 1, 2, 3 키를 이용한 구역 변경 및 이동 (이동 중이 아닐 때만 가능)
            elif not state["is_moving"]:
                if key == ord('1'):
                    state["current_zone"] = "RIGHT"
                    print(f"📍 구역 변경: {state['current_zone']} (이동 중...)")
                    threading.Thread(target=robot.move_to, 
                                     args=(HOMES["RIGHT"],), 
                                     kwargs={'acc': FAST_ACC, 'vel': FAST_VEL, 'wait': True}).start()
                
                elif key == ord('2'):
                    state["current_zone"] = "CENTER"
                    print(f"📍 구역 변경: {state['current_zone']} (이동 중...)")
                    threading.Thread(target=robot.move_to, 
                                     args=(HOMES["CENTER"],), 
                                     kwargs={'acc': FAST_ACC, 'vel': FAST_VEL, 'wait': True}).start()
                
                elif key == ord('3'):
                    state["current_zone"] = "LEFT"
                    print(f"📍 구역 변경: {state['current_zone']} (이동 중...)")
                    threading.Thread(target=robot.move_to, 
                                     args=(HOMES["LEFT"],), 
                                     kwargs={'acc': FAST_ACC, 'vel': FAST_VEL, 'wait': True}).start()
    finally:
        executor.shutdown(); rclpy.shutdown(); cv2.destroyAllWindows()

if __name__ == '__main__':
    main()