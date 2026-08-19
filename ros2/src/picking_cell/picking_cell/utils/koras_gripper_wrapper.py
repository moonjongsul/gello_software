import time
import serial
import minimalmodbus

# 그리퍼 제어용 레지스터 명령어 상수
GRP_INIT = 101
GRP_OPEN = 102
GRP_CLOSE = 103
GRP_POS_CTRL = 104

class GripperControlWrapper:
    def __init__(self, usb_port, logger=None):
        self.usb_port = usb_port
        self._logger = logger
        self.gripper = None
        
        self._load_gripper()

    def _log_info(self, msg):
        if self._logger:
            self._logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def _log_warn(self, msg):
        if self._logger:
            self._logger.warn(msg)
        else:
            print(f"[WARN] {msg}")

    def _load_gripper(self):
        try:
            self.gripper = minimalmodbus.Instrument(self.usb_port, 1, 'rtu')
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.bytesize = 8
            self.gripper.serial.parity = serial.PARITY_NONE
            self.gripper.serial.stopbits = 1
            self.gripper.serial.timeout = 0.5
            self.gripper.address = 1
            self.gripper.mode = minimalmodbus.MODE_RTU
            self.gripper.clear_buffers_before_each_transaction = True
            
            self._init_gripper()
            self._log_info(f"Gripper initialized on {self.usb_port}")
        except Exception as e:
            self._log_warn(f"Gripper init failed: {e}")
            self.gripper = None
    
    def _init_gripper(self):
        if self.gripper is None: 
            return False
        success = self._send_gripper([GRP_INIT])
        time.sleep(1) # 초기화 후 대기
        return success

    def _send_gripper(self, opt):
        if self.gripper is None: 
            return False
        try:
            # 레지스터 주소 0번에 opt 리스트 전송
            self.gripper.write_registers(0, opt)
            return True
        except Exception as e:
            self._log_warn(f"Gripper command failed: {e}")
            return False

    def set_gripper(self, state):
        """
        상태에 따라 그리퍼를 동작시킵니다.
        state: 'open', 'close', 또는 위치값(int/float)
        """
        if self.gripper is None: 
            self._log_warn("Cannot set gripper: Not initialized.")
            return False
            
        if isinstance(state, str):
            if state.lower() == 'open': 
                return self._send_gripper([GRP_OPEN])
            elif state.lower() == 'close': 
                return self._send_gripper([GRP_CLOSE])
            elif state.lower() == 'init':
                return self._init_gripper()
        elif isinstance(state, (int, float)):
            pos = int(state * 5.0)
            return self._send_gripper([GRP_POS_CTRL, pos])
        
        return False