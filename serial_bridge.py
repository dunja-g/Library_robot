"""
Library Robot — Serial Bridge
Person 1: Pi ↔ Arduino 串口通信
"""

import serial
import time
import logging

logger = logging.getLogger(__name__)


class SerialBridge:
    """封装所有 Arduino 串口指令"""

    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)
        time.sleep(2)  # 等 Arduino 重启
        self.ser.reset_input_buffer()
        logger.info(f"SerialBridge ready on {port}")

    def _send(self, cmd):
        self.ser.write((cmd + '\n').encode())
        self.ser.flush()

    def send_forward(self):
        self._send("FORWARD")

    def send_backward(self):
        self._send("BACKWARD")

    def send_rotate_left(self):
        self._send("ROTATE_LEFT")

    def send_rotate_right(self):
        self._send("ROTATE_RIGHT")

    def send_stop(self):
        self._send("STOP")

    def get_ultrasonic(self):
        """返回 {"left": float, "center": float, "right": float} 或 None"""
        self._send("CHECK")
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line.startswith("US:"):
            logger.warning(f"Unexpected CHECK response: {line}")
            return None
        try:
            parts = line[3:].split(',')
            return {
                "left":   float(parts[0]),
                "center": float(parts[1]),
                "right":  float(parts[2]),
            }
        except (ValueError, IndexError) as e:
            logger.warning(f"Parse error: {e} — {line}")
            return None

    def close(self):
        self._send("STOP")
        self.ser.close()


# ============================================================
if __name__ == '__main__':
    # 独立测试
    logging.basicConfig(level=logging.INFO)
    bridge = SerialBridge()

    print("FORWARD 1s")
    bridge.send_forward()
    time.sleep(1)

    print("STOP")
    bridge.send_stop()
    time.sleep(0.5)

    print("ROTATE_LEFT 1s")
    bridge.send_rotate_left()
    time.sleep(1)

    print("STOP")
    bridge.send_stop()
    time.sleep(0.5)

    print("CHECK ultrasonic...")
    result = bridge.get_ultrasonic()
    print(f"  -> {result}")

    bridge.close()
    print("Test done.")
