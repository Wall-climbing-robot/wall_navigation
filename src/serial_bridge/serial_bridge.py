#!/usr/bin/env python3
"""
serial_bridge — Nav2 cmd_vel → 串口 → Arduino

通信协议（文本行）：
  格式：L<left_mms> R<right_mms>\n
  示例：L500 R500\n    → 两轮全速前进
        L-500 R-500\n  → 两轮全速后退
        L0 R0\n        → 停车

硬件连接：
  上位机 USB (/dev/ttyACM0 等) → Arduino Serial2 (Pin16/17)

运行方式：
  python3 serial_bridge.py
  # 或通过 ROS2 参数覆盖：
  python3 serial_bridge.py --ros-args -p serial_port:=/dev/ttyUSB0 -p wheel_base:=0.25
"""

import glob
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


def _auto_detect_port() -> str:
    """按优先级自动查找串口设备。"""
    candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    return candidates[0] if candidates else "/dev/ttyUSB0"


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        # ── ROS2 参数（可通过 --ros-args -p 覆盖） ──
        self.declare_parameter("serial_port",  "auto")    # "auto" 自动检测
        self.declare_parameter("baudrate",     115200)    # 与 Arduino JETSON_BAUD 一致
        self.declare_parameter("wheel_base",   0.30)      # 左右轮距（米）
        self.declare_parameter("max_speed",    500)       # 最大速度（mm/s）
        self.declare_parameter("namespace",    "")        # 空字符串 → 订阅 /cmd_vel

        port_param = self.get_parameter("serial_port").value
        self.port       = _auto_detect_port() if port_param == "auto" else port_param
        self.baudrate   = self.get_parameter("baudrate").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.max_mms    = self.get_parameter("max_speed").value
        ns              = self.get_parameter("namespace").value

        self.ser: serial.Serial | None = None
        self._connect()

        cmd_vel_topic = f"/{ns}/cmd_vel" if ns else "/cmd_vel"
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.get_logger().info(f"订阅话题：{cmd_vel_topic}")

        # 定时检查串口是否断线（每 2 秒）
        self.create_timer(2.0, self._check_connection)

    # ────────────────────────────────────────
    # 串口连接 / 重连
    # ────────────────────────────────────────
    def _connect(self):
        try:
            # 先用 stty 重置 tty 流控状态（防止 Arduino 复位后 Linux 流控锁死）
            subprocess.run(
                ['stty', '-F', self.port, 'raw', '-echo', '-crtscts', '-ixon', '-ixoff',
                 'cs8', str(self.baudrate)],
                capture_output=True, timeout=2
            )
        except Exception:
            pass
        try:
            self.ser = serial.Serial(
                self.port, self.baudrate,
                timeout=0.1, write_timeout=1.0,
                rtscts=False, dsrdtr=False, xonxoff=False,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"[OK] 串口已连接：{self.port} @ {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"[ERR] 串口打开失败：{e}")
            self.ser = None

    def _check_connection(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("串口断线，尝试重连…")
            new_port = _auto_detect_port()
            if new_port != self.port:
                self.get_logger().info(f"检测到新端口：{new_port}")
                self.port = new_port
            self._connect()

    # ────────────────────────────────────────
    # cmd_vel 回调 → 编码并发送文本行
    # ────────────────────────────────────────
    def _on_cmd_vel(self, msg: Twist):
        vx = msg.linear.x    # 前进速度 m/s
        wz = msg.angular.z   # 转向角速度 rad/s

        # 差速轮换算：m/s → mm/s，钳位到 ±max_mms
        left_mms  = int(max(-self.max_mms,
                            min(self.max_mms, (vx - wz * self.wheel_base / 2.0) * 1000)))
        right_mms = int(max(-self.max_mms,
                            min(self.max_mms, (vx + wz * self.wheel_base / 2.0) * 1000)))

        # 死区补偿：绝对值小于 20 mm/s 的非零值直接置 0（电机启不动）
        DEAD_ZONE = 20
        if 0 < abs(left_mms) < DEAD_ZONE:
            left_mms = 0
        if 0 < abs(right_mms) < DEAD_ZONE:
            right_mms = 0

        line = f"L{right_mms} R{left_mms}\n"

        print(f"[serial_bridge] RX cmd_vel  vx={vx:+.3f} wz={wz:+.3f}  "
              f"→ L={left_mms:+5d} R={right_mms:+5d} mm/s", flush=True)

        if self.ser is None or not self.ser.is_open:
            print("[serial_bridge] 串口未连接，跳过发送", flush=True)
            return

        try:
            self.ser.write(line.encode())
            print(f"[serial_bridge] TX: {line.strip()}", flush=True)
        except Exception as e:
            print(f"[serial_bridge] 串口写入失败({type(e).__name__}): {e}", flush=True)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
