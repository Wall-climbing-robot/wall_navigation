#!/usr/bin/env python3
"""
Nav2 cmd_vel → 串口 → Arduino ATmega2560
协议：[0xAA][0x55][左速高][左速低][右速高][右速低][CRC8]
速度单位：mm/s，int16有符号（正=前进，负=后退）
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial, struct

# ===== 你需要改的参数 =====
SERIAL_PORT = '/dev/ttyUSB0'  # 查询：ls /dev/ttyUSB* 或 ls /dev/ttyACM*
BAUDRATE    = 115200           # Arduino Serial1的波特率，要和Arduino代码一致
WHEEL_BASE  = 0.30            # 左右轮距（米），用卷尺量
MAX_MMS     = 500             # 最大速度mm/s，地面500，上墙改200
# ==========================

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            self.get_logger().info(f'串口已连接：{SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败：{e}，请检查接线和端口号')
            raise
        self.create_subscription(Twist, '/red_standard_robot1/cmd_vel',
                                 self.on_cmd_vel, 10)

    def on_cmd_vel(self, msg):
        vx = msg.linear.x   # 前进速度 m/s
        wz = msg.angular.z  # 转向角速度 rad/s

        # 差速公式转换，m/s → mm/s
        left_mms  = int(max(-MAX_MMS, min(MAX_MMS, (vx - wz * WHEEL_BASE / 2.0) * 1000)))
        right_mms = int(max(-MAX_MMS, min(MAX_MMS, (vx + wz * WHEEL_BASE / 2.0) * 1000)))

        payload = struct.pack('>hh', left_mms, right_mms)  # big-endian int16
        frame   = bytes([0xAA, 0x55]) + payload + bytes([self.crc8(payload)])
        self.ser.write(frame)

    def crc8(self, data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
        return crc

def main():
    rclpy.init()
    rclpy.spin(SerialBridge())

if __name__ == '__main__':
    main()