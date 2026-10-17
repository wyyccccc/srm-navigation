import math
import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        # =========================
        # 参数
        # =========================

        # 真车时建议改成 /dev/serial/by-id/xxx
        self.serial_port = '/dev/ttyACM0'

        self.baudrate = 115200

        # 100 Hz
        self.send_period = 0.01

        # 超过 0.5 秒没收到 cmd_vel，自动停车
        self.watchdog_timeout = 0.5

        # 第一次实车建议最大 0.2
        self.max_vx = 0.2
        self.max_vy = 0.2
        self.max_wz = 0.2

        # 当前速度
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_cmd_time = time.time()

        # seq: uint8
        self.seq = 0

        # =========================
        # 打开串口
        # =========================

        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0
            )

            self.get_logger().info(
                f'Serial opened: {self.serial_port}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to open serial: {e}'
            )

            raise

        # =========================
        # 订阅 Nav2 cmd_vel
        # =========================

        self.subscription = self.create_subscription(
            Twist,
            '/red_standard_robot1/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 100 Hz 定时发送
        self.timer = self.create_timer(
            self.send_period,
            self.send_timer_callback
        )

        # 启动先发停车帧
        self.send_zero()

    # =========================
    # CRC8
    # =========================

    def crc8(self, data):

        crc = 0xFF

        for byte in data:

            crc ^= byte

            for _ in range(8):

                if crc & 0x01:
                    crc = (crc >> 1) ^ 0x8C
                else:
                    crc >>= 1

        return crc & 0xFF

    # =========================
    # CRC16
    # =========================

    def crc16(self, data):

        crc = 0xFFFF

        for byte in data:

            crc ^= byte

            for _ in range(8):

                if crc & 0x01:
                    crc = (crc >> 1) ^ 0x8408
                else:
                    crc >>= 1

        return crc & 0xFFFF

    # =========================
    # 限幅
    # =========================

    def clamp(self, value, limit):

        return max(
            -limit,
            min(limit, value)
        )

    # =========================
    # cmd_vel 回调
    # =========================

    def cmd_vel_callback(self, msg):

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # 防 NaN / Inf
        if not (
            math.isfinite(vx)
            and math.isfinite(vy)
            and math.isfinite(wz)
        ):

            self.get_logger().warning(
                'Invalid cmd_vel, sending zero'
            )

            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0

            return

        # =========================
        # 坐标转换
        # =========================
        #
        # 协议说明：
        #
        # 上位机 vx -> 底盘 y
        # 上位机 vy -> 底盘 -x
        #
        # 所以这里先做映射
        #

        chassis_vx = -vy
        chassis_vy = vx

        # wz 当前下位机可能不使用
        chassis_wz = wz

        self.vx = self.clamp(
            chassis_vx,
            self.max_vx
        )

        self.vy = self.clamp(
            chassis_vy,
            self.max_vy
        )

        self.wz = self.clamp(
            chassis_wz,
            self.max_wz
        )

        self.last_cmd_time = time.time()

    # =========================
    # 构造 19 字节帧
    # =========================

    def build_frame(self, vx, vy, wz):

        frame = bytearray()

        # byte 0
        frame.append(0xA5)

        # byte 1-2
        # data_length = 12
        frame += struct.pack(
            '<H',
            12
        )

        # byte 3
        frame.append(
            self.seq
        )

        # byte 4
        # CRC8 校验前 4 字节
        crc8_value = self.crc8(
            frame
        )

        frame.append(
            crc8_value
        )

        # byte 5-16
        # 3 个 float32
        frame += struct.pack(
            '<fff',
            vx,
            vy,
            wz
        )

        # CRC16 校验 byte 0-16
        crc16_value = self.crc16(
            frame
        )

        # byte 17-18
        # little endian
        frame += struct.pack(
            '<H',
            crc16_value
        )

        # seq +1
        self.seq = (
            self.seq + 1
        ) & 0xFF

        return frame

    # =========================
    # 定时发送
    # =========================

    def send_timer_callback(self):

        now = time.time()

        # watchdog
        if (
            now - self.last_cmd_time
            > self.watchdog_timeout
        ):

            vx = 0.0
            vy = 0.0
            wz = 0.0

        else:

            vx = self.vx
            vy = self.vy
            wz = self.wz

        frame = self.build_frame(
            vx,
            vy,
            wz
        )

        try:

            self.ser.write(
                frame
            )

        except Exception as e:

            self.get_logger().error(
                f'Serial write failed: {e}'
            )

    # =========================
    # 停车
    # =========================

    def send_zero(self):

        frame = self.build_frame(
            0.0,
            0.0,
            0.0
        )

        try:

            # 多发几帧保险
            for _ in range(5):

                self.ser.write(
                    frame
                )

                time.sleep(
                    0.01
                )

        except Exception:

            pass

    # =========================
    # 销毁节点
    # =========================

    def destroy_node(self):

        self.get_logger().info(
            'Stopping robot...'
        )

        try:

            self.send_zero()

        except Exception:

            pass

        try:

            self.ser.close()

        except Exception:

            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = SerialBridge()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        if node is not None:

            node.send_zero()

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()
