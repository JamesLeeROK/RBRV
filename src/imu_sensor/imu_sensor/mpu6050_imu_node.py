#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time
import math

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

DEG2RAD = math.pi / 180.0
G = 9.80665

class Mpu6050ImuNode(Node):
    def __init__(self):
        super().__init__('mpu6050_imu_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('calib_samples', 200)

        self.bus = smbus2.SMBus(self.get_parameter('i2c_bus').value)
        self.addr = self.get_parameter('i2c_addr').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0)
        time.sleep(0.1)

        self.gyro_bias = [0.0, 0.0, 0.0]
        self._calibrate_gyro()

        rate = self.get_parameter('publish_rate').value
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)
        self.get_logger().info('MPU-6050 IMU 노드 시작(신뢰도 강화)')

    def _calibrate_gyro(self):
        n = self.get_parameter('calib_samples').value
        self.get_logger().info(f'자이로 캘리브레이션 중... ({n}개 샘플, 센서를 가만히 두세요)')
        sx = sy = sz = 0.0
        ok = 0
        for _ in range(n):
            try:
                gx, gy, gz = self._read_gyro_raw()
                sx += gx
                sy += gy
                sz += gz
                ok += 1
            except OSError:
                pass
            time.sleep(0.005)
        if ok > 0:
            self.gyro_bias = [sx / ok, sy / ok, sz / ok]
        self.get_logger().info(f'캘리브레이션 완료: bias={self.gyro_bias}')

    def _read_word(self, data, i):
        val = (data[i] << 8) + data[i + 1]
        return val - 65536 if val >= 32768 else val

    def _read_gyro_raw(self):
        data = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        gx = self._read_word(data, 8) / 131.0
        gy = self._read_word(data, 10) / 131.0
        gz = self._read_word(data, 12) / 131.0
        return gx, gy, gz  # deg/s

    def timer_cb(self):
        try:
            data = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        except OSError as e:
            self.get_logger().warn(f'I2C 읽기 실패: {e}')
            return

        ax = self._read_word(data, 0) / 16384.0 * G
        ay = self._read_word(data, 2) / 16384.0 * G
        az = self._read_word(data, 4) / 16384.0 * G

        gx = self._read_word(data, 8) / 131.0 - self.gyro_bias[0]
        gy = self._read_word(data, 10) / 131.0 - self.gyro_bias[1]
        gz = self._read_word(data, 12) / 131.0 - self.gyro_bias[2]

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # 지자기 없음 -> orientation 미사용 명시 (robot_localization 규약)
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gx * DEG2RAD
        msg.angular_velocity.y = gy * DEG2RAD
        msg.angular_velocity.z = gz * DEG2RAD
        msg.angular_velocity_covariance = [
            0.005, 0.0,   0.0,
            0.0,   0.005, 0.0,
            0.0,   0.0,   0.005,
        ]

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04,
        ]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()