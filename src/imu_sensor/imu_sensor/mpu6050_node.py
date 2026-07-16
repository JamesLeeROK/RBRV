#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import time

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        self.I2C_BUS = 1
        self.address = 0x68 # MPU6050 기본 주소 (AD0 핀에 따라 0x69일 수 있음)
        
        # MPU6050 레지스터 맵
        self.PWR_MGMT_1 = 0x6B
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        
        try:
            self.bus = smbus2.SMBus(self.I2C_BUS)
            # MPU6050 깨우기 (전원 관리 레지스터를 0으로 설정)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
            self.get_logger().info("MPU6050 I2C 연결 성공! (0x68)")
        except Exception as e:
            self.get_logger().error(f"MPU6050 초기화 실패: {e}")
            raise e
            
        # 💡 [핵심] MPU6050은 100Hz(0.01초) 이상으로 쏴줘야 EKF가 좋아합니다.
        self.timer = self.create_timer(0.01, self.publish_imu_data)
        
    def read_word_2c(self, reg):
        """16비트 2의 보수 데이터를 읽어오는 헬퍼 함수"""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def publish_imu_data(self):
        try:
            # 원시 데이터 읽기
            accel_x = self.read_word_2c(self.ACCEL_XOUT_H)
            accel_y = self.read_word_2c(self.ACCEL_XOUT_H + 2)
            accel_z = self.read_word_2c(self.ACCEL_XOUT_H + 4)
            
            gyro_x = self.read_word_2c(self.GYRO_XOUT_H)
            gyro_y = self.read_word_2c(self.GYRO_XOUT_H + 2)
            gyro_z = self.read_word_2c(self.GYRO_XOUT_H + 4)
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            # 💡 [스케일 변환] 
            # 가속도: +-2g 범위 설정 기준 -> 1g = 16384 LSB. 중력가속도(9.80665)를 곱해 m/s^2 로 변환
            msg.linear_acceleration.x = (accel_x / 16384.0) * 9.80665
            msg.linear_acceleration.y = (accel_y / 16384.0) * 9.80665
            msg.linear_acceleration.z = (accel_z / 16384.0) * 9.80665
            
            # 자이로: +-250 deg/s 범위 설정 기준 -> 1 deg/s = 131 LSB. 라디안(rad/s)으로 변환
            msg.angular_velocity.x = (gyro_x / 131.0) * (math.pi / 180.0)
            msg.angular_velocity.y = (gyro_y / 131.0) * (math.pi / 180.0)
            msg.angular_velocity.z = (gyro_z / 131.0) * (math.pi / 180.0)
            
            # MPU6050은 내부에 절대 방향(Orientation)을 계산하는 칩(DMP)이 있지만, 
            # Raw 데이터만으로는 알 수 없으므로 EKF에게 "난 방향은 몰라"라고 선언해야 함 (매우 중요)
            msg.orientation_covariance[0] = -1.0 
            
            self.pub_imu.publish(msg)
            
        except Exception as e:
            pass # 가끔 I2C 충돌 시 스킵

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()