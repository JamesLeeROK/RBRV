#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smbus2 import SMBus


print("hello")
class MotorTeleopSubscriber(Node):
    def __init__(self):
        super().__init__('motor_teleop_subscriber')
        
        # I2C 설정 상수
        self.I2C_BUS = 1
        self.MOTOR_ADDR = 0x34
        self.MOTOR_TYPE_ADDR = 0x14
        self.MOTOR_FIXED_PWM_ADDR = 0x1F
        self.MOTOR_TYPE_WITHOUT_ENCODER = 0
        
        # 제어 파라미터 (안전을 위해 최대 PWM 60 제한)
        self.MAX_PWM = 60  
        
        try:
            self.bus = SMBus(self.I2C_BUS)
            self.bus.write_byte_data(self.MOTOR_ADDR, self.MOTOR_TYPE_ADDR, self.MOTOR_TYPE_WITHOUT_ENCODER)
            self.get_logger().info("I2C Motor Driver Initialized in PWM Mode.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C Bus: {e}")
            raise e

        # 1. 토픽 이름 수정 (/turtle1/cmd_vel)
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel', 
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Subscribed to '/turtle1/cmd_vel' topic. Ready for teleop.")

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z
        
        # 2. 스케일 팩터 조정 (입력값 2.0이 들어올 때 적절한 PWM이 되도록 조정)
        # 예: 2.0 * 25.0 = 50 PWM
        scale_factor = 25.0 
        
        left_pwm = int(left_speed * scale_factor)
        right_pwm = int(right_speed * scale_factor)
        
        left_pwm = max(min(left_pwm, self.MAX_PWM), -self.MAX_PWM)
        right_pwm = max(min(right_pwm, self.MAX_PWM), -self.MAX_PWM)
        
        pwm_payload = [left_pwm, left_pwm, right_pwm, right_pwm]
        
        i2c_data = [val & 0xFF for val in pwm_payload]
        
        try:
            self.bus.write_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_FIXED_PWM_ADDR, i2c_data)
            # 3. 로그 레벨을 info로 변경하여 터미널에 항상 표출
            self.get_logger().info(f"Received Twist(x:{linear_x:.1f}, z:{angular_z:.1f}) -> Sent PWM: {pwm_payload}")
        except Exception as e:
            self.get_logger().warn(f"I2C Write Failed: {e}")

    def stop_motors(self):
        try:
            self.bus.write_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_FIXED_PWM_ADDR, [0, 0, 0, 0])
            self.get_logger().info("All motors stopped safely.")
        except Exception as e:
            self.get_logger().error(f"Failed to stop motors: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorTeleopSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutting down...")
    finally:
        if node:
            node.stop_motors()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
