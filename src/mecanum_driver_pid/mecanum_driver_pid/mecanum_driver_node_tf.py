#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import smbus2
import struct
import math
import time

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0.0] * 4
    q[0] = sr * cp * cy - cr * sp * sy 
    q[1] = cr * sp * cy + sr * cp * sy 
    q[2] = cr * cp * sy - sr * sp * cy 
    q[3] = cr * cp * cy + sr * sp * sy 
    return q

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, actual, dt, dynamic_max_out):
        error = target - actual
        self.integral += error * dt
        self.integral = max(min(self.integral, dynamic_max_out), -dynamic_max_out)
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, dynamic_max_out), -dynamic_max_out)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class MecanumDriverNode(Node):
    def __init__(self):
        super().__init__('mecanum_driver_node')

        # --- 1. 물리적 제원 및 파라미터 ---
        self.declare_parameter('wheel_radius', 0.04)         
        self.declare_parameter('wheelbase', 0.245)           
        self.declare_parameter('track_width', 0.2276)        
        self.declare_parameter('pulse_per_rev', 1320.0)      
        self.declare_parameter('timeout_sec', 0.5)           
        
        # 💡 모션별 가변 출력 파라미터 (대각선 추가)
        self.declare_parameter('max_pwm_fwd', 35)      # 직진/후진
        self.declare_parameter('max_pwm_strafe', 60)   # 좌우 평행이동
        self.declare_parameter('max_pwm_diag', 45)     # 💡 대각선 기동 (신규 추가)
        self.declare_parameter('max_pwm_turn', 65)     # 제자리 돌기
        
        self.declare_parameter('pid_kp', 10.0)               
        self.declare_parameter('pid_ki', 0.5)                
        self.declare_parameter('pid_kd', 0.1)                

        self.R = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('wheelbase').value / 2.0
        self.Ly = self.get_parameter('track_width').value / 2.0
        self.K = self.Lx + self.Ly
        self.PPR = self.get_parameter('pulse_per_rev').value

        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        
        self.pids = [PIDController(kp, ki, kd) for _ in range(4)]

        # --- 2. I2C 하드웨어 설정 ---
        self.I2C_BUS = 1
        self.MOTOR_ADDR = 0x34
        self.MOTOR_TYPE_ADDR = 0x14
        self.MOTOR_FIXED_PWM_ADDR = 0x1F
        self.MOTOR_ENCODER_TOTAL_ADDR = 0x3C
        
        try:
            self.bus = smbus2.SMBus(self.I2C_BUS)
            self.bus.write_byte_data(self.MOTOR_ADDR, self.MOTOR_TYPE_ADDR, 0)
            self.get_logger().info("I2C 하드웨어 초기화 완료 (PWM 모드)")
        except Exception as e:
            self.get_logger().error(f"I2C 초기화 실패: {e}")
            raise e

        # --- 3. ROS 2 통신 설정 ---
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.last_cmd_time = self.get_clock().now()
        
        self.prev_encoders = [0, 0, 0, 0] 
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("메카넘 모션 인지형 드라이버 시작! (타임스템프 고정형)")

    def cmd_vel_callback(self, msg: Twist):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y   
        self.target_wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        # 💡 [핵심 수정] 타임스탬프를 루프 최상단에서 한 번만 받아 고정합니다.
        current_time = self.get_clock().now()
        timestamp = current_time.to_msg() 

        # --- 1. 엔코더 읽기 ---
        try:
            enc_data = self.bus.read_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_ENCODER_TOTAL_ADDR, 16)
            current_encoders = list(struct.unpack('iiii', bytes(enc_data)))
            
            # 💡 [안정성 수정] 엔코더 읽기에 성공했을 때만 이동 거리 연산을 수행합니다.
            delta_hw = [current_encoders[i] - self.prev_encoders[i] for i in range(4)]
            self.prev_encoders = current_encoders

            # M2(RL)와 M3(FR) 물리 극성 보정 (B-A-A-B 장착 기준)
            delta_fl = delta_hw[0]       
            delta_rl = -delta_hw[1]      
            delta_fr = -delta_hw[2]      
            delta_rr = delta_hw[3]       

            dist_fl = (delta_fl / self.PPR) * (2.0 * math.pi * self.R)
            dist_rl = (delta_rl / self.PPR) * (2.0 * math.pi * self.R)
            dist_fr = (delta_fr / self.PPR) * (2.0 * math.pi * self.R)
            dist_rr = (delta_rr / self.PPR) * (2.0 * math.pi * self.R)
            
            dx_local = (dist_fl + dist_fr + dist_rl + dist_rr) / 4.0
            dy_local = (dist_fl - dist_fr - dist_rl + dist_rr) / 4.0
            dtheta   = (-dist_fl + dist_fr - dist_rl + dist_rr) / (4.0 * self.K)

            self.odom_x += dx_local * math.cos(self.odom_theta) - dy_local * math.sin(self.odom_theta)
            self.odom_y += dx_local * math.sin(self.odom_theta) + dy_local * math.cos(self.odom_theta)
            self.odom_theta += dtheta
            
            # SLAM에 쏠 속도 (Velocity) 기록
            vel_x = dx_local / self.timer_period
            vel_y = dy_local / self.timer_period
            vel_z = dtheta / self.timer_period
            
            # PID 실제 속도 (Actual Velocity)
            w_actual_fl = dist_fl / self.timer_period
            w_actual_rl = dist_rl / self.timer_period
            w_actual_fr = dist_fr / self.timer_period
            w_actual_rr = dist_rr / self.timer_period
            w_actuals = [w_actual_fl, w_actual_rl, w_actual_fr, w_actual_rr]
            
        except Exception as e:
            # I2C 읽기 에러 발생 시: 이동 연산은 건너뛰지만, TF는 끊기지 않게 기존 값을 유지합니다.
            self.get_logger().warn(f"I2C Read 에러: {e}")
            vel_x, vel_y, vel_z = 0.0, 0.0, 0.0
            w_actuals = [0.0, 0.0, 0.0, 0.0]

        # --- 2. TF & Odometry 발행 (에러가 나도 항상 실행됨) ---
        q = quaternion_from_euler(0, 0, self.odom_theta)

        t = TransformStamped()
        t.header.stamp = timestamp # 고정된 타임스탬프
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = timestamp # 고정된 타임스탬프
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # SLAM 알고리즘이 휠 오도메트리를 신뢰할 수 있도록 공분산 추가
        odom.pose.covariance[0] = 0.05  
        odom.pose.covariance[7] = 0.05  
        odom.pose.covariance[35] = 0.1  

        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        odom.twist.twist.angular.z = vel_z
        self.pub_odom.publish(odom)

        # --- 3. 모터 제어 (Write) ---
        timeout = self.get_parameter('timeout_sec').value
        if (current_time - self.last_cmd_time).nanoseconds > timeout * 1e9:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self.target_vx, self.target_vy, self.target_wz

        # 대각선(Diagonal) 판별
        if abs(wz) > 0.01:
            current_max_pwm = self.get_parameter('max_pwm_turn').value
        elif abs(vx) > 0.01 and abs(vy) > 0.01:
            current_max_pwm = self.get_parameter('max_pwm_diag').value
        elif abs(vy) > 0.01:
            current_max_pwm = self.get_parameter('max_pwm_strafe').value
        else:
            current_max_pwm = self.get_parameter('max_pwm_fwd').value

        w_target_fl = (vx + vy - self.K * wz) / self.R
        w_target_rl = (vx - vy - self.K * wz) / self.R
        w_target_fr = (vx - vy + self.K * wz) / self.R
        w_target_rr = (vx + vy + self.K * wz) / self.R
        w_targets = [w_target_fl, w_target_rl, w_target_fr, w_target_rr]

        pwm_logical = [0, 0, 0, 0]
        
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            for pid in self.pids:
                pid.reset()
        else:
            for i in range(4):
                pid_out = self.pids[i].compute(w_targets[i], w_actuals[i], self.timer_period, current_max_pwm)
                pwm_logical[i] = int(max(min(pid_out, current_max_pwm), -current_max_pwm))

        pwm_payload = [0, 0, 0, 0]
        pwm_payload[0] = pwm_logical[0]      
        pwm_payload[1] = -pwm_logical[1]     
        pwm_payload[2] = -pwm_logical[2]     
        pwm_payload[3] = pwm_logical[3]      

        i2c_data = [val & 0xFF for val in pwm_payload]

        try:
            self.bus.write_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_FIXED_PWM_ADDR, i2c_data)
        except Exception as e:
            self.get_logger().warn(f"I2C Write 에러: {e}")

    def stop_motors(self):
        try:
            self.bus.write_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_FIXED_PWM_ADDR, [0, 0, 0, 0])
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()