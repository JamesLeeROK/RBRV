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
    """오일러 각도를 쿼터니언으로 변환하는 헬퍼 함수"""
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
    """각 바퀴의 속도를 독립적으로 제어하기 위한 PID 제어기 클래스"""
    def __init__(self, kp, ki, kd, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, actual, dt):
        error = target - actual
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_out), -self.max_out)
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, self.max_out), -self.max_out)

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
        self.declare_parameter('max_pwm', 30)                
        self.declare_parameter('timeout_sec', 0.5)           
        
        self.declare_parameter('pid_kp', 10.0)               
        self.declare_parameter('pid_ki', 0.5)                
        self.declare_parameter('pid_kd', 0.1)                

        self.R = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('wheelbase').value / 2.0
        self.Ly = self.get_parameter('track_width').value / 2.0
        self.K = self.Lx + self.Ly
        self.PPR = self.get_parameter('pulse_per_rev').value
        self.MAX_PWM = self.get_parameter('max_pwm').value

        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        
        self.pids = [PIDController(kp, ki, kd, self.MAX_PWM) for _ in range(4)]

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
        self.get_logger().info("메카넘 최종 로직 시작! (M2, M3 극성 보정 & B-A-A-B 적용)")

    def cmd_vel_callback(self, msg: Twist):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y   
        self.target_wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        current_time = self.get_clock().now()

        # ==========================================
        # STEP 1: I2C 엔코더 읽기 및 하드웨어 극성 보정
        # ==========================================
        try:
            enc_data = self.bus.read_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_ENCODER_TOTAL_ADDR, 16)
            current_encoders = list(struct.unpack('iiii', bytes(enc_data)))
        except Exception as e:
            return

        time.sleep(0.002)

        # 하드웨어 배선: M1=FL, M2=RL, M3=FR, M4=RR
        delta_hw = [current_encoders[i] - self.prev_encoders[i] for i in range(4)]
        self.prev_encoders = current_encoders

        # 💡 [핵심 보정 1] 진단 테스트 결과 반영
        # M2(RL)와 M3(FR)는 양수 전압에 뒤로 돌았으므로, 엔코더 증가 방향을 음수로 반전시켜 정방향으로 맞춤
        delta_fl = delta_hw[0]       # M1 (F)
        delta_rl = -delta_hw[1]      # M2 (B) -> 극성 보정
        delta_fr = -delta_hw[2]      # M3 (B) -> 극성 보정
        delta_rr = delta_hw[3]       # M4 (F)

        # ==========================================
        # STEP 2: 순운동학(FK) 및 오도메트리 계산 (B-A-A-B 패턴)
        # ==========================================
        dist_fl = (delta_fl / self.PPR) * (2.0 * math.pi * self.R)
        dist_rl = (delta_rl / self.PPR) * (2.0 * math.pi * self.R)
        dist_fr = (delta_fr / self.PPR) * (2.0 * math.pi * self.R)
        dist_rr = (delta_rr / self.PPR) * (2.0 * math.pi * self.R)
        
        # B-A-A-B 패턴 오도메트리 공식 (Y축 이동의 합산 부호가 X패턴과 반대)
        dx_local = (dist_fl + dist_fr + dist_rl + dist_rr) / 4.0
        dy_local = (dist_fl - dist_fr - dist_rl + dist_rr) / 4.0
        dtheta   = (-dist_fl + dist_fr - dist_rl + dist_rr) / (4.0 * self.K)

        self.odom_x += dx_local * math.cos(self.odom_theta) - dy_local * math.sin(self.odom_theta)
        self.odom_y += dx_local * math.sin(self.odom_theta) + dy_local * math.cos(self.odom_theta)
        self.odom_theta += dtheta

        # ==========================================
        # STEP 3: ROS 2 Odom & TF 퍼블리시
        # ==========================================
        q = quaternion_from_euler(0, 0, self.odom_theta)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
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
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = dx_local / self.timer_period
        odom.twist.twist.linear.y = dy_local / self.timer_period
        odom.twist.twist.angular.z = dtheta / self.timer_period
        self.pub_odom.publish(odom)

        # ==========================================
        # STEP 4: 역운동학(IK) 및 PID 제어 (B-A-A-B 패턴)
        # ==========================================
        timeout = self.get_parameter('timeout_sec').value
        if (current_time - self.last_cmd_time).nanoseconds > timeout * 1e9:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self.target_vx, self.target_vy, self.target_wz

        # B-A-A-B 패턴 역운동학 (Y축 vy 부호 유의)
        w_target_fl = (vx + vy - self.K * wz) / self.R
        w_target_rl = (vx - vy - self.K * wz) / self.R
        w_target_fr = (vx - vy + self.K * wz) / self.R
        w_target_rr = (vx + vy + self.K * wz) / self.R
        
        w_targets = [w_target_fl, w_target_rl, w_target_fr, w_target_rr]

        # 실제 각속도 환산 (극성이 올바르게 맞춰진 dist 활용)
        w_actual_fl = dist_fl / self.timer_period
        w_actual_rl = dist_rl / self.timer_period
        w_actual_fr = dist_fr / self.timer_period
        w_actual_rr = dist_rr / self.timer_period
        w_actuals = [w_actual_fl, w_actual_rl, w_actual_fr, w_actual_rr]

        pwm_logical = [0, 0, 0, 0]
        
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            for pid in self.pids:
                pid.reset()
        else:
            for i in range(4):
                pid_out = self.pids[i].compute(w_targets[i], w_actuals[i], self.timer_period)
                pwm_logical[i] = int(max(min(pid_out, self.MAX_PWM), -self.MAX_PWM))

        # ==========================================
        # STEP 5: 최종 하드웨어 출력 (M2, M3 역전 보정) 및 쓰기
        # ==========================================
        # 💡 [핵심 보정 2] 진단 테스트 결과 반영
        # M2(RL)와 M3(FR)는 물리적으로 뒤집혀 있으므로 PWM 출력 시에도 음수를 곱해줍니다.
        pwm_payload = [0, 0, 0, 0]
        pwm_payload[0] = pwm_logical[0]      # M1 (FL)
        pwm_payload[1] = -pwm_logical[1]     # M2 (RL) -> 역전 보정
        pwm_payload[2] = -pwm_logical[2]     # M3 (FR) -> 역전 보정
        pwm_payload[3] = pwm_logical[3]      # M4 (RR)

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
    def __init__(self, kp, ki, kd, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, actual, dt):
        error = target - actual
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_out), -self.max_out)
        
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, self.max_out), -self.max_out)

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
        self.declare_parameter('max_pwm', 30)                
        self.declare_parameter('timeout_sec', 0.5)           
        
        self.declare_parameter('pid_kp', 10.0)               
        self.declare_parameter('pid_ki', 0.5)                
        self.declare_parameter('pid_kd', 0.1)                

        self.R = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('wheelbase').value / 2.0
        self.Ly = self.get_parameter('track_width').value / 2.0
        self.K = self.Lx + self.Ly
        self.PPR = self.get_parameter('pulse_per_rev').value
        self.MAX_PWM = self.get_parameter('max_pwm').value

        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        
        self.pids = [PIDController(kp, ki, kd, self.MAX_PWM) for _ in range(4)]

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
        self.get_logger().info("메카넘 최종 로직 시작! (M2, M3 극성 보정 & B-A-A-B 적용)")

    def cmd_vel_callback(self, msg: Twist):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y   
        self.target_wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        current_time = self.get_clock().now()

        # ==========================================
        # STEP 1: I2C 엔코더 읽기 및 하드웨어 극성 보정
        # ==========================================
        try:
            enc_data = self.bus.read_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_ENCODER_TOTAL_ADDR, 16)
            current_encoders = list(struct.unpack('iiii', bytes(enc_data)))
        except Exception as e:
            return

        time.sleep(0.002)

        # 하드웨어 배선: M1=FL, M2=RL, M3=FR, M4=RR
        delta_hw = [current_encoders[i] - self.prev_encoders[i] for i in range(4)]
        self.prev_encoders = current_encoders

        # 💡 [핵심 보정] 진단 테스트 결과(FL:F, RL:B, FR:B, RR:F) 반영
        # M2(RL)와 M3(FR)는 양수 전압에 뒤로 돌았으므로, 엔코더 증가 방향을 음수로 반전시켜 정방향으로 맞춤
        delta_fl = delta_hw[0]       # M1 (F)
        delta_rl = -delta_hw[1]      # M2 (B) -> 극성 보정
        delta_fr = -delta_hw[2]      # M3 (B) -> 극성 보정
        delta_rr = delta_hw[3]       # M4 (F)

        # ==========================================
        # STEP 2: 순운동학(FK) 및 오도메트리 계산 (B-A-A-B 패턴)
        # ==========================================
        dist_fl = (delta_fl / self.PPR) * (2.0 * math.pi * self.R)
        dist_rl = (delta_rl / self.PPR) * (2.0 * math.pi * self.R)
        dist_fr = (delta_fr / self.PPR) * (2.0 * math.pi * self.R)
        dist_rr = (delta_rr / self.PPR) * (2.0 * math.pi * self.R)
        
        # B-A-A-B 패턴 오도메트리 공식 (Y축 이동의 합산 부호가 X패턴과 반대)
        dx_local = (dist_fl + dist_fr + dist_rl + dist_rr) / 4.0
        dy_local = (dist_fl - dist_fr - dist_rl + dist_rr) / 4.0
        dtheta   = (-dist_fl + dist_fr - dist_rl + dist_rr) / (4.0 * self.K)

        self.odom_x += dx_local * math.cos(self.odom_theta) - dy_local * math.sin(self.odom_theta)
        self.odom_y += dx_local * math.sin(self.odom_theta) + dy_local * math.cos(self.odom_theta)
        self.odom_theta += dtheta

        # ==========================================
        # STEP 3: ROS 2 Odom & TF 퍼블리시
        # ==========================================
        q = quaternion_from_euler(0, 0, self.odom_theta)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
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
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = dx_local / self.timer_period
        odom.twist.twist.linear.y = dy_local / self.timer_period
        odom.twist.twist.angular.z = dtheta / self.timer_period
        self.pub_odom.publish(odom)

        # ==========================================
        # STEP 4: 역운동학(IK) 및 PID 제어 (B-A-A-B 패턴)
        # ==========================================
        timeout = self.get_parameter('timeout_sec').value
        if (current_time - self.last_cmd_time).nanoseconds > timeout * 1e9:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self.target_vx, self.target_vy, self.target_wz

        # B-A-A-B 패턴 역운동학 (Y축 vy 부호 유의)
        w_target_fl = (vx + vy - self.K * wz) / self.R
        w_target_rl = (vx - vy - self.K * wz) / self.R
        w_target_fr = (vx - vy + self.K * wz) / self.R
        w_target_rr = (vx + vy + self.K * wz) / self.R
        
        w_targets = [w_target_fl, w_target_rl, w_target_fr, w_target_rr]

        # 실제 각속도 환산 (극성이 올바르게 맞춰진 dist 활용)
        w_actual_fl = dist_fl / self.timer_period
        w_actual_rl = dist_rl / self.timer_period
        w_actual_fr = dist_fr / self.timer_period
        w_actual_rr = dist_rr / self.timer_period
        w_actuals = [w_actual_fl, w_actual_rl, w_actual_fr, w_actual_rr]

        pwm_logical = [0, 0, 0, 0]
        
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            for pid in self.pids:
                pid.reset()
        else:
            for i in range(4):
                pid_out = self.pids[i].compute(w_targets[i], w_actuals[i], self.timer_period)
                pwm_logical[i] = int(max(min(pid_out, self.MAX_PWM), -self.MAX_PWM))

        # ==========================================
        # STEP 5: 최종 하드웨어 출력 (M2, M3 역전 보정) 및 쓰기
        # ==========================================
        # 💡 [핵심 보정] 진단 테스트 결과 반영: M2(RL)와 M3(FR)는 물리적으로 뒤집혀 있으므로 PWM 출력 시에도 음수를 곱해줍니다.
        pwm_payload = [0, 0, 0, 0]
        pwm_payload[0] = pwm_logical[0]      # M1 (FL)
        pwm_payload[1] = -pwm_logical[1]     # M2 (RL) -> 역전 보정
        pwm_payload[2] = -pwm_logical[2]     # M3 (FR) -> 역전 보정
        pwm_payload[3] = pwm_logical[3]      # M4 (RR)

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
if __name__ == '__main__':
    main()