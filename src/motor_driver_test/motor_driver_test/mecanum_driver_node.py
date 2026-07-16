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
    q[0] = sr * cp * cy - cr * sp * sy # x
    q[1] = cr * sp * cy + sr * cp * sy # y
    q[2] = cr * cp * sy - sr * sp * cy # z
    q[3] = cr * cp * cy + sr * sp * sy # w
    return q

class MecanumDriverNode(Node):
    def __init__(self):
        super().__init__('mecanum_driver_node')

        # --- 1. 물리적 제원 (파라미터화) ---
        self.declare_parameter('wheel_radius', 0.04)         # 80mm / 2
        self.declare_parameter('wheelbase', 0.245)           # 앞/뒷바퀴 축간 거리
        self.declare_parameter('track_width', 0.2276)        # 전폭(0.265) - 휠두께(0.0374)
        self.declare_parameter('pulse_per_rev', 1320.0)      # 1:30 모터 한 바퀴 펄스 수
        self.declare_parameter('max_pwm', 70)                # 안전 최대 PWM (0~100)
        self.declare_parameter('timeout_sec', 0.5)           # 통신 끊김 시 정지 타임아웃
        
        self.R = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('wheelbase').value / 2.0
        self.Ly = self.get_parameter('track_width').value / 2.0
        self.K = self.Lx + self.Ly
        self.PPR = self.get_parameter('pulse_per_rev').value
        self.MAX_PWM = self.get_parameter('max_pwm').value

        # --- 2. I2C 하드웨어 설정 ---
        self.I2C_BUS = 1
        self.MOTOR_ADDR = 0x34
        self.MOTOR_TYPE_ADDR = 0x14
        self.MOTOR_FIXED_PWM_ADDR = 0x1F
        self.MOTOR_ENCODER_TOTAL_ADDR = 0x3C
        
        try:
            self.bus = smbus2.SMBus(self.I2C_BUS)
            # 엔코더 없는 PWM 모드로 초기화 (자체 PID 사용 안함)
            self.bus.write_byte_data(self.MOTOR_ADDR, self.MOTOR_TYPE_ADDR, 0)
            self.get_logger().info("I2C 하드웨어 초기화 완료 (PWM 모드)")
        except Exception as e:
            self.get_logger().error(f"I2C 초기화 실패: {e}")
            raise e

        # --- 3. ROS 2 통신 설정 ---
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 상태 변수
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.last_cmd_time = self.get_clock().now()
        
        self.prev_encoders = [0, 0, 0, 0] # FL, RL, FR, RR
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # --- 4. 메인 제어 루프 (20Hz = 50ms) ---
        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("메카넘 통합 드라이버 루프 시작! (20Hz)")

    def cmd_vel_callback(self, msg: Twist):
        """키보드/네비게이션으로부터 목표 속도 수신"""
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y   # 평행이동(Strafe)을 위한 Y축 속도
        self.target_wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        """1스레드 내에서 I2C 읽기 -> 연산 -> 쓰기를 순차적으로 진행 (충돌 방지)"""
        current_time = self.get_clock().now()

        # ==========================================
        # STEP 1: I2C 읽기 (엔코더 값 16바이트)
        # ==========================================
        try:
            enc_data = self.bus.read_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_ENCODER_TOTAL_ADDR, 16)
            # 4개의 32비트 정수로 언패킹 (M1: FL, M2: RL, M3: FR, M4: RR 구조로 가정)
            current_encoders = struct.unpack('iiii', bytes(enc_data))
        except Exception as e:
            self.get_logger().warn(f"I2C Read 에러 (패킷 스킵): {e}")
            return

        # I2C 안정화를 위한 미세 대기
        time.sleep(0.002)

        # ==========================================
        # STEP 2: 순운동학(FK) 및 오도메트리 계산
        # ==========================================
        delta_pulses = [current_encoders[i] - self.prev_encoders[i] for i in range(4)]
        self.prev_encoders = current_encoders

        # 각 바퀴별 이동 거리 (미터)
        dist = [(dp / self.PPR) * (2.0 * math.pi * self.R) for dp in delta_pulses]
        
        # 메카넘 순운동학 공식 (로봇 중심의 로컬 이동량 계산)
        # 가정: M1=앞좌(FL), M2=뒤좌(RL), M3=앞우(FR), M4=뒤우(RR)
        dx_local = (dist[0] + dist[1] + dist[2] + dist[3]) / 4.0
        dy_local = (-dist[0] + dist[1] + dist[2] - dist[3]) / 4.0
        dtheta   = (-dist[0] - dist[1] + dist[2] + dist[3]) / (4.0 * self.K)

        # 글로벌 좌표계 누적 (로봇의 현재 헤딩을 기준으로 회전 변환 적용)
        self.odom_x += dx_local * math.cos(self.odom_theta) - dy_local * math.sin(self.odom_theta)
        self.odom_y += dx_local * math.sin(self.odom_theta) + dy_local * math.cos(self.odom_theta)
        self.odom_theta += dtheta

        # ==========================================
        # STEP 3: ROS 2 Odom & TF 퍼블리시
        # ==========================================
        q = quaternion_from_euler(0, 0, self.odom_theta)

        # TF 변환 발행
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

        # Odometry 메시지 발행
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
        
        # 속도(Twist) 정보
        odom.twist.twist.linear.x = dx_local / self.timer_period
        odom.twist.twist.linear.y = dy_local / self.timer_period
        odom.twist.twist.angular.z = dtheta / self.timer_period
        self.pub_odom.publish(odom)

        # ==========================================
        # STEP 4: 안전장치(Watchdog) 및 역운동학(IK) 
        # ==========================================
        timeout = self.get_parameter('timeout_sec').value
        if (current_time - self.last_cmd_time).nanoseconds > timeout * 1e9:
            # 타임아웃 발생 시 모터 강제 정지
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self.target_vx, self.target_vy, self.target_wz

        # 메카넘 역운동학 공식 (각 바퀴의 목표 각속도 rad/s 계산)
        # FL(앞좌), RL(뒤좌), FR(앞우), RR(뒤우)
        w_fl = (vx - vy - self.K * wz) / self.R
        w_rl = (vx + vy - self.K * wz) / self.R
        w_fr = (vx + vy + self.K * wz) / self.R
        w_rr = (vx - vy + self.K * wz) / self.R

        # 각속도(rad/s)를 개루프 PWM 출력으로 맵핑 
        # *주의*: 현재는 엔코더 피드백 PID가 없으므로 임의의 스케일 팩터를 곱합니다.
        # 실제 주행을 보며 이 15.0이라는 값을 늘리거나 줄여야 합니다.
        PWM_SCALE = 15.0 
        
        pwm_fl = int(w_fl * PWM_SCALE)
        pwm_rl = int(w_rl * PWM_SCALE)
        pwm_fr = int(w_fr * PWM_SCALE)
        pwm_rr = int(w_rr * PWM_SCALE)

        # PWM 한계치(Saturation) 적용
        pwm_fl = max(min(pwm_fl, self.MAX_PWM), -self.MAX_PWM)
        pwm_rl = max(min(pwm_rl, self.MAX_PWM), -self.MAX_PWM)
        pwm_fr = max(min(pwm_fr, self.MAX_PWM), -self.MAX_PWM)
        pwm_rr = max(min(pwm_rr, self.MAX_PWM), -self.MAX_PWM)

        # ==========================================
        # STEP 5: I2C 쓰기
        # ==========================================
        # 음수를 8비트 부호없는 정수형(0~255) 비트로 변환 (smbus2 요구사항)
        pwm_payload = [pwm_fl, pwm_rl, pwm_fr, pwm_rr]
        i2c_data = [val & 0xFF for val in pwm_payload]

        try:
            self.bus.write_i2c_block_data(self.MOTOR_ADDR, self.MOTOR_FIXED_PWM_ADDR, i2c_data)
        except Exception as e:
            self.get_logger().warn(f"I2C Write 에러: {e}")

    def stop_motors(self):
        """종료 시 안전 정지"""
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