#!/usr/bin/env python3
import smbus2 as smbus
import time

# MPU-6050 레지스터 주소
MPU_ADDR = 0x68  # AD0가 GND면 0x68, VCC면 0x69
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus.SMBus(1)  # RPi는 보통 버스 1

def mpu_init():
    # Sleep 모드 해제 (기본값은 sleep=1이라 깨워줘야 함)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
    time.sleep(0.1)

def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def read_all():
    accel_x = read_word(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word(ACCEL_XOUT_H + 2) / 16384.0
    accel_z = read_word(ACCEL_XOUT_H + 4) / 16384.0

    gyro_x = read_word(GYRO_XOUT_H) / 131.0
    gyro_y = read_word(GYRO_XOUT_H + 2) / 131.0
    gyro_z = read_word(GYRO_XOUT_H + 4) / 131.0

    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

if __name__ == "__main__":
    try:
        mpu_init()
        print("MPU-6050 초기화 완료. Ctrl+C로 종료.")
        while True:
            ax, ay, az, gx, gy, gz = read_all()
            print(f"Accel[g] X:{ax:6.2f} Y:{ay:6.2f} Z:{az:6.2f} | "
                  f"Gyro[deg/s] X:{gx:6.2f} Y:{gy:6.2f} Z:{gz:6.2f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n종료")
    except OSError as e:
        print(f"통신 오류: {e}")
        print("배선(VCC/GND/SDA/SCL) 및 i2cdetect 결과를 다시 확인하세요.")