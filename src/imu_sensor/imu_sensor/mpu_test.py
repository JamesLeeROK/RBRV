#!/usr/bin/env python3
import smbus2
import time

I2C_BUS = 1
ADDRESS = 0x68

try:
    bus = smbus2.SMBus(I2C_BUS)
    print("I2C 버스 연결 성공...")
    
    # 1. 센서 신분증 검사 (WHO_AM_I 레지스터)
    # MPU6050은 0x75 레지스터를 읽으면 무조건 0x68(104)을 반환해야 합니다.
    who_am_i = bus.read_byte_data(ADDRESS, 0x75)
    print(f"[검문 결과] WHO_AM_I 레지스터 값: {hex(who_am_i)} (정상값: 0x68)")
    
    if who_am_i != 0x68:
        print("🚨 경고: 이 칩은 MPU6050이 아니거나, 완전히 고장났습니다!")
    else:
        print("✅ 정상적인 MPU6050 칩이 확인되었습니다.")

    # 2. 센서 깨우기 및 안정화 대기
    bus.write_byte_data(ADDRESS, 0x6B, 0)
    time.sleep(0.1) # 깨어날 시간 주기 (매우 중요)

    # 3. 가속도 X축 데이터 (High, Low 바이트) 강제 읽기
    high = bus.read_byte_data(ADDRESS, 0x3B)
    low = bus.read_byte_data(ADDRESS, 0x3C)
    
    print(f"가속도 X축 Raw 데이터 - High: {high}, Low: {low}")
    if high == 0 and low == 0:
        print("❌ 센서가 깨어났지만 데이터를 측정하지 못하고 있습니다 (MEMS 고장 의심).")
    else:
        print("🎉 센서 데이터가 정상적으로 올라오고 있습니다!")

except Exception as e:
    print(f"오류 발생: {e}")
