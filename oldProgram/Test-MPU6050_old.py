#!/usr/bin/env python3
import smbus
import time
from mpu6050 import mpu6050
import math
 
# MPU6050の初期化
mpu = mpu6050(0x68)
DEV_ADDR = 0x68

# オフセットの設定（適宜調整してください）
GYRO_X_OFFSET = -1.8396946564885497
GYRO_Y_OFFSET = -0.6946564885496184
GYRO_Z_OFFSET = 1.2595419847328244
ACCEL_X_OFFSET = 0.9935936889648437
ACCEL_Y_OFFSET = 0.316034619140625
ACCEL_Z_OFFSET = 16.462530615234375
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
GYRO_XOUT = 0x43

PWR_MGMT_1 = 0x6b

bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low = bus.read_byte_data(DEV_ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_sensor(adr):
    val = read_word(adr)
    if (val >= 0x8000):  return -((65535 - val) + 1)
    else:  return val

def get_gyro_data():
    # ジャイロスコープのデータ取得
    gyro_data = mpu.get_gyro_data()
    gyrox_now = gyro_data['x'] + GYRO_X_OFFSET
    gyroy_now = gyro_data['y'] + GYRO_Y_OFFSET
    gyroz_now = gyro_data['z'] + GYRO_Z_OFFSET
 
    return gyrox_now, gyroy_now, gyroz_now
 
def get_accel_data():
    accel_x = read_word_sensor(ACCEL_XOUT)/ 16384.0 * 9.80665
    accel_y = read_word_sensor(ACCEL_YOUT)/ 16384.0 * 9.80665
    accel_z = read_word_sensor(ACCEL_ZOUT)/ 16384.0 * 9.80665
    return accel_x, accel_y, accel_z
 
def calculate_angles(accel_x, accel_y, accel_z):
    # 加速度データからピッチとロールを計算
    pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    roll = math.atan2(-accel_x, accel_z) * 180 / math.pi
    return pitch, roll
 
def main():
    while True:
        # ジャイロと加速度のデータ取得
        gyrox, gyroy, gyroz = get_gyro_data()
        accel_x, accel_y, accel_z = get_accel_data()
        # 角度計算
        pitch, roll = calculate_angles(accel_x, accel_y, accel_z)
 
        # データ表示
        print(f"Gyro X: {gyrox:.2f}, Gyro Y: {gyroy:.2f}, Gyro Z: {gyroz:.2f}")
        print(f"Accel X: {accel_x:.2f}, Accel Y: {accel_y:.2f}, Accel Z: {accel_z:.2f}")
        print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}")
        print("")
 
        # データ取得の間隔を調整
        time.sleep(0.1)
 
if __name__ == "__main__":
    main()
