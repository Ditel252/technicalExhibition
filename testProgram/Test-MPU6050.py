import smbus
import math
import time
 
# MPU6050のI2Cアドレス
MPU6050_ADDR = 0x68
 
# レジスタ
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
 
# I2Cバスに接続
bus = smbus.SMBus(1)
 
# MPU6050を初期化
def mpu_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
 
# 16ビットのデータを取得
def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr+1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value
 
# キャリブレーション関数
def calibrate_mpu(samples=1000):
    gyro_offsets = {'x': 0, 'y': 0, 'z': 0}
    accel_offsets = {'x': 0, 'y': 0, 'z': 0}
 
    print("キャリブレーション中...")
 
    for _ in range(samples):
        # ジャイロデータの平均を計算
        gyro_offsets['x'] += read_raw_data(GYRO_XOUT_H)
        gyro_offsets['y'] += read_raw_data(GYRO_XOUT_H + 2)
        gyro_offsets['z'] += read_raw_data(GYRO_XOUT_H + 4)
 
        # 加速度データの平均を計算
        accel_offsets['x'] += read_raw_data(ACCEL_XOUT_H)
        accel_offsets['y'] += read_raw_data(ACCEL_XOUT_H + 2)
        accel_offsets['z'] += read_raw_data(ACCEL_XOUT_H + 4)
 
        time.sleep(0.001)  # サンプリングの間隔を調整
 
    # 平均を取ってオフセット値を算出
    gyro_offsets['x'] /= samples
    gyro_offsets['y'] /= samples
    gyro_offsets['z'] /= samples
 
    accel_offsets['x'] /= samples
    accel_offsets['y'] /= samples
    accel_offsets['z'] /= samples
 
    # Z軸は地球重力があるのでその分を補正 (1g = 16384 LSB)
    accel_offsets['z'] -= 16384
 
    print("キャリブレーション完了")
 
    return gyro_offsets, accel_offsets
 
# カルマンフィルタクラス
class KalmanFilter:
    def __init__(self, Q_angle, Q_bias, R_measure):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure
 
        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
 
        self.P = [[0, 0], [0, 0]]
 
    def get_angle(self, new_angle, new_rate, dt):
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate
 
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt
 
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]
 
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y
 
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
 
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp
 
        return self.angle
 
# 姿勢角度を計算
def calculate_angles(kalman_pitch, kalman_roll, prev_yaw_rate, dt, gyro_offsets, accel_offsets):
    acc_x = read_raw_data(ACCEL_XOUT_H) - accel_offsets['x']
    acc_y = read_raw_data(ACCEL_XOUT_H + 2) - accel_offsets['y']
    acc_z = read_raw_data(ACCEL_XOUT_H + 4) - accel_offsets['z']
 
    accel_pitch = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)) * 180 / math.pi
    accel_roll = math.atan2(-acc_x, acc_z) * 180 / math.pi
 
    gyro_x = read_raw_data(GYRO_XOUT_H) - gyro_offsets['x']
    gyro_y = read_raw_data(GYRO_XOUT_H + 2) - gyro_offsets['y']
    gyro_z = read_raw_data(GYRO_XOUT_H + 4) - gyro_offsets['z']
 
    pitch_rate = gyro_x / 131.0
    roll_rate = gyro_y / 131.0
    yaw_rate = gyro_z / 131.0
 
    pitch = kalman_pitch.get_angle(accel_pitch, pitch_rate, dt)
    roll = kalman_roll.get_angle(accel_roll, roll_rate, dt)
 
    yaw = yaw_rate + prev_yaw_rate * dt
 
    return pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate
 
# 初期化
mpu_init()
 
# センサーのキャリブレーション
gyro_offsets, accel_offsets = calibrate_mpu()
 
# カルマンフィルタの初期化
kalman_pitch = KalmanFilter(0.001, 0.003, 0.03)
kalman_roll = KalmanFilter(0.001, 0.003, 0.03)
 
prev_time = time.time()
pitch_total = 0
roll_total = 0
yaw_total = 0
prev_pitch_rate = 0
prev_roll_rate = 0
prev_yaw_rate = 0
 
try:
    while True:
        current_time = time.time()
        dt = current_time - prev_time
 
        pitch, roll, yaw_increment, pitch_rate, roll_rate, yaw_rate = calculate_angles(
            kalman_pitch, kalman_roll, prev_yaw_rate, dt, gyro_offsets, accel_offsets
        )
 
        yaw_total += yaw_increment
 
        print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw_total:.2f}")
 
        prev_pitch_rate = pitch_rate
        prev_roll_rate = roll_rate
        prev_yaw_rate = yaw_rate
        prev_time = current_time
 
        time.sleep(0.01)
except KeyboardInterrupt:
    print("プログラム終了")
