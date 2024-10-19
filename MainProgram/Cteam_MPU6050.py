import smbus
import math
import time

# MPU6050のI2Cアドレス
MPU6050_ADDR = 0x68

# レジスタ
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

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

class Cteamt_MPU6050:
    def init(self, _isDebugEnable:bool):
        self.i2cBus = smbus.SMBus(1)
        self.i2cBus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        self.isDebugEnable = _isDebugEnable
        
        self.kalmanPitch = KalmanFilter(0.001, 0.003, 0.03)
        self.kalmanRoll = KalmanFilter(0.001, 0.003, 0.03)
        self.kalmanYaw = KalmanFilter(0.001, 0.003, 0.03)
        
        self.gyroOffsets = {'x': 0, 'y': 0, 'z': 0}
        self.accelOffsets = {'x': 0, 'y': 0, 'z': 0}
        
        self.gyroOffsets = 0.0
        self.accelOffsets = 0.0
        
        self.acclX = 0.0
        self.acclY = 0.0
        self.acclZ = 0.0
        
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        
        self.pitchRate = 0.0
        self.rollRate = 0.0
        self.yawRate = 0.0
        
    def _read2ByteData(self, _addr):
        highByte = self.i2cBus.read_byte_data(MPU6050_ADDR, _addr)
        lowByte = self.i2cBus.read_byte_data(MPU6050_ADDR, _addr + 1)
        readData = ((highByte << 8) | lowByte)
        if readData > 0x8000: # > 32768
            readData = readData - 0x10000 # = readData - 65536
        return readData
        
    def calibrate(self, _numberOfSamples:int=1000):
        _gyroOffsets = {'x': 0, 'y': 0, 'z': 0}
        _accelOffsets = {'x': 0, 'y': 0, 'z': 0}
        
        if(self.isDebugEnable):
            print("キャリブレーション中...")

        for _ in range(_numberOfSamples):
            # ジャイロデータの平均を計算
            _gyroOffsets['x'] += self._read2ByteData(GYRO_XOUT_H)
            _gyroOffsets['y'] += self._read2ByteData(GYRO_XOUT_H + 2)
            _gyroOffsets['z'] += self._read2ByteData(GYRO_XOUT_H + 4)

            # 加速度データの平均を計算
            _accelOffsets['x'] += self._read2ByteData(ACCEL_XOUT_H)
            _accelOffsets['y'] += self._read2ByteData(ACCEL_XOUT_H + 2)
            _accelOffsets['z'] += self._read2ByteData(ACCEL_XOUT_H + 4)

            time.sleep(0.001)  # サンプリングの間隔を調整

        # 平均を取ってオフセット値を算出
        _gyroOffsets['x'] /= _numberOfSamples
        _gyroOffsets['y'] /= _numberOfSamples
        _gyroOffsets['z'] /= _numberOfSamples

        _accelOffsets['x'] /= _numberOfSamples
        _accelOffsets['y'] /= _numberOfSamples
        _accelOffsets['z'] /= _numberOfSamples

        # Z軸は地球重力があるのでその分を補正 (1g = 16384 LSB)
        _accelOffsets['z'] -= 16384
        
        if(self.isDebugEnable):
            print("キャリブレーション完了")
            
        self.gyroOffsets = _gyroOffsets
        self.accelOffsets = _accelOffsets

        return _gyroOffsets, _accelOffsets
    
    def getAngleAndGyro(self, _prevYawRate, _dt):
        _accX = self._read2ByteData(ACCEL_XOUT_H) - self.accelOffsets['x']
        _accY = self._read2ByteData(ACCEL_XOUT_H + 2) - self.accelOffsets['y']
        _accZ = self._read2ByteData(ACCEL_XOUT_H + 4) - self.accelOffsets['z']
    
        _accelPitch = math.atan2(_accY, math.sqrt(_accX**2 + _accZ**2)) * 180 / math.pi
        _accelRoll = math.atan2(-_accX, _accZ) * 180 / math.pi
    
        _gyroX = self._read2ByteData(GYRO_XOUT_H) - self.gyroOffsets['x']
        _gyroY = self._read2ByteData(GYRO_XOUT_H + 2) - self.gyroOffsets['y']
        _gyroZ = self._read2ByteData(GYRO_XOUT_H + 4) - self.gyroOffsets['z']
    
        _pitchRate = _gyroX / 131.0
        _rollRate = _gyroY / 131.0
        _yawRate = _gyroZ / 131.0
    
        _pitch = self.kalmanPitch.get_angle(_accelPitch, _pitchRate, _dt)
        _roll = self.kalmanRoll.get_angle(_accelRoll, _rollRate, _dt)
    
        _yaw = _yawRate + _prevYawRate * _dt
        
        self.acclX = _accX
        self.acclY = _accY
        self.acclZ = _accZ
        
        self.pitch = _pitch
        self.roll = _roll
        self.yaw = _yaw
        
        self.pitchRate = _pitchRate
        self.rollRate = _rollRate
        self.yawRate = _yawRate
    
        return _pitch, _roll, _yaw, _pitchRate, _rollRate, _yawRate


if __name__ == "__main__":
    # 初期化
    mpu = Cteamt_MPU6050()
    
    mpu.init()

    # センサーのキャリブレーション
    _gyroOffsets, _accelOffsets = mpu.calibrate()

    prev_time = time.time()
    yaw_total = 0
    prev_yaw_rate = 0

    try:
        while True:
            current_time = time.time()
            dt = current_time - prev_time

            mpu.getAngleAndGyro(prev_yaw_rate, dt)

            yaw_total += mpu.yaw

            # 各軸の角度、角速度、加速度を表示
            #print(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw_total:.2f}")
            print(f"Pitch Rate: {mpu.yawRate:.2f}, Roll Rate: {mpu.rollRate:.2f}, Yaw Rate: {mpu.yawRate:.2f}")
            print(f"Accel X: {mpu.acclX:.2f}, Accel Y: {mpu.acclY:.2f}, Accel Z: {mpu.acclZ:.2f}\n")
            
            prev_time = current_time

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("プログラム終了")
