import smbus
import math
import time

# MPU6050のI2Cアドレス
MPU6050_ADDR = 0x68

READ_CYCLE = 10 #[Hz]

# レジスタ
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

lastReadTime:float = 0.0
isProgramCycleOK:bool = False

class Cteamt_MPU6050:
    def init(self, _isDebugEnable:bool):
        self.i2cBus = smbus.SMBus(1)
        self.i2cBus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        self.isDebugEnable = _isDebugEnable
        
        self.gyroOffsets = {'x': 0, 'y': 0, 'z': 0}
        self.accelOffsets = {'x': 0, 'y': 0, 'z': 0}
        
        self.gyroOffsets = 0.0
        self.accelOffsets = 0.0
        
        self.acclX:float = 0.0
        self.acclY:float = 0.0
        self.acclZ:float = 0.0
        
        self.pitchRate:float = 0.0
        self.rollRate = 0.0
        self.yawRate = 0.0
        
    def _read2ByteData(self, _addr):
        highByte = self.i2cBus.read_byte_data(MPU6050_ADDR, _addr)
        lowByte = self.i2cBus.read_byte_data(MPU6050_ADDR, _addr + 1)
        readData = ((highByte << 8) | lowByte)
        if readData > 32768:
            readData = readData - 65536
        return readData
        
    def calibrate(self, _numberOfSamples=10):
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
    
    def getAccelAndGyro(self):
        _accX = self._read2ByteData(ACCEL_XOUT_H) - self.accelOffsets['x']
        _accY = self._read2ByteData(ACCEL_XOUT_H + 2) - self.accelOffsets['y']
        _accZ = self._read2ByteData(ACCEL_XOUT_H + 4) - self.accelOffsets['z']
    
        _gyroX = self._read2ByteData(GYRO_XOUT_H) - self.gyroOffsets['x']
        _gyroY = self._read2ByteData(GYRO_XOUT_H + 2) - self.gyroOffsets['y']
        _gyroZ = self._read2ByteData(GYRO_XOUT_H + 4) - self.gyroOffsets['z']
    
        _pitchRate = _gyroX / 131.0
        _rollRate = _gyroY / 131.0
        _yawRate = _gyroZ / 131.0
        
        self.acclX = _accX / 16384.0 * 9.80665
        self.acclY = _accY / 16384.0 * 9.80665
        self.acclZ = _accZ / 16384.0 * 9.80665
        
        self.pitchRate = _pitchRate
        self.rollRate = _rollRate
        self.yawRate = _yawRate


if __name__ == "__main__":
    # 初期化
    mpu = Cteamt_MPU6050()
    
    mpu.init(True)

    # センサーのキャリブレーション
    mpu.calibrate()

    prev_time = time.time()
    yaw_total = 0
    prev_yaw_rate = 0

    try:
        lastReadTime = time.perf_counter()
        while True:

            mpu.getAccelAndGyro()
                      
            nowTime = time.perf_counter()
            isProgramCycleOK = False

            while ((nowTime - lastReadTime) < (1.0 / READ_CYCLE)):
                isProgramCycleOK = True
                
                nowTime = time.perf_counter()
            
            if (isProgramCycleOK):
                lastReadTime = nowTime
            else:
                break
            
            print("aX:{:6.2f} aY:{:6.2f} aZ:{:6.2f} | pR:{:6.2f} rR:{:6.2f} yR:{:6.2f} | t{:.2f}".format(mpu.acclX, mpu.acclY, mpu.acclZ, mpu.pitchRate, mpu.rollRate, mpu.yawRate, nowTime))
            
            
        print("Faild Program Cycle")
                
    except KeyboardInterrupt:
        print("プログラム終了")
