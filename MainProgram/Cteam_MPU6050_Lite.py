import smbus
import math
import time

# MPU6050のI2Cアドレス
MPU6050_ADDR = 0x68

READ_CYCLE = 500 #[Hz] 470Hz Max

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
    
    def _read12ByteData(self, _addr):
        _readBytes = self.i2cBus.read_i2c_block_data(MPU6050_ADDR, _addr, (0x43 - 0x3B) + 6)
        _readData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        _readData[0] = ((_readBytes[0] << 8) | _readBytes[1])
        _readData[1] = ((_readBytes[2] << 8) | _readBytes[3])
        _readData[2] = ((_readBytes[4] << 8) | _readBytes[5])
        _readData[3] = ((_readBytes[0 + (0x43 - 0x3B)] << 8) | _readBytes[1 + (0x43 - 0x3B)])
        _readData[4] = ((_readBytes[2 + (0x43 - 0x3B)] << 8) | _readBytes[3 + (0x43 - 0x3B)])
        _readData[5] = ((_readBytes[4 + (0x43 - 0x3B)] << 8) | _readBytes[5 + (0x43 - 0x3B)])
        
        for _i in range(0, 6, 1):
            if (_readData[_i] > 32768):
                _readData[_i] = _readData[_i] - 65536
        
        return _readData
        
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
        
        _readData = self._read12ByteData(ACCEL_XOUT_H)
        
        self.acclX = (_readData[0] - self.accelOffsets['x']) * 5.985504150390625E-4 # 9.80665 / 16384.0
        self.acclY = (_readData[1] - self.accelOffsets['y']) * 5.985504150390625E-4
        self.acclZ = (_readData[2] - self.accelOffsets['z']) * 5.985504150390625E-4
    
        self.pitchRate =    (_readData[3] - self.gyroOffsets['x']) / 131.0
        self.rollRate =     (_readData[4] - self.gyroOffsets['y']) / 131.0
        self.yawRate =      (_readData[5] - self.gyroOffsets['z']) / 131.0


if __name__ == "__main__":
    # 初期化
    mpu = Cteamt_MPU6050()
    
    mpu.init(True)

    # センサーのキャリブレーション
    mpu.calibrate()

    prev_time = time.time()
    yaw_total = 0
    prev_yaw_rate = 0
    
    tryStartTime = 0.0
    tryEndTime = 0.0

    try:
        while True:
            lastReadTime = tryStartTime = time.perf_counter()
            
            while True:
                startTime = time.perf_counter_ns()
                mpu.getAccelAndGyro()
                endTime = time.perf_counter_ns()
                
                # print("\r{:f}".format(endTime - startTime), end="")
                        
                nowTime = tryEndTime =time.perf_counter()
                isProgramCycleOK = False

                while ((nowTime - lastReadTime) <= (1.0 / READ_CYCLE)):
                    isProgramCycleOK = True
                    
                    nowTime = time.perf_counter()
                
                if (isProgramCycleOK):
                    print("\rf={:10.6f} Err={:e}".format(1.0 / (nowTime - lastReadTime), READ_CYCLE - 1.0 / (nowTime - lastReadTime)),end="")
                    lastReadTime = nowTime
                else:
                    break
                
                # print("\raX:{:6.2f} aY:{:6.2f} aZ:{:6.2f} | pR:{:6.2f} rR:{:6.2f} yR:{:6.2f} | t{:.2f}".format(mpu.acclX, mpu.acclY, mpu.acclZ, mpu.pitchRate, mpu.rollRate, mpu.yawRate, nowTime), end="")
                
                
            print("\tFaild Program Cycle. Time={:f}".format(tryEndTime - tryStartTime))
            
            READ_CYCLE -= 5
                
    except KeyboardInterrupt:
        print("プログラム終了")
