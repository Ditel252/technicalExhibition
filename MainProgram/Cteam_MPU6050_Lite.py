import smbus
import math
import time

# MPU6050のI2Cアドレス
MPU6050_ADDR = 0x68

READ_CYCLE = 300 #[Hz] 470Hz Max

CALIBRATION_TIME = 1000

# レジスタ
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

lastReadTime:float = 0.0
isProgramCycleOK:bool = False

class Simpson:
    def init(self):
        self.gyroOffsets = {'x': 0, 'y': 0, 'z': 0}
        self.acclOffsets = {'x': 0, 'y': 0, 'z': 0}
        
        self.sumOfEven:float = 0.0
        self.sumOfOdd:float = 0.0
        self.sumCount:int = 0
        
        self.ans:float = 0.0

    def simpson(self, _value:float, _dt:float):
        if(self.sumCount == 1): # 奇数の時の式
            self.sumOfOdd += _value
            
            self.sumCount = 0
            return False
        else:   # 偶数の時の式
            self.ans = (_dt / 3.0) * (0.0 + _value + 4.0 * self.sumOfOdd + 2.0 * self.sumOfEven)
            
            self.sumOfEven += _value
            self.sumCount += 1
            
            return True
        
class Cteam_MPU6050_I2c:
    def init(self, _isDebugEnable:bool):
        self.i2cBus = smbus.SMBus(1)
        self.i2cBus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        self.isDebugEnable = _isDebugEnable
    
    def _read2ByteData(self, _addr):
        highByte = self.i2cBus.read_byte_data(MPU6050_ADDR, _addr)
        lowByte = self.i2cBus.read_byte_data(MPU6050_ADDR, _addr + 1)
        readData = ((highByte << 8) | lowByte)
        if readData > 32768:
            readData = readData - 65536
        return readData
    
    def _readAcclAndGyro(self):
        _readBytes = self.i2cBus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, (0x43 - 0x3B) + 6)
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
    
    def _readAcclAndGyro_ReadOnly(self): # return 14Bytes
        return self.i2cBus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, (0x43 - 0x3B) + 6)
    
    def calibrate(self, _numberOfSamples=CALIBRATION_TIME):
        _gyroOffsets = {'x': 0, 'y': 0, 'z': 0}
        _acclOffsets = {'x': 0, 'y': 0, 'z': 0}
        
        if(self.isDebugEnable):
            print("Start Calibration")

        for _ in range(_numberOfSamples):
            # ジャイロデータの平均を計算
            _gyroOffsets['x'] += self._read2ByteData(GYRO_XOUT_H)
            _gyroOffsets['y'] += self._read2ByteData(GYRO_XOUT_H + 2)
            _gyroOffsets['z'] += self._read2ByteData(GYRO_XOUT_H + 4)

            # 加速度データの平均を計算
            _acclOffsets['x'] += self._read2ByteData(ACCEL_XOUT_H)
            _acclOffsets['y'] += self._read2ByteData(ACCEL_XOUT_H + 2)
            _acclOffsets['z'] += self._read2ByteData(ACCEL_XOUT_H + 4)

            time.sleep(0.001)  # サンプリングの間隔を調整

        # 平均を取ってオフセット値を算出
        _gyroOffsets['x'] /= _numberOfSamples
        _gyroOffsets['y'] /= _numberOfSamples
        _gyroOffsets['z'] /= _numberOfSamples

        _acclOffsets['x'] /= _numberOfSamples
        _acclOffsets['y'] /= _numberOfSamples
        _acclOffsets['z'] /= _numberOfSamples

        # Z軸は地球重力があるのでその分を補正 (1g = 16384 LSB)
        _acclOffsets['z'] -= 16384
        
        if(self.isDebugEnable):
            print("Finish Calibration")
            
        self.gyroOffsets = _gyroOffsets
        self.acclOffsets = _acclOffsets

class Cteam_MPU6050_Cal:
    def init(self, _isDebugEnable:bool):
        self.isDebugEnable = _isDebugEnable
        
        self.gyroOffsets = {'x': 0, 'y': 0, 'z': 0}
        self.acclOffsets = {'x': 0, 'y': 0, 'z': 0}
        
        self.acclX:float = 0.0
        self.acclY:float = 0.0
        self.acclZ:float = 0.0
        
        self.pitchRate:float = 0.0
        self.rollRate:float = 0.0
        self.yawRate:float = 0.0
        
        self.pitchRateMem:float = 0.0
        self.rollRateMem:float = 0.0
        self.yawRateMem:float = 0.0
        
        self.pitchAccl:float = 0.0
        self.rollAccl:float = 0.0
        self.yawAccl:float = 0.0
        
        self.pitch:float = 0.0
        self.roll:float = 0.0
        self.yaw:float = 0.0
        
        self.pitchSimpson = Simpson()
        self.rollSimpson = Simpson()
        self.yawSimpson = Simpson()
        
        self.pitchSimpson.init()
        self.rollSimpson.init()
        self.yawSimpson.init()
        
    def setCalibrationData(self, _acclOffset, _gyroOffset):
        self.acclOffsets['x'] = _acclOffset[0]
        self.acclOffsets['y'] = _acclOffset[1]
        self.acclOffsets['z'] = _acclOffset[2]
        
        self.gyroOffsets['x'] = _gyroOffset[0]
        self.gyroOffsets['y'] = _gyroOffset[1]
        self.gyroOffsets['z'] = _gyroOffset[2]
    
    def _readAcclAndGyro_CalculateOnly(self, _readBytes):
        _readData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        _readData[0] = ((_readBytes[0] << 8) | _readBytes[1])
        _readData[1] = ((_readBytes[2] << 8) | _readBytes[3])
        _readData[2] = ((_readBytes[4] << 8) | _readBytes[5])
        _readData[3] = ((_readBytes[6] << 8) | _readBytes[7])
        _readData[4] = ((_readBytes[8] << 8) | _readBytes[9])
        _readData[5] = ((_readBytes[10] << 8) | _readBytes[11])
        
        for _i in range(0, 6, 1):
            if (_readData[_i] > 32768):
                _readData[_i] = _readData[_i] - 65536
        
        return _readData
    
    def getAcclAndGyro(self, _readData):
        self.acclX = (_readData[0] - self.acclOffsets['x']) * 5.985504150390625E-4 # 9.80665 / 16384.0
        self.acclY = (_readData[1] - self.acclOffsets['y']) * 5.985504150390625E-4
        self.acclZ = (_readData[2] - self.acclOffsets['z']) * 5.985504150390625E-4
    
        self.pitchRate =    (_readData[3] - self.gyroOffsets['x']) / 131.0
        self.rollRate =     (_readData[4] - self.gyroOffsets['y']) / 131.0
        self.yawRate =      (_readData[5] - self.gyroOffsets['z']) / 131.0
        
    def getAngle(self, _dt):
        self.pitchSimpson.simpson(self.pitchRate, _dt)
        self.rollSimpson.simpson(self.rollRate, _dt)
        self.yawSimpson.simpson(self.yawRate, _dt)
        
        self.pitch = self.pitchSimpson.ans
        self.roll = self.rollSimpson.ans
        self.yaw = self.yawSimpson.ans
        
    def getAngleAcceleration(self, _dt):
        self.pitchAccl = (self.pitchRate - self.pitchRateMem) / _dt
        self.rollAccl = (self.rollRate - self.rollRateMem) / _dt
        self.yawAccl = (self.yawRate - self.yawRateMem) / _dt
        
        self.pitchRateMem = self.pitchRate
        self.rollRateMem = self.rollRate
        self.yawRate = self.yawRate
        

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
                mpu.getAcclAndGyro()
                        
                nowTime = tryEndTime =time.perf_counter()
                isProgramCycleOK = False

                while ((nowTime - lastReadTime) <= (1.0 / READ_CYCLE)):
                    isProgramCycleOK = True
                    
                    nowTime = time.perf_counter()
                
                if (isProgramCycleOK):
                    # print("\rf={:10.6f} Err={:e}".format(1.0 / (nowTime - lastReadTime), READ_CYCLE - 1.0 / (nowTime - lastReadTime)),end="")
                    lastReadTime = nowTime
                else:
                    break
                
                mpu.getAngle(mpu.pitchRate, mpu.rollRate, mpu.yawRate, 1.0 / READ_CYCLE)
                
                # print("\raX:{:6.2f} aY:{:6.2f} aZ:{:6.2f} | pR:{:6.2f} rR:{:6.2f} yR:{:6.2f} | t{:.2f}".format(mpu.acclX, mpu.acclY, mpu.acclZ, mpu.pitchRate, mpu.rollRate, mpu.yawRate, nowTime), end="")
                print("\rroll:{:6.2f} pitch:{:6.2f} yaw:{:6.2f}".format(mpu.pitch, mpu.roll, mpu.yaw), end="")
                
                
            print("\tFaild Program Cycle. Time={:f}".format(tryEndTime - tryStartTime))
            
            READ_CYCLE -= 5
                
    except KeyboardInterrupt:
        print("プログラム終了")
