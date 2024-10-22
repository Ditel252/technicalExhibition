import Cteam_MPU6050_Lite as mpu6050
from multiprocessing import Value, Array, Process
import time 

# MPU6050
mpuI2c = mpu6050.Cteam_MPU6050_I2c()
mpuCal = mpu6050.Cteam_MPU6050_Cal()
READ_CYCLE = 500 # [Hz]
CALIBRATION_TIME = 100

def readMPU6050(endReadPosture, readDataOfMPU6050, acclOffset, gyroOffset, wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer):
    print("{:<20} | Read MPU6050 Start".format("Read MPU6050"))
    mpuI2c.init(True)
    
    wasCalibrationFinished.value = 0
    print("{:<20} | Calibration Start".format("Read MPU6050"))
    
    mpuI2c.calibrate(CALIBRATION_TIME)
    
    acclOffset[0] = mpuI2c.acclOffsets['x']
    acclOffset[1] = mpuI2c.acclOffsets['y']
    acclOffset[2] = mpuI2c.acclOffsets['z']
    
    gyroOffset[0] = mpuI2c.gyroOffsets['x']
    gyroOffset[1] = mpuI2c.gyroOffsets['y']
    gyroOffset[2] = mpuI2c.gyroOffsets['z']    
    
    print("{:<20} | Calibration Finish".format("Read MPU6050"))   
    
    wasCalibrationFinished.value = 1
    
    _lastReadTime:float = time.perf_counter()
    _nowTime:float = time.perf_counter()
    
    _delayTimeCounter:int = 0
    
    while(not wasMeasureStarted.value):
        pass
    
    print("{:<20} | Measure Start".format("Read MPU6050"))
    
    while(not endReadPosture.value):
        _reciveData = mpuI2c._readAcclAndGyro_ReadOnly()
        
        
        for _i in range(0, 6, 1):
            readDataOfMPU6050[_i] = _reciveData[_i]
            readDataOfMPU6050[_i + 6] = _reciveData[_i + 8]
        
        _delayTimeCounter = 0
        
        while((_nowTime - _lastReadTime) <= (1.0 / READ_CYCLE)):
            _nowTime:float = time.perf_counter()
            _delayTimeCounter += 1
        
        readTimeBuffer.value = _delayTimeCounter
        
        if(_delayTimeCounter < 10):
            print("{:<20} | 1 Cycle Time Buffer is too short".format("Read MPU6050"))
        
        mesureTimeCount.value += 1
        _lastReadTime = _nowTime        
        
    print("{:<20} | Read MPU6050 Finish".format("Read MPU6050"))
    

def calPosture(endReadPosture, readDataOfMPU6050, accl, angleAccl, angleRate, angle, acclOffset, gyroOffset, wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer):
    print("{:<20} | Program Start".format("Calculate Prosture"))
    
    mpuCal.init(True)
    
    print("{:<20} | Wating For Calibration Finish".format("Calculate Prosture"))
    
    while(not wasCalibrationFinished.value):
        pass
    
    print("{:<20} | Calibration Finished".format("Calculate Prosture"))
    
    mpuCal.setCalibrationData(acclOffset, gyroOffset)
    
    print("{:<20} | Calibration Set".format("Calculate Prosture"))
    
    _nowMesureTimeCount:int = mesureTimeCount.value
    _lastMesureTimeCount:int = _nowMesureTimeCount
    
    # MPU6050 Measure Start
    wasMeasureStarted.value = 1
    print("{:<20} | Order Measure Start".format("Calculate Prosture"))
    
    _lastReadTime:float = time.perf_counter()
        
    for _ in range(0, 500 * 10, 1):
        _nowReadTime = time.perf_counter()
        _1CycleTime = _nowReadTime - _lastReadTime
        _lastReadTime = _nowReadTime
        
        while(_nowMesureTimeCount <= _lastMesureTimeCount):
            _nowMesureTimeCount = mesureTimeCount.value
        
        if(_nowMesureTimeCount != (_lastMesureTimeCount + 1)):
            print("\n{:<20} | Error : Attitude calculation process is out of sync.".format("Calculate Prosture"))
            
        _lastMesureTimeCount = _nowMesureTimeCount
        
        mpuCal.getAcclAndGyro(mpuCal._readAcclAndGyro_CalculateOnly(readDataOfMPU6050)) # Get Acceleration and Angular-velocity
        mpuCal.getAngleAcceleration(1.0 / READ_CYCLE) # Get Angle Acceleration
        mpuCal.getAngle(1.0 / READ_CYCLE)   # Get Angle
        
        accl[0] = mpuCal.acclX
        accl[1] = mpuCal.acclY
        accl[2] = mpuCal.acclZ
        
        angleAccl[0] = mpuCal.pitchAccl
        angleAccl[1] = mpuCal.rollAccl
        angleAccl[2] = mpuCal.yawAccl
        
        angleRate[0] = mpuCal.pitchRate
        angleRate[1] = mpuCal.rollRate
        angleRate[2] = mpuCal.yawRate
        
        angle[0] = mpuCal.pitch
        angle[1] = mpuCal.roll
        angle[2] = mpuCal.yaw
        
        # print("\raX:{:10.2f} aY:{:10.2f} aZ:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.acclX, mpuCal.acclY, mpuCal.acclZ, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        print("\rpR:{:10.2f} rR:{:10.2f} yR:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.pitchRate, mpuCal.rollRate, mpuCal.yawRate, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        # print("\rpA:{:10.2f} rA:{:10.2f} yA:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.pitchAccl, mpuCal.rollAccl, mpuCal.yawAccl, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        # print("\rp:{:10.2f} r:{:10.2f} y:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.pitch, mpuCal.roll, mpuCal.yaw, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        
    endReadPosture.value = 1
    print("{:<20} | Program End".format("Read Prosture"))
        
def mainProgram(endReadPosture, readDataOfMPU6050, accl, angleAccl, gyro, angle):    
    print("{:<20} | Program Start".format("Main"))
    
    for i in range(1000):
        # print(readDataOfMPU6050[0] << 8 | readDataOfMPU6050[1])
        mpu._readAcclAndGyro_CalculateOnly(readDataOfMPU6050)
        
        print("ax:{:6.2f} ay:{:6.2f} az:{:6.2f} | pr:{:6.2f} rr:{:6.2f} yr:{:6.2f}".format(mpu.acclX, mpu.acclY, mpu.acclZ, mpu.pitchRate, mpu.rollRate, mpu.yawRate))
        
        # print("ax:{:6.2f} ay:{:6.2f} az:{:6.2f} | p:{:6.2f} r:{:6.2f} y:{:6.2f} | pR:{:6.2f} rR:{:6.2f} yR:{:6.2f} | pA:{:6.2f} rA:{:6.2f} yA:{:6.2f}".format( \
        #     accl[0], accl[1], accl[2], angle[0], angle[1], angle[2], gyro[0], gyro[1], gyro[2], angleAccl[0], angleAccl[1], angleAccl[2]))
        
        time.sleep(0.1)
    
    print("{:<20} | Program End".format("Main"))
    endReadPosture.value = 1

if __name__ == "__main__":
    endReadPosture = Value('H', 0)
    wasCalibrationFinished = Value('H', 0)
    wasMeasureStarted = Value('H', 0)
    
    acclOffset = Array('f', 3)
    gyroOffset = Array('f', 3)
    readTimeBuffer = Value('i', 0)
    mesureTimeCount = Value('i', 0)
    
    readDataOfMPU6050 = Array('I', 12)
    
    accl = Array('f', 3)
    
    angleAccl = Array('f', 3)
    angleRate = Array('f', 3)
    angle = Array('f', 3)
    
    process_readMPU6050 = Process(target=readMPU6050, args=[endReadPosture, readDataOfMPU6050, acclOffset, gyroOffset, wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer])
    process_calPosture = Process(target=calPosture, args=[endReadPosture, readDataOfMPU6050, accl, angleAccl, angleRate, angle, acclOffset, gyroOffset, wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer])
    # process_mainProgram = Process(target=mainProgram, args=[endReadPosture, readDataOfMPU6050, accl, angleAccl, gyro, angle])
    
    # process_readPosture.start()
    process_readMPU6050.start()
    process_calPosture.start()
    # process_mainProgram.start()
    
    # process_readPosture.join()
    process_readMPU6050.join()
    process_calPosture.join()
    # process_mainProgram.join()
    
    del endReadPosture
    del wasCalibrationFinished
    del wasMeasureStarted
    del acclOffset
    del gyroOffset
    del readTimeBuffer
    del mesureTimeCount
    del readDataOfMPU6050
    del accl
    del angleAccl
    del angleRate
    del angle