import Cteam_MPU6050_Lite as mpu6050
from multiprocessing import Value, Array, Process
import time 

# MPU6050
mpu = mpu6050.Cteamt_MPU6050()
READ_CYCLE = 200 # [Hz]

def readMPU6050(endReadPosture, readDataOfMPU6050):    
    while(not endReadPosture.value):
        _reciveData = mpu._readAcclAndGyro_ReadOnly()
        
        for _i in range(0, 6, 1):
            readDataOfMPU6050[_i] = _reciveData[_i]
            readDataOfMPU6050[_i + 6] = _reciveData[_i + 8]
        
        time.sleep(0.2)
    

def readPosture(endReadPosture, accl, angleAccl, gyro, angle):
    print("{:<20} | Program Start".format("Read Prosture"))

    # センサーのキャリブレーション
    mpu.calibrate()
    
    lastReadTime = time.perf_counter()
    
    while (not endReadPosture.value):
        mpu.getAccelAndGyro()
        mpu.getAngle(1.0 / READ_CYCLE)
        mpu.getAngleAcceleration(1.0 / READ_CYCLE)
        
        accl[0] = mpu.acclX
        accl[1] = mpu.acclY
        accl[2] = mpu.acclZ
        
        angleAccl[0] = mpu.pitchAccl
        angleAccl[1] = mpu.rollAccl
        angleAccl[2] = mpu.yawAccl
        
        gyro[0] = mpu.pitchRate
        gyro[1] = mpu.rollRate
        gyro[2] = mpu.yawRate
        
        angle[0] = mpu.pitch
        angle[1] = mpu.roll
        angle[2] = mpu.yaw
                
        nowTime = time.perf_counter()
        isProgramCycleOK = False

        while ((nowTime - lastReadTime) <= (1.0 / READ_CYCLE)):
            isProgramCycleOK = True
            
            nowTime = time.perf_counter()
        
        if (isProgramCycleOK):
            lastReadTime = nowTime
        else:
            lastReadTime = nowTime
            print("{:<20} | Error !! Cycle is too short!!".format("Read Prosture"))
                
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
    mpu.init(True)
    
    endReadPosture = Value('H', 0)
    
    acclOffset = Array('f', 3)
    gyroOffset = Array('f', 3)
    
    readDataOfMPU6050 = Array('I', 12)
    
    accl = Array('f', 3)
    angleAccl = Array('f', 3)
    gyro = Array('f', 3)
    angle = Array('f', 3)
    
    process_readMPU6050 = Process(target=readMPU6050, args=[endReadPosture, readDataOfMPU6050])
    # process_readPosture = Process(target=readPosture, args=[endReadPosture, accl, angleAccl, gyro, angle])
    process_mainProgram = Process(target=mainProgram, args=[endReadPosture, readDataOfMPU6050, accl, angleAccl, gyro, angle])
    
    # process_readPosture.start()
    process_readMPU6050.start()
    process_mainProgram.start()
    
    # process_readPosture.join()
    process_readMPU6050.join()
    process_mainProgram.join()
    
    