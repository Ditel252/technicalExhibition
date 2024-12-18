# ver 0.9.20
import Cteam_MPU6050_Lite as MPU6050
import Cteam_BLDC as BLDC
import Cteam_ControllerReciver as CERx
import Cteam_HCSR04 as HCSR04
import Cteam_PID as PID
import pigpio
from multiprocessing import Value, Array, Process
import time

CMD_START_SETUP_ESC     = 0x01
CMD_START_CALIBRATION   = 0x02
CMD_START_MEASUREING    = 0x03
CMD_ESC_START           = 0x04
CMD_TAKE_OFF            = 0x05
# CMD_START_HEIGHT_PD     = 0x06
CMD_END_PROGRAM         = 0x06

PHASE_START_SETUP_ESC   = 1
PHASE_START_CALIBRATION = 2
PHASE_START_MEASUREING  = 3
PHASE_ESC_START         = 4
PAHSE_TAKE_OFF          = 5
# PAHSE_START_HEIGHT_PD   = 6
PHASE_END_PROGRAM       = 6

MAX_BLDC_SPEED = 370
# BASE_BLDC_SPEED     = 275
BASE_BLDC_PERCENT = 85 # [%]
# START_BLDC_SPEED    = 310
START_BLDC_PERCENT = 92.5 # [%]
# WAITING_BLDC_SPEED  = 230
WAITING_BLDC_PERCENT = 75 # [%]
WAITTING_TIME_FOR_START_BLDC    = 250.0 # [ms]
WAITTING_TIME_FOR_CHANGE_BLDC   = 100.0 # [ms]

REF_HEIGHT = 0.35 # [m]

BLDC_BASE_GAIN = [1.0, 1.0, 1.006, 0.95,  0.89, 0.9425, 0.9575, 1.05]

SAFETY_STOPPER:bool = True

# MPU6050
mpuI2c = MPU6050.Cteam_MPU6050_I2c()
mpuCal = MPU6050.Cteam_MPU6050_Cal()

esc = [BLDC.Cteam_BLDC(), BLDC.Cteam_BLDC(), BLDC.Cteam_BLDC(), BLDC.Cteam_BLDC(), BLDC.Cteam_BLDC(), BLDC.Cteam_BLDC(), BLDC.Cteam_BLDC(), BLDC.Cteam_BLDC()]
uds = [HCSR04.Cteam_HCSR04(), HCSR04.Cteam_HCSR04(), HCSR04.Cteam_HCSR04(), HCSR04.Cteam_HCSR04(), HCSR04.Cteam_HCSR04()]
ESC_PWM_PIN:int = [17, 27, 22, 10, 9, 11, 0, 5]
UDS_TRIGER_PIN:int = [6, 19, 7, 16, 21]
UDS_ECHO_PIN:int = [13, 26, 1, 12, 20]

READ_CYCLE = 300 # [Hz]
CALIBRATION_TIME = 1000

def readMPU6050(endReadPosture, readDataOfMPU6050, acclOffset, gyroOffset, isCalibrationStart,wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer):
    print("{:<20} | Read MPU6050 Start".format("Read MPU6050"))
    mpuI2c.init(True)
    
    wasCalibrationFinished.value = 0
    print("{:<20} | Calibration Start".format("Read MPU6050"))
    
    while(not isCalibrationStart.value):
        pass
    
    mpuI2c.calibrate(CALIBRATION_TIME)
    
    acclOffset[0] = mpuI2c.acclOffsets['x']
    acclOffset[1] = mpuI2c.acclOffsets['y']
    acclOffset[2] = mpuI2c.acclOffsets['z']
    
    gyroOffset[0] = mpuI2c.gyroOffsets['x']
    gyroOffset[1] = mpuI2c.gyroOffsets['y']
    gyroOffset[2] = mpuI2c.gyroOffsets['z']    
    
    print("{:<20} | Calibration Finish".format("Read MPU6050"))   
    
    wasCalibrationFinished.value = 1
    
    _lastReadTime:float = 0.0
    _nowTime:float = 0.0
    
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
            if(_delayTimeCounter < 1):
                print("{:<20} | Waring : 1 Cycle Time Buffer is too short".format("Read MPU6050"))
            else:
                print("{:<20} | Error : 1 Cycle Time Buffer is too short".format("Read MPU6050"))                
        
        mesureTimeCount.value += 1
        _lastReadTime = _nowTime        
        
    print("{:<20} | Read MPU6050 Finish".format("Read MPU6050"))
    

def calPosture(endReadPosture, readDataOfMPU6050, accl, velocity, displacement, angleAccl, angleRate, angle, acclOffset, gyroOffset, wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer):
    print("{:<20} | Program Start".format("Calculate Prosture"))
    
    mpuCal.init(True)
    
    print("{:<20} | Waiting For Calibration Finish".format("Calculate Prosture"))
    
    while(not wasCalibrationFinished.value):
        pass
    
    print("{:<20} | Calibration Finished".format("Calculate Prosture"))
    
    mpuCal.setCalibrationData(acclOffset, gyroOffset)
    
    print("{:<20} | Calibration Set".format("Calculate Prosture"))
    
    _nowMesureTimeCount:int = mesureTimeCount.value
    _lastMesureTimeCount:int = _nowMesureTimeCount
    
    print("{:<20} | Calibration Finished".format("Calculate Prosture"))
    
    print("{:<20} | Waiting For Start Measure".format("Calculate Prosture"))
    while(not wasMeasureStarted.value):
        pass
    
    # MPU6050 Measure Start
    print("{:<20} | Measure Start".format("Calculate Prosture"))
    
    _lastReadTime:float = time.perf_counter()
        
    while (not endReadPosture.value):
        _nowReadTime = time.perf_counter()
        _1CycleTime = _nowReadTime - _lastReadTime
        _lastReadTime = _nowReadTime
        
        while(_nowMesureTimeCount <= _lastMesureTimeCount):
            _nowMesureTimeCount = mesureTimeCount.value
        
        if(_nowMesureTimeCount != (_lastMesureTimeCount + 1)):
            print("\n{:<20} | Error : Attitude calculation process is out of sync.".format("Calculate Prosture"))
            
        _lastMesureTimeCount = _nowMesureTimeCount
        
        mpuCal.getAcclAndGyro(mpuCal._readAcclAndGyro_CalculateOnly(readDataOfMPU6050)) # Get Acceleration and Angular-velocity
        mpuCal.getVelocity(1.0 / READ_CYCLE)    # Get Velocity
        mpuCal.getDisplacement(1.0 / READ_CYCLE)    # Get Displacement
        mpuCal.getAngleAcceleration(1.0 / READ_CYCLE) # Get Angle Acceleration
        mpuCal.getAngle(1.0 / READ_CYCLE)   # Get Angle
        
        accl[0] = mpuCal.acclX
        accl[1] = mpuCal.acclY
        accl[2] = mpuCal.acclZ
        
        velocity[0] = mpuCal.velocityX
        velocity[1] = mpuCal.velocityY
        velocity[2] = mpuCal.velocityZ
        
        displacement[0] = mpuCal.displacementX
        displacement[1] = mpuCal.displacementY
        displacement[2] = mpuCal.displacementZ
        
        angleAccl[0] = mpuCal.pitchAccl
        angleAccl[1] = mpuCal.rollAccl
        angleAccl[2] = mpuCal.yawAccl
        
        angleRate[0] = mpuCal.pitchRate
        angleRate[1] = mpuCal.rollRate
        angleRate[2] = mpuCal.yawRate
        
        angle[0] = mpuCal.pitch
        angle[1] = mpuCal.roll
        angle[2] = mpuCal.yaw
        
        # print("\raX:{:10.2f} aY:{:10.2f} aZ:{:10.2f} | RBT{:4d} | Cyoocle{:6.2f}".format(mpuCal.acclX, mpuCal.acclY, mpuCal.acclZ, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        # print("\rvX:{:10.2f} vY:{:10.2f} vZ:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.velocityX, mpuCal.velocityY, mpuCal.velocityZ, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        # print("\rX:{:10.2f} Y:{:10.2f} Z:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.displacementX, mpuCal.displacementY, mpuCal.displacementZ, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        # print("\rpR:{:10.2f} rR:{:10.2f} yR:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.pitchRate, mpuCal.rollRate, mpuCal.yawRate, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        # print("\rpA:{:10.2f} rA:{:10.2f} yA:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.pitchAccl, mpuCal.rollAccl, mpuCal.yawAccl, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        # print("\rp:{:10.2f} r:{:10.2f} y:{:10.2f} | RBT{:4d} | Cycle{:6.2f}".format(mpuCal.pitch, mpuCal.roll, mpuCal.yaw, readTimeBuffer.value, 1.0 / _1CycleTime), end="")
        
    print("{:<20} | Program End".format("Read Prosture"))

def safetyStopper(endReadPosture, permittedPhases, permitRequestPhases):
    print("{:<20} | Program Start".format("Safety Stopper"))
    
    controllerRx = CERx.Cteam_ControllerReciver()
    controllerRx.init('/dev/ttyACM0')
    
    print("{:<20} | Serial Setup Finish".format("Safety Stopper"))
    
    permittedPhases.value = 0
    
    
    if(SAFETY_STOPPER): # ESC Setup
        while(permitRequestPhases.value < PHASE_START_SETUP_ESC):
            pass
        
        print("{:<20} $ Waiting For Setup ESC Command".format("Safety Stopper"))
        while(not endReadPosture.value):
            if(controllerRx.getReadByte()):
                if(controllerRx.readByte == CMD_START_SETUP_ESC):
                    break
            time.sleep(0.01)
        print("{:<20} | Get Start Setup ESC Command".format("Safety Stopper"))
        
    permittedPhases.value = PHASE_START_SETUP_ESC
    
    
    if(SAFETY_STOPPER): # Calibration
        while(permitRequestPhases.value < PHASE_START_CALIBRATION):
            pass
        
        print("{:<20} $ Waiting For Start Calibration Command".format("Safety Stopper"))
        while(not endReadPosture.value):
            if(controllerRx.getReadByte()):
                if(controllerRx.readByte == CMD_START_CALIBRATION):
                    break
            time.sleep(0.01)
        print("{:<20} | Get Start Calibration Command".format("Safety Stopper"))
    
    permittedPhases.value = PHASE_START_CALIBRATION
    
        
    if(SAFETY_STOPPER): # Start Measureing
        while(permitRequestPhases.value < PHASE_START_MEASUREING):
            pass
        
        print("{:<20} $ Waiting For Start Measure Command".format("Safety Stopper"))
        while(not endReadPosture.value):
            if(controllerRx.getReadByte()):
                if(controllerRx.readByte == CMD_START_MEASUREING):
                    break
            time.sleep(0.01)
        print("{:<20} | Get Start Start Measure Command".format("Safety Stopper"))

    permittedPhases.value = PHASE_START_MEASUREING

    
    if(SAFETY_STOPPER): # ESC Start
        while(permitRequestPhases.value < PHASE_ESC_START):
            pass
        
        print("{:<20} $ Waiting For Ready Command".format("Safety Stopper"))
        while(not endReadPosture.value):
            if(controllerRx.getReadByte()):
                if(controllerRx.readByte == CMD_ESC_START):
                    break
            time.sleep(0.01)
        print("{:<20} | Get Ready Command".format("Safety Stopper"))

    permittedPhases.value = PHASE_ESC_START

    
    if(SAFETY_STOPPER): # Take Off
        while(permitRequestPhases.value < PAHSE_TAKE_OFF):
            pass
        
        print("{:<20} $ Waiting For Take Off Command".format("Safety Stopper"))
        while(not endReadPosture.value):
            if(controllerRx.getReadByte()):
                if(controllerRx.readByte == CMD_TAKE_OFF):
                    break
            time.sleep(0.01)
        print("{:<20} | Get Start Take Off Command".format("Safety Stopper"))

    permittedPhases.value = PAHSE_TAKE_OFF
    
    
    # if(SAFETY_STOPPER): # Start PD Height
    #     while(permitRequestPhases.value < PAHSE_START_HEIGHT_PD):
    #         pass
        
    #     print("{:<20} $ Waiting For Start PD Calibration Command".format("Safety Stopper"))
    #     while(not endReadPosture.value):
    #         if(controllerRx.getReadByte()):
    #             if(controllerRx.readByte == CMD_START_HEIGHT_PD):
    #                 break
    #         time.sleep(0.01)
    #     print("{:<20} | Get Start Start PD Calibration Command".format("Safety Stopper"))

    # permittedPhases.value = PAHSE_START_HEIGHT_PD

    
    if(SAFETY_STOPPER): # Program End
        while(permitRequestPhases.value < PHASE_END_PROGRAM):
            pass
        
        print("{:<20} $ Waiting For End Program Command".format("Safety Stopper"))
        while(not endReadPosture.value):
            if(controllerRx.getReadByte()):
                if(controllerRx.readByte == CMD_END_PROGRAM):
                    break
            time.sleep(0.01)
        print("{:<20} | Get Start End Program Command".format("Safety Stopper"))

    permittedPhases.value = PHASE_END_PROGRAM
        
def mainProgram(endReadPosture, accl, velocity, displacement, angleAccl, angleRate, angle, isCalibrationStart, wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer, permittedPhases, permitRequestPhases):    
    print("{:<20} | Program Start".format("Main Program"))
    
    _nowMesureTimeCount:int = mesureTimeCount.value
    _lastMesureTimeCount:int = _nowMesureTimeCount
    
    # ====PID Setting(from here)====
    PID_Gyro = [PID.Cteam_PID(), PID.Cteam_PID(), PID.Cteam_PID()]
    
    for _gyroNum in range(0, 3, 1):
        PID_Gyro[_gyroNum].enableKi = 0
        PID_Gyro[_gyroNum].enableKp = 1
        PID_Gyro[_gyroNum].enableKd = 1
        
        PID_Gyro[_gyroNum].K_I = 0.0
        PID_Gyro[_gyroNum].K_P = 2.0
        PID_Gyro[_gyroNum].K_D = 2.25
        
        PID_Gyro[_gyroNum].init()
    
    
    PID_Accl = PID.Cteam_PID()
    
    PID_Accl.enableKi = 1
    PID_Accl.enableKp = 1
    PID_Accl.enableKd = 1
    
    PID_Accl.K_I = 1
    PID_Accl.K_P = 1
    PID_Accl.K_D = 1
    
    PID_Accl.init()
    
    PID_Height = PID.Cteam_PID()
    
    PID_Height.enableKi = 0
    PID_Height.enableKp = 1
    PID_Height.enableKd = 1
    
    PID_Height.K_I = 0
    PID_Height.K_P = 75
    PID_Height.K_D = 75
    
    PID_Height.init()
    # ====PID Setting(this far)====
    
    # ===Waiting Command From Controller(from here)===
    permitRequestPhases.value = PHASE_START_SETUP_ESC
    
    while(permittedPhases.value < PHASE_START_SETUP_ESC):
        pass
    # ===Waiting Command From Controller(this far)===
    
    
    _gpio = pigpio.pi()
    
    for _escNum in range(0, 8, 1):
        esc[_escNum].init(_gpio, ESC_PWM_PIN[_escNum], False)
        
        esc[_escNum].maxValueOfMaxPercent = MAX_BLDC_SPEED
    
    # ===Setting coefficient(from here)===
    esc[0].proximityFormula_coefficientA = -0.008
    esc[0].proximityFormula_coefficientB = 26.5
    esc[0].proximityFormula_coefficientC = -17000
    
    esc[1].proximityFormula_coefficientA = -0.008
    esc[1].proximityFormula_coefficientB = 26.5
    esc[1].proximityFormula_coefficientC = -17000
    
    esc[2].proximityFormula_coefficientA = -0.008
    esc[2].proximityFormula_coefficientB = 26.5
    esc[2].proximityFormula_coefficientC = -17000
    
    esc[3].proximityFormula_coefficientA = -0.008
    esc[3].proximityFormula_coefficientB = 26.5
    esc[3].proximityFormula_coefficientC = -17000
    
    
    esc[4].proximityFormula_coefficientA = -0.01
    esc[4].proximityFormula_coefficientB = 32.344
    esc[4].proximityFormula_coefficientC = -21043
    
    esc[5].proximityFormula_coefficientA = -0.008
    esc[5].proximityFormula_coefficientB = 26.5
    esc[5].proximityFormula_coefficientC = -17000
    
    esc[6].proximityFormula_coefficientA = -0.008
    esc[6].proximityFormula_coefficientB = 26.5
    esc[6].proximityFormula_coefficientC = 17000
    
    esc[7].proximityFormula_coefficientA = -0.008
    esc[7].proximityFormula_coefficientB = 26.5
    esc[7].proximityFormula_coefficientC = -17000
    # ===Setting coefficient(this far)===
        
    for _udsNum in range(0, 5, 1):
        uds[_udsNum].init(_gpio, UDS_TRIGER_PIN[_udsNum], UDS_ECHO_PIN[_udsNum])
    
    print("{:<20} | Set Max and Min Value to ESC Start".format("Main Program"))
    
    for _escNum in range(0, 8, 1):
        esc[_escNum].setMaxValue()
        time.sleep(0.2)
        
    time.sleep(2 - 0.2 * 8)

    for _escNum in range(0, 8, 1):
        esc[_escNum].setMinValue()
        time.sleep(0.2)
        
    print("{:<20} | Set Max and Min Value to ESC End".format("Main Program"))
    
    
    # ===Waiting Command From Controller(from here)===
    permitRequestPhases.value = PHASE_START_CALIBRATION
    
    while(permittedPhases.value < PHASE_START_CALIBRATION):
        pass
    # ===Waiting Command From Controller(this far)===
        
        
    isCalibrationStart.value = 1
    
    while (not wasCalibrationFinished.value):
        pass
    
    # ===Waiting Command From Controller(from here)===
    permitRequestPhases.value = PHASE_START_MEASUREING
    
    while(permittedPhases.value < PHASE_START_MEASUREING):
        pass
    # ===Waiting Command From Controller(this far)===
    
    wasMeasureStarted.value = 1
    
    print("{:<20} | Order Measure Start".format("Main Program"))
    
    
    # ===Waiting Command From Controller(from here)===
    permitRequestPhases.value = PHASE_ESC_START
    
    while(permittedPhases.value < PHASE_ESC_START):
        pass
    # ===Waiting Command From Controller(this far)===
    
    # For BLDC 1
    for _escNum in range(0, 8, 1):
        esc[_escNum].toDutyFromPercent(START_BLDC_PERCENT * BLDC_BASE_GAIN[_escNum])
        _startBLDCValue:float = esc[_escNum].convertedValueFormPWM
        
        esc[_escNum].setValue(int(_startBLDCValue))
        
    time.sleep(0.2)
    
    for _escNum in range(0, 8, 1):
        esc[_escNum].toDutyFromPercent(WAITING_BLDC_PERCENT * BLDC_BASE_GAIN[_escNum])
        _waittingBLDCValue:float = esc[_escNum].convertedValueFormPWM
        
        esc[_escNum].setValue(int(_waittingBLDCValue))
        
    time.sleep(0.1)
    print("{:<20} | BLDC STARTED".format("Main Program"))
    
    # for _escNum in range(0, 8, 1):
    #     esc[_escNum].setValue(WAITING_BLDC_SPEED)
    
    # ===Waiting Command From Controller(from here)===
    permitRequestPhases.value = PAHSE_TAKE_OFF
    
    while(permittedPhases.value < PAHSE_TAKE_OFF):
        pass
    # ===Waiting Command From Controller(this far)===
    
    
    permitRequestPhases.value = PHASE_END_PROGRAM
    
    _velocityOfHeight:float = 0.0
    if(uds[4].getDistance() == False):
        print("{:<20} | False Get Heignt".format("Main Program"))
    time.sleep(0.003)
    
    _lastReadTime:float = -1 # [s]
    
    while(permittedPhases.value < PHASE_END_PROGRAM):
        _escSpeedSum:float = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        for _escNum in range(0, 8, 1):
            esc[_escNum].toDutyFromPercent(BASE_BLDC_PERCENT * BLDC_BASE_GAIN[_escNum])
            _escSpeedSum[_escNum] =  esc[_escNum].convertedValueFormPWM
        
        # Begin MainProgram While from here
        
        
        # permitRequestPhases.value = PHASE_END_PROGRAM
        
        if(True):
            permitRequestPhases.value = PHASE_END_PROGRAM
            
            _lastHeight = uds[4].distance
            
            if(uds[4].getDistance() == False):
                print("{:<20} | False Get Heignt".format("Main Program"))
                
            time.sleep(0.003)
            
            PID_Gyro[_gyroNum].K_I = 0.0 / 1.15
            PID_Gyro[_gyroNum].K_P = 2.0
            PID_Gyro[_gyroNum].K_D = 2.25
            
            _nowReadTime = time.perf_counter()
            if(_lastReadTime >= 0.0):
                PID_Height.PID(0, REF_HEIGHT - uds[4].distance, float(_lastHeight - uds[4].distance) / float(_nowReadTime - _lastReadTime))
            else:
                PID_Height.PID(0, REF_HEIGHT - uds[4].distance, 0)
                
            _lastReadTime = _nowReadTime
                
            for _emcNum in range(0, 8, 1):
                _escSpeedSum[_emcNum] += PID_Height.ans
        
        PID_Gyro[1].PID(0, angle[1], angleRate[1])
    
        PID_Gyro[0].PID(0, angle[0], angleRate[0])
                
        _escSpeedSum[0] += 2 * PID_Gyro[1].ans
        _escSpeedSum[1] += 1 * PID_Gyro[1].ans
        _escSpeedSum[2] += 0 * PID_Gyro[1].ans
        _escSpeedSum[3] += -1 * PID_Gyro[1].ans
        _escSpeedSum[4] += -2 * PID_Gyro[1].ans
        _escSpeedSum[5] += -1 * PID_Gyro[1].ans
        _escSpeedSum[6] += 0 * PID_Gyro[1].ans
        _escSpeedSum[7] += 1 * PID_Gyro[1].ans
        
        _escSpeedSum[0] += 0 * PID_Gyro[0].ans
        _escSpeedSum[1] += -1 * PID_Gyro[0].ans
        _escSpeedSum[2] += -2 * PID_Gyro[0].ans
        _escSpeedSum[3] += -1 * PID_Gyro[0].ans
        _escSpeedSum[4] += 0 * PID_Gyro[0].ans
        _escSpeedSum[5] += 1 * PID_Gyro[0].ans
        _escSpeedSum[6] += 2 * PID_Gyro[0].ans
        _escSpeedSum[7] += 1 * PID_Gyro[0].ans
        
        for _escNum in range(0, 8, 1):
            _escSpeedSum[_escNum] *= BLDC_BASE_GAIN[_escNum]
            if(_escSpeedSum[_escNum] > 370):
                _escSpeedSum[_escNum] = 370
        
        for _escNum in range(0, 8, 1):
            esc[_escNum].setValue(int(_escSpeedSum[_escNum]))
            
        print("\r{:3d} {:3d} {:3d} {:3d} | {:3d} {:3d} {:3d} {:3d}".format(int(_escSpeedSum[0]), int(_escSpeedSum[1]), int(_escSpeedSum[2]), int(_escSpeedSum[3]), int(_escSpeedSum[4]), int(_escSpeedSum[5]), int(_escSpeedSum[6]), int(_escSpeedSum[7])), end="")
        
    
    for _escNum in range(0, 8, 1):
        esc[_escNum].BLDC_off()
    
    endReadPosture.value = 1
    print("{:<20} | Program End".format("Main Program"))

if __name__ == "__main__":
    endReadPosture = Value('H', 0)
    wasCalibrationFinished = Value('H', 0)
    isCalibrationStart = Value('H', 0)
    wasMeasureStarted = Value('H', 0)
    permittedPhases = Value('H', 0)
    permitRequestPhases = Value('H', 0)
    
    acclOffset = Array('f', 3)
    gyroOffset = Array('f', 3)
    readTimeBuffer = Value('i', 0)
    mesureTimeCount = Value('i', 0)
    
    readDataOfMPU6050 = Array('I', 12)
    
    accl = Array('f', 3)
    velocity = Array('f', 3)
    displacement = Array('f', 3)
    
    angleAccl = Array('f', 3)
    angleRate = Array('f', 3)
    angle = Array('f', 3)
    
    process_readMPU6050 = Process(target=readMPU6050, args=[endReadPosture, readDataOfMPU6050, acclOffset, gyroOffset, isCalibrationStart,wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer])
    process_calPosture = Process(target=calPosture, args=[endReadPosture, readDataOfMPU6050, accl, velocity, displacement, angleAccl, angleRate, angle, acclOffset, gyroOffset, wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer])
    process_safetyStopper = Process(target=safetyStopper, args=[endReadPosture, permittedPhases, permitRequestPhases])
    process_mainProgram = Process(target=mainProgram, args=[endReadPosture, accl, velocity, displacement, angleAccl, angleRate, angle, isCalibrationStart, wasCalibrationFinished, wasMeasureStarted, mesureTimeCount, readTimeBuffer, permittedPhases, permitRequestPhases])
    
    # Safety Stopper Start
    process_safetyStopper.start()
    # Main Program Start
    process_mainProgram.start()
    # Postrue Caluculate Start
    process_calPosture.start()    
    # MPU6050 Read Start
    process_readMPU6050.start()
    
    process_safetyStopper.join()
    process_readMPU6050.join()
    process_calPosture.join()
    process_mainProgram.join()