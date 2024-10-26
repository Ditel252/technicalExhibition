import Cteam_BLDC
import pigpio
import time

motor = Cteam_BLDC.Cteam_BLDC()

if __name__ == "__main__":
    gpio = pigpio.pi()
    
    motor.init(gpio, 17, False)
    
    motor.maxValueOfPercent = 360
    
    motor.proximityFormula_coefficientA = 1
    motor.proximityFormula_coefficientB = 26.541
    motor.proximityFormula_coefficientC = -1743
    
    # motor.setMaxValue()
    # time.sleep(2)
    # motor.setMinValue()
    # time.sleep(2)
    
    try:
        for _percent in range(60, 120, 1):
            _lastConvertedValue = motor.convertedValueFormPWM
            motor.toDutyFromPercent(float(_percent))
            
            motor.setValue(motor.convertedValueFormPWM)
            print("\rper : {:3d} | duty : {:6.2f} | deltaDuty : {:6.2f}".format(_percent, motor.convertedValueFormPWM, motor.convertedValueFormPWM - _lastConvertedValue), end="")
            
            time.sleep(0.1)
            
        print("")
    
    except KeyboardInterrupt:
        print("program stop")
    
    finally:        
        print("esc off")
        
    motor.BLDC_off()
    gpio.stop()