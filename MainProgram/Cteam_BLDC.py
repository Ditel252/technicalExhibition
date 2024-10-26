# 必要なライブラリをインポート
import time
import sys
import pigpio
import math

# 定数定義
MAX_SIGNAL = 2000  # 最大パルス幅 [us]
MIN_SIGNAL = 1000  # 最小パルス幅 [us]
PWM_MAX = 40000 # PWMの幅

class Cteam_BLDC:        
    def setMaxValue(self):
        self.gpio.set_PWM_dutycycle(self.pwmPin, int((MAX_SIGNAL / 20000.0) * PWM_MAX))
        
    def setMinValue(self):
        self.gpio.set_PWM_dutycycle(self.pwmPin, int((MIN_SIGNAL / 20000.0) * PWM_MAX))
    
    def init(self, _gpio,_pin:int, _debug:bool):
        self.gpio = _gpio
        self.pwmPin:int = _pin
        self.isDebugEnable:bool = _debug
        
        self.gpio.set_PWM_range(self.pwmPin, 40000)
        self.gpio.set_PWM_frequency(self.pwmPin, 50)
        self.value:int = MIN_SIGNAL
        
        self.maxValueOfPercent:float = 1000
        
        # y = ax^2 + bx + c
        self.proximityFormula_coefficientA:float = 1.0
        self.proximityFormula_coefficientB:float = 1.0
        self.proximityFormula_coefficientC:float = 1.0
        
        self.convertedValueFormPWM:float = 0.0
        
    def toDutyFromPercent(self, _percent:float):
        _maxPWM:float = self.proximityFormula_coefficientA * ((self.maxValueOfPercent + MIN_SIGNAL) ** 2) + self.proximityFormula_coefficientB * (self.maxValueOfPercent + MIN_SIGNAL) + self.proximityFormula_coefficientC
        _minPWM:float = self.proximityFormula_coefficientA * (MIN_SIGNAL ** 2) + self.proximityFormula_coefficientB *MIN_SIGNAL + self.proximityFormula_coefficientC
        
        _requestPWM:float = (_percent / 100.0) * (_maxPWM - _minPWM) + _minPWM
        
        self.convertedValueFormPWM = (-1 * self.proximityFormula_coefficientB + math.sqrt((self.proximityFormula_coefficientB ** 2) - 4 * self.proximityFormula_coefficientA * (self.proximityFormula_coefficientC - _requestPWM))) / (2 * self.proximityFormula_coefficientA) - MIN_SIGNAL
        
        if(self.convertedValueFormPWM > self.maxValueOfPercent):
            self.convertedValueFormPWM = self.maxValueOfPercent
        
    def setValue(self, _value:int):
        # 入力が全て0, 0ならモータ停止
        if _value == 0:
            self.BLDC_stop()
        else:            
            # 新たなパルス幅が有効範囲内にあるか確認
            if MIN_SIGNAL <= (int(_value) + MIN_SIGNAL) <= MAX_SIGNAL:
                self.value = int(_value) + MIN_SIGNAL
                
                if(self.isDebugEnable):
                    print("Changing ESC(Pin {:d})".format(self.pwmPin))
                
                self.gpio.set_PWM_dutycycle(self.pwmPin, int((self.value / 20000.0) * PWM_MAX))
            else:
                if(self.isDebugEnable):
                    print("Invalid value for ESC(Pin {:d}). Value must be between {:d} and {:d}.".format(self.pwmPin, MIN_SIGNAL, MAX_SIGNAL))
                            
    def BLDC_stop(self):
        self.value = MIN_SIGNAL
        
        self.gpio.set_PWM_dutycycle(self.pwmPin, int((self.value / 20000.0) * PWM_MAX))
        
        if(self.isDebugEnable):
            print("Stop the motor of ESC(Pin {:d}).".format(self.pwmPin))
            
    def BLDC_off(self):
        self.value = 0
        
        self.gpio.set_PWM_dutycycle(self.pwmPin, int((self.value / 20000.0) * PWM_MAX))
        
        if(self.isDebugEnable):
            print("Off the motor of ESC(Pin {:d}).".format(self.pwmPin))

        
motor = Cteam_BLDC()

if __name__ == "__main__":
    gpio = pigpio.pi()
    
    motor.init(gpio, 17, True)
    
    motor.setMaxValue()
    time.sleep(2)
    motor.setMinValue()
    time.sleep(2)
    
    try:
        while True:
            motorValue = input("esc power : ")
            
            motor.setValue(motorValue)
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("program stop")
    
    finally:        
        print("esc off")
        
    motor.BLDC_off()
    gpio.stop()
    
    