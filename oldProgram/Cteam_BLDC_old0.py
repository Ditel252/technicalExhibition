# 必要なライブラリをインポート
import time
import sys
from gpiozero import PWMOutputDevice

# 定数定義
MAX_SIGNAL = 2000  # 最大パルス幅 [us]
MIN_SIGNAL = 1000  # 最小パルス幅 [us]

class Cteam_BLDC:
    def _toDutyCycle(self, _value:int):
        self.value = _value
        return self.value / 20000.0
        
    def setMaxValue(self):
        self.escDevice.value = self._toDutyCycle(MAX_SIGNAL)
        
    def setMinValue(self):
        self.escDevice.value = self._toDutyCycle(MIN_SIGNAL)        
    
    def init(self, _pin:int, _debug:bool):
        self.pwmPin:int = _pin
        self.isDebugEnable:bool = _debug
        
        self.escDevice = PWMOutputDevice(self.pwmPin, frequency=50)
        self.value:int = MIN_SIGNAL
        
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
                
                self.escDevice.value = self._toDutyCycle(self.value)
            else:
                if(self.isDebugEnable):
                    print("Invalid value for ESC(Pin {:d}). Value must be between {:d} and {:d}.".format(self.pwmPin, MIN_SIGNAL, MAX_SIGNAL))
                            
    def BLDC_stop(self):
        self.value = MIN_SIGNAL
        
        self.escDevice.value = self._toDutyCycle(self.value)
        
        if(self.isDebugEnable):
            print("Stop the motor of ESC(Pin {:d}).".format(self.pwmPin))
            
    def escOff(self):
        self.BLDC_stop()
        
        self.escDevice.off()

        
motor = Cteam_BLDC()

if __name__ == "__main__":
    motor.init(17, True)
    
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
        motor.escOff()
        
        print("esc off")
    
    