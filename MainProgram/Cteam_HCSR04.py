import pigpio
import time
import sys

TEMPERTURE = 25
SOUND_VELOCITY = 331.5 + 0.61 * TEMPERTURE
TIMEOUT_LENGTH = 3.0 * 2.0 * 1.0 / SOUND_VELOCITY

UDS_TRIGER_PIN:int = [6, 19, 7, 16, 21]
UDS_ECHO_PIN:int = [13, 26, 1, 12, 20]

class Cteam_HCSR04:
    def init(self, _gpio,_triggerPin:int, _echoPin:int):
        self.gpio = _gpio
        self.triggerPin = _triggerPin
        self.echoPin = _echoPin
        
        self.gpio.set_mode(self.triggerPin, pigpio.OUTPUT)
        self.gpio.set_mode(self.echoPin, pigpio.OUTPUT)
        
        self.distance:float = 0.0

    def getDistance(self):
        self.gpio.gpio_trigger(self.triggerPin, 10, 1)  # 10マイクロ秒のパルスを送信
        _startTime = time.perf_counter()
        _stopTime = time.perf_counter()
        _timeout = _startTime + TIMEOUT_LENGTH
    
        while self.gpio.read(self.echoPin) == 0:
            _startTime = time.perf_counter()
            if _startTime > _timeout:
                print(" : ", _startTime - _timeout)
                return False
            
        while self.gpio.read(self.echoPin) == 1:
            _stopTime = time.perf_counter()
            if time.perf_counter() - _startTime >= 1:
                return False
        
        _elapsedTime = _stopTime - _startTime
        self.distance = (_elapsedTime * SOUND_VELOCITY) / 2  # 音速（34300 cm/s）で計算
        return True

if __name__ == "__main__":
    hcsr04 = Cteam_HCSR04()
    
    gpio = pigpio.pi()
    
    hcsr04.init(gpio, UDS_TRIGER_PIN[3], UDS_ECHO_PIN[3])
    
    while(True):
        if(not hcsr04.getDistance()):
            pass
        
        print("\rdistance = {:6.3f} [m]".format(hcsr04.distance), end="")
        time.sleep(0.005)