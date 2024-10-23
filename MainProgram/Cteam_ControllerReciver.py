import serial

class Cteam_ControllerReciver:
    def init(self, _port:str):
        self.usbSerial = serial.Serial(_port, 9600)
        self.readByte:int = 0
        
    def getReadByte(self):
        _readByte = self.usbSerial.read()
        if(len(_readByte) > 0):
            self.readByte = int.from_bytes(_readByte)
            
            return True
        else:
            return False
        
if __name__ == "__main__":
    controller = Cteam_ControllerReciver()
    
    controller.init('/dev/ttyACM0')
    
    while(True):
        if(controller.getReadByte()):
            break
    
    print(controller.readByte)