from Cteam_BLDC import Cteam_BLDC
import time

motor = [Cteam_BLDC(), Cteam_BLDC(), Cteam_BLDC(), Cteam_BLDC(),  Cteam_BLDC(), Cteam_BLDC(), Cteam_BLDC(), Cteam_BLDC()]

motor[0].init(17, True)
motor[1].init(27, True)
motor[2].init(22, True)
motor[3].init(10, True)

motor[4].init(9, True)
motor[5].init(11, True)
motor[6].init(0, True)
motor[7].init(5, True)

print("= Start ESC Init =")
for i in range(0, 8, 1):
    motor[i].setMaxValue()
    time.sleep(0.2)
    
time.sleep(2 - 0.2 * 8)

for i in range(0, 8, 1):
    motor[i].setMinValue()
    time.sleep(0.2)
    
print("= Finish ESC Init =")

try:
    while True:
        motorValue = input("esc power : ")
        
        for i in range(0, 8, 1):
            motor[i].setValue(motorValue)
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("program stop")

finally:
    for i in range(0, 8, 1):
        motor[i].escOff()
    
    print("esc off")