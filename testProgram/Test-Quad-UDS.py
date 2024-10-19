from gpiozero import DistanceSensor
import time
import sys
 
# Define pin numbers
Trigger1 = 11 #3Bでは27が使えるか不安なので変更
Echo1 = 12    #同じ素子でピンが隣り合うように
Trigger2 = 15
Echo2 = 16
Trigger3 =35
Echo3 =36
Trigger4 = 37
Echo4 = 38 

 
# Create DistanceSensor objects
sensor1 = DistanceSensor(trigger=Trigger1, echo=Echo1)
sensor2 = DistanceSensor(trigger=Trigger2, echo=Echo2)
sensor3 = DistanceSensor(trigger=Trigger3, echo=Echo3)
sensor4 = DistanceSensor(trigger=Trigger4, echo=Echo4)
 
# Continuously read values
try:
    while True:
        distance1 = sensor1.distance * 100  # Distance in cm
        distance2 = sensor2.distance * 100  # Distance in cm
        distance3 = sensor3.distance * 100
        distance4 = sensor4.distance * 100
 
        if 2 < distance1 < 400:
            print("Sensor 1 distance =", int(distance1), "cm")
        if 2 < distance2 < 400:
            print("Sensor 2 distance =", int(distance2), "cm")
        if 2 < distance3 < 400:
            print("Sensor 3 distance =", int(distance3), "cm")
        if 2 < distance4 < 400:
            print("Sensor 4 distance =", int(distance4), "cm")
        time.sleep(1)
 
except KeyboardInterrupt:  # Exit on Ctrl+C
    print("Exiting program")
    sys.exit()