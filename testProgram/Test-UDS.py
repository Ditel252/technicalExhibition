import RPi.GPIO as GPIO
import time
import sys

# ポート番号の定義
Trigger1 = 27
Trigger2 = 17
Echo1 = 16
Echo2 = 18

# GPIOの設定
GPIO.setmode(GPIO.BCM)              # GPIOのモードを"GPIO.BCM"に設定
GPIO.setup(Trigger1, GPIO.OUT)
GPIO.setup(Trigger2, GPIO.OUT)
GPIO.setup(Echo1, GPIO.IN)
GPIO.setup(Echo2, GPIO.IN)

# HC-SR04で距離を測定する関数
def read_distance():
    # トリガーピンをHighに設定
    GPIO.output(Trigger1, GPIO.HIGH)
    GPIO.output(Trigger2, GPIO.HIGH)
    time.sleep(0.00001)  # 10μ秒間待つ
    GPIO.output(Trigger1, GPIO.LOW)
    GPIO.output(Trigger2, GPIO.LOW)

    # Echoピンから信号を受け取る
    while GPIO.input(Echo1) == GPIO.LOW:
        sig_off1 = time.time()
    while GPIO.input(Echo1) == GPIO.HIGH:
        sig_on1 = time.time()

    while GPIO.input(Echo2) == GPIO.LOW:
        sig_off2 = time.time()
    while GPIO.input(Echo2) == GPIO.HIGH:
        sig_on2 = time.time()

    # 距離を計算
    duration1 = sig_on1 - sig_off1
    distance1 = duration1 * 34000 / 2
    duration2 = sig_on2 - sig_off2
    distance2 = duration2 * 34000 / 2

    return distance1, distance2

cm = [0,0]

# 連続して値を読み取る
while True:
    try:
        cm = read_distance()
        if 2 < cm[0] < 400:
            print("Sensor 1 distance =", int(cm[0]), "cm")
        if 2 < cm[1] < 400:
            print("Sensor 2 distance =", int(cm[1]), "cm")
        time.sleep(1)

    except KeyboardInterrupt:  # Ctrl+Cキーでプログラムを終了
        GPIO.cleanup()
        sys.exit()
