import pigpio
import time
import sys

# ピン番号の定義
Trigger = 7
Echo = 1

# pigpioを初期化
pi = pigpio.pi()

if not pi.connected:
    print("pigpioデーモンに接続できませんでした")
    sys.exit()

# トリガーピンのセットアップ（出力モード）
pi.set_mode(Trigger, pigpio.OUTPUT)

# エコーピンのセットアップ（入力モード）
pi.set_mode(Echo, pigpio.INPUT)

# 値の読み取り
def get_distance(trigger, echo):
    pi.gpio_trigger(trigger, 10, 1)  # 10マイクロ秒のパルスを送信
    start_time = time.time()
    stop_time = time.time()
    timeout=start_time+0.012
    
    while pi.read(echo) == 0:
        start_time = time.time()
        if start_time>timeout:
            print("Failed not found signal")
            return None
        
    while pi.read(echo) == 1:
        stop_time = time.time()
        if time.time()-start_time>=(200*2)/34300:
            stop_time=start_time+0.03 #距離が400cm以上であったら時間差を0.03秒にして4m超えを伝える
            break
    
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # 音速（34300 cm/s）で計算
    return distance

try:
    while True:
        distance1 = get_distance(Trigger, Echo)

        if distance1 is None:
            print("Failed")
            continue

        if 2 < distance1 < 400:
            print("Sensor 1 distance =", int(distance1), "cm")
        
        # 距離が4m以上ならループを終了
        if distance1 >= 400:
            print("Failed Over")
            break

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Exiting program")
    pi.stop()
    sys.exit()
