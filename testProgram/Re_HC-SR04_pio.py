import pigpio
import time
import sys

def time_bet(t1,t2):
    return t2-t1

def dis_bet(x1,x2):
    return x2-x1

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
    
    while pi.read(echo) == 0:
        start_time = time.time()
        
    while pi.read(echo) == 1:
        stop_time = time.time()
        if stop_time-start_time>=(400*2)/34300:
            stop_time=start_time+0.03 #距離が400cm以上であったら時間差を0.03秒にして4m超えを伝える
    
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # 音速（34300 cm/s）で計算
    return distance

t_bef = time.time()
x_bef = get_distance(Trigger, Echo)

try:
    while True:
        distance1 = get_distance(Trigger, Echo)

        t_aft = time.time() #現在の時間を計る
        x_aft=distance1 #測った距離を今測った距離に代入

        t_bet = time_bet(t_bef,t_aft) #時間差(秒単位)と変位(cm単位)を計算
        x_bet = dis_bet(x_bef,x_aft)

        v_x = x_bet/t_bet #速度を計算(cm/s単位)

        t_bef = t_aft #今の時間を変化前の時間として保存
        x_bef = x_aft #今測った距離を変化前の距離に代入

        if 2 < distance1 < 400:
            print("Sensor 1 distance =", int(distance1), "cm")
        else:
        # 距離が4m以上ならループを終了
            print("距離が4mを超えています")
            break

        if 2 < distance1 < 400 :
            print("変位(",x_bet,")")
            print("速度(",v_x,")")
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting program")
    pi.stop()
    sys.exit()
