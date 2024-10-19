# 必要なライブラリをインポート
import time
import sys
from gpiozero import PWMOutputDevice

# 定数定義
MAX_SIGNAL = 2000  # 最大パルス幅
MIN_SIGNAL = 1000  # 最小パルス幅
STOP_SIGNAL = 0.0  # モータ停止信号
NUM_ESC = 8  # モータ数

# ESCピン定義
esc_freq = 100
esc_Pins = [17, 27, 22, 10, 9, 11, 0, 5]  # ピン指定
esc_Devices = [PWMOutputDevice(pin, frequency=esc_freq) for pin in esc_Pins]  # PWMデバイスの作成
esc_Pulsewidth = [MIN_SIGNAL for _ in range(NUM_ESC)]  # ESCの初期パルス幅

# パルス幅を0~1の範囲に変換する関数
def pulsewidth_to_duty_cycle(pulsewidth):
    duty = pulsewidth * esc_freq / 1000000.0 # pulsewith / (1000000.0 / esc_freq)
    return duty

# ESCの初期設定
def esc_setup():
    print("Program begin...")
    print("This program will control the ESCs by adjusting pulse width.")
    print("Turn on power source, then wait 2 seconds.")
    time.sleep(2)

    for i in range(NUM_ESC):
        print(f"Writing maximum output to ESC {i + 1}")
        esc_Devices[i].value = pulsewidth_to_duty_cycle(MAX_SIGNAL)  # 最大出力を送信
        time.sleep(0.2)
        
    time.sleep(2)

    for i in range(NUM_ESC):
        print(f"Writing minimum output to ESC {i + 1}")
        esc_Devices[i].value = pulsewidth_to_duty_cycle(MIN_SIGNAL)  # 最小出力を送信
        time.sleep(0.2)
    
    time.sleep(2)

# モータをすべて停止
def motors_stop():
    for i in range(NUM_ESC):
        esc_Devices[i].value = STOP_SIGNAL  # PWMデバイスに0を送信
        esc_Pulsewidth[i] = MIN_SIGNAL  # パルス幅を初期値に戻す
        print(f"Stop the motor of ESC {i + 1}.")

# ESCに信号を送信
def set_esc_values(values):
    global esc_Pulsewidth
    # 入力の型変換
    try:
        values = [int(v) for v in values]
    except ValueError:
        print("Invalid input. Please enter integers only.")
        return

    # 入力が全て0, 0ならモータ停止
    if values == [0, 0]:
        motors_stop()
        return

    for i in range(NUM_ESC):
        # 現在のパルス幅に入力した変化量を加算
        esc_Pulsewidth[i] += values[i]

        # 新たなパルス幅が有効範囲内にあるか確認
        if MIN_SIGNAL <= esc_Pulsewidth[i] <= MAX_SIGNAL:
            print(f"Changing ESC {i + 1} to: {esc_Pulsewidth[i]}")
            esc_Devices[i].value = pulsewidth_to_duty_cycle(esc_Pulsewidth[i])
        else:
            # 範囲外の場合、元の値に戻す
            esc_Pulsewidth[i] -= values[i]
            print(f"Invalid value for ESC {i + 1}. Value must be between {MIN_SIGNAL} and {MAX_SIGNAL}.")

# メイン関数
def main():
    # ESC初期化
    esc_setup()

    try:
        while True:
            # 入力を取得
            Input_Value = input("Input the signal values separated by commas (e.g., '100, 200'): ").strip()
            Input_List = Input_Value.split(',')

            # ESCに信号を送信
            set_esc_values(Input_List)

    except KeyboardInterrupt:
        print("Exit the program.")

    finally:
        # ESCを停止し、リソースを開放
        motors_stop()
        for esc in esc_Devices:
            esc.off()
        print("Stop the ESCs.")

if __name__ == "__main__":
    main()

