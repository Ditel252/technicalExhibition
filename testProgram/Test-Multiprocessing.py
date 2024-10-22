import time
from multiprocessing import Value, Array, Process


def _process1(count, array):
    for i in range(5):
        time.sleep(0.5)
        # Valueオブジェクトの値を操作
        count.value += 1
        # Arrayオブジェクトの値を操作
        array[count.value - 1] = count.value
        print("process1:" + str(count.value))


def _process2(count, array):
    for i in range(5):
        time.sleep(0.7)
        count.value += 1
        array[count.value - 1] = count.value
        print("process2:" + str(count.value))


def _process3(count, array):
    for i in range(5):
        time.sleep(0.9)
        count.value += 1
        array[count.value - 1] = count.value
        print("process3:" + str(count.value))


if __name__ == '__main__':
    # 共有メモリの作成
    # Valueオブジェクトの生成
    count = Value('i', 0)
    # Arrayオブジェクトの生成
    array = Array('i', 15)
    print("count:" + str(type(count)))
    print("array:" + str(type(array)))
    print(array[:])

    process1 = Process(target=_process1, args=[count, array])
    process2 = Process(target=_process2, args=[count, array])
    process3 = Process(target=_process3, args=[count, array])

    process1.start()
    process2.start()
    process3.start()

    process1.join()
    process2.join()
    process3.join()

    print(array[:])
    print("process ended")

