from multiprocessing import Process
import time

def write_mail(number):
    print(f"Write(No.{number}):Hello")
    time.sleep(0.03)
    print(f"Write({number}):OK?")
    time.sleep(0.03)
    print(f"Write({number}):Good Bye")
    time.sleep(0.03)

def send_mail(number):
    print(f"send({number})")
    time.sleep(5)

def check_response(number):
    hoge=0
    # 無駄な計算
    for i in range(1, 100000000):
        hoge += i/3 + i/5 + i/7 + i/9 + i/11
    print(f"ok({number})")

def task(process_num):
    write_mail(process_num)
    send_mail(process_num)
    check_response(process_num)

if __name__ == '__main__':
    start_time=time.time()
    t1 = Process(target=task, args=(1,))
    t2 = Process(target=task, args=(2,))
    t3 = Process(target=task, args=(3,))
    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()
    print(f"time:{time.time()-start_time}s")

