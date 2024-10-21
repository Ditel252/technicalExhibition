import math

class Simpson:
    def init(self):
        self.sumOfEven:float = 0.0
        self.sumOfOdd:float = 0.0
        self.sumCount:int = 0
        
        self.ans:float = 0.0

    def simpson(self, _value:float, _dt:float):
        if(self.sumCount == 1): # 奇数の時の式
            self.sumOfOdd += _value
            
            self.sumCount = 0
            return False
        else:   # 偶数の時の式
            self.ans = (_dt / 3.0) * (0.0 + _value + 4.0 * self.sumOfOdd + 2.0 * self.sumOfEven)
            
            self.sumOfEven += _value
            self.sumCount += 1
            
            return True
        
        
simpson = Simpson()

def f(x):
    return math.sqrt(x)

NUM = 1000

simpson.init()

for i in range(NUM + 1):
    simpson.simpson(float(f(i / 100.0)), 1 / 100.0)
    print("x:{:f} y:{:f} sumOdd:{:f} sumEven:{:f} ans:{:f}".format(i, f(i), simpson.sumOfOdd, simpson.sumOfEven, simpson.ans))
    
print(simpson.ans)
    