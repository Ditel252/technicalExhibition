class Cteam_PID:
    def init(self):
        self.K_I:float = 1.0
        self.K_P:float = 1.0
        self.K_D:float = 1.0
        
        self.enableKp:bool = True
        self.enableKi:bool = True
        self.enableKd:bool = True
        
        self.ans:float = 0.0
        
    def setting(self, _Ki:float, _Kp:float, _Kd:float, _enableKi:bool, _enableKp:bool, _enableKd:bool):
        self.K_I = _Ki
        self.K_P = _Kp
        self.K_D = _Kd
        
        self.enableKi = _enableKi
        self.enableKp = _enableKp
        self.enableKd = _enableKd        
                
    def PID(self, _integralValue:float, _rawValue:float, _diffrerentialValue:float):
        self.ans = self.K_I = self.K_I * _integralValue + self.K_P * _rawValue + self.K_D + _diffrerentialValue