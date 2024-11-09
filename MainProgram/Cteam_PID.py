class Cteam_PID:
    def init(self):
        self.K_I:float = 1.0
        self.K_P:float = 1.0
        self.K_D:float = 1.0
        
        #  Enter 1 to enable each gain. Enter 0 to enable each gain.
        self.enableKp:int = 1
        self.enableKi:int = 1
        self.enableKd:int = 1
        
        self.ans:float = 0.0
        
    def setting(self, _Ki:float, _Kp:float, _Kd:float, _enableKi:bool, _enableKp:bool, _enableKd:bool):
        self.K_I = _Ki
        self.K_P = _Kp
        self.K_D = _Kd
        
        self.enableKi = _enableKi
        self.enableKp = _enableKp
        self.enableKd = _enableKd        
                
    def PID(self, _integralValue:float, _rawValue:float, _diffrerentialValue:float):
        self.ans =  self.enableKi * self.K_I * _integralValue + self.enableKp * self.K_P * _rawValue + self.enableKd * self.K_D * _diffrerentialValue
        
        