
class PidController():

    def __init__(self):
        self.Kp = 0.03
        self.Ti = 0.05
        self.Td = 0.0
        self.err_prev=0
        self.err_int=0
        
    def GetControl(self, r, y, dt):
        err = r-y
        derr = (err-self.err_prev)/dt
        self.err_int = self.err_int + dt*err
        self.err_prev = err
        return self.Kp*(err + 1/self.Ti*self.err_int + self.Td*derr)

