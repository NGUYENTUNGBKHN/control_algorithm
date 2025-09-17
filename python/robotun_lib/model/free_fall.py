import numpy as np

"""

"""
class free_fall():
    def __init__(self,
                M: float = 0.25,
                m: float = 0.3,
                r: float = 0.25,
                l: float = 1.0,
                g: float = 9.8,
                b1: float = 0.01,
                b2: float = 0.01,
        ) -> None:
        self.M = M
        self.m = m
        self.r = r
        self.l = l
        self.g = g
        self.b1 = b1
        self.b2 = b2
        self.I = 0.5 * M * r
    
    def get_delta2(self, state, u: float = 0.0):
        dth, th, dphi, phi = state 

        s = np.sin(th)
        c = np.cos(th)

        _dphi = (self.m * self.r * (self.l * dth ** 2 * s + self.b1 * dth * c - self.g * s * c) - self.b2 * dphi + u) / \
                (self.I + self.m * self.r ** 2 * s ** 2)
        _dth = (self.g * s - self.r * _dphi * c - self.b1 * dth) / self.l

        return [_dphi, _dth]
    
    def get_delta2_v(self, state, v: float = 0.0):
        dth, th = state 

        s = np.sin(th)
        c = np.cos(th)

        _dphi = (self.m * self.r * (self.l * dth ** 2 * s + self.b1 * dth * c - self.g * s * c) - self.b2 * v ) / \
                (self.I + self.m * self.r ** 2 * s ** 2)
        _dth = (self.g * s - self.r * _dphi * c - self.b1 * dth) / self.l

        return _dth


