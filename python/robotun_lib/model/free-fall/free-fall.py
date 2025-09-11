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
    
    def get_delta2(self, ):


