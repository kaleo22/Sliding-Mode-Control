class LongitudinalModel:
    def __init__(self):
        self.m = 50 #kg
        self.v_desired = 0.3 #m/s
        self.v = 0.0 #m/s
        self.kappa = 0.2339 #N*s/m
        self.friction = 0.1 # friction coefficient
        self.g = 9.81 #m/s^2

    def update(self, u_vel, dt):
        self.v += 1/self.m * (u_vel - self.kappa *self.v**2 - self.friction * self.m * self.g) * dt
        return self.v
    