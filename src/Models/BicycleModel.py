import numpy as np

class Kinematic():
    def __init__(self, l_v, l_h):
        self.phi = 0.0  # orientation of the robot
        self.x = 0.0
        self.y = 0.0
        self.l_v = l_v
        self.l_h = l_h
        self.beta = 0.0
        self.delta_v = 0.0
        self.velocity = 0.3
    
    def update(self, delta_v, dt):
        self.beta = np.arctan((self.l_h / (self.l_v + self.l_h)) * np.tan(delta_v))
        self.x += self.velocity * np.cos(self.phi + self.beta) * dt
        self.y += self.velocity * np.sin(self.phi + self.beta) * dt
        self.phi += (self.velocity / (self.l_v + self.l_h)) * np.tan(delta_v) * np.cos(self.beta) * dt

        return self.x, self.y, self.phi

    def drive(self, waypoint):
        desired_phi = np.arctan2(waypoint[1] - self.y, waypoint[0] - self.x)
        phi_error = desired_phi - self.phi
        phi_error = (phi_error + np.pi) % (2 * np.pi) - np.pi

        k_phi = 1.0
        delta_v = k_phi * phi_error
        self.delta_v = delta_v

        return delta_v 
