import numpy as np

class DiffDrive:
    def __init__(self, x=0.0, y=0.0, omega_l=0.0, omega_r=0.0):
        ''' Initializes the robot's position and orientation.'''
        self.L = 0.5  # distance between wheels
        self.x = x
        self.y = y
        self.omega_l = omega_l
        self.omega_r = omega_r

    def update(self, omega_l, omega_r, r, phi, dt):
        ''' updates the robots position and orientation based on the angualar rates of the left and right wheels, the radius of the wheels, the current orientation of the robot, and the time step.'''
        self.A = np.ndarray([[r * np.cos(phi), r * np.cos(phi)], 
                              [r * np.sin(phi), r * np.sin(phi)], 
                              [r / self.L, -r / self.L]])
        self.omega = np.array([[omega_l], [omega_r]])
        self.state += (self.A @ self.omega) * dt

        return self.state
         
    
  