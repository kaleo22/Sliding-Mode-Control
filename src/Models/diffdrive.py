import numpy as np

class DiffDrive:
    def __init__(self, x=0.0, y=0.0, omega_l=0.0, omega_r=0.0):
        ''' Initializes the robot's position and orientation.'''
        self.L = 0.5 # distance between the wheels
        self.x = x
        self.y = y
        self.omega_l = omega_l
        self.omega_r = omega_r
        self.phi = 0.0 
        self.state = np.array([[self.x], [self.y], [self.phi]])
        self.r = 0.2 # radius of the wheels
        self.v = 0.3

    def update(self, omega_l, omega_r, dt):
        '''
        Updates the robot pose from the wheel angular rates.

        Args:
            omega_l: Angular rate of the left wheel.
            omega_r: Angular rate of the right wheel.
            dt: Time step used for the integration.

        Returns:
            tuple[float, float, float]: The updated x position, y position,
            and orientation phi.
        '''
        self.v = self.r / 2 * (omega_r + omega_l)
        omega = self.r / self.L * (omega_r - omega_l)

        self.x += self.v * np.cos(self.phi) * dt
        self.y += self.v * np.sin(self.phi) * dt
        self.phi += omega * dt

        self.phi = (self.phi + np.pi) % (2 * np.pi) - np.pi

        self.state = np.array([[self.x], [self.y], [self.phi]])

        return self.x, self.y, self.phi

    def drive(self, waypoint):
        '''
        Computes wheel commands that turn the robot toward a waypoint.

        Args:
            waypoint: Target position as a 2D point-like sequence [x, y].

        Returns:
            tuple[float, float]: The commanded angular rates for the left and
            right wheels.
        '''
        desired_phi = np.arctan2(waypoint[1] - self.y, waypoint[0] - self.x)
        phi_error = desired_phi - self.phi
        phi_error = (phi_error + np.pi) % (2 * np.pi) - np.pi

        k_phi = 2.0
        omega = k_phi * phi_error

        omega_l = (self.v - omega * self.L / 2) / self.r
        omega_r = (self.v + omega * self.L / 2) / self.r

        return omega_l, omega_r


        
         
    
  