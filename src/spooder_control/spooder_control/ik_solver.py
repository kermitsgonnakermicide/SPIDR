import math

class IKSolver:
    def __init__(self, coxa_len, femur_len, tibia_len):
        self.coxa = coxa_len
        self.femur = femur_len
        self.tibia = tibia_len

    def solve(self, x, y, z):
        """
        Solves IK for a single leg.
        x, y, z are coordinates of the foot tip relative to the coxa frame.
        """
        
        theta1 = math.atan2(y, x)

        horizontal_dist = math.sqrt(x**2 + y**2) - self.coxa
        
        
        L = math.sqrt(horizontal_dist**2 + z**2)

        
        try:
            alpha = math.acos((self.femur**2 + L**2 - self.tibia**2) / (2 * self.femur * L))
            gamma = math.acos((self.femur**2 + self.tibia**2 - L**2) / (2 * self.femur * self.tibia))
        except ValueError:
            return 0.0, 0.0, 0.0

        beta = math.atan2(z, horizontal_dist)
        theta2 = beta + alpha 
        theta3 = gamma - math.pi 

        return theta1, theta2, theta3
