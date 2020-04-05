import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class Cart:

  def __init__(self, x=0.0, y=0.0, v=0.0, L=0.0, R=0.0):
    self.x = x
    self.y = y
    self.yaw = 120 # vehicle angle
    self.steering_yaw = 0 # steering wheel angle
    self.v = v # velocity
    self.L = 15
    self.R = 20
    # full Ackermann model parameters
#         self.rear_x = self.x - ((self.L / 2) * math.cos(self.yaw))
#         self.rear_y = self.y - ((self.L / 2) * math.sin(self.yaw))

  def update(self,u):
    # u[0] - velocity, u[1] - steering angle

#     Ackermann -> Bicycle model
    B = math.atan2(math.tan(self.steering_yaw))
    self.v = u[0]
    self.steering_yaw = u[1]
    self.x = self.x + self.v * math.cos(self.yaw + B)
    self.y = self.y + self.v * math.sin(self.yaw + B)
    self.yaw = self.yaw + (self.v * math.cos(B) * math.tan(B))/self.L

# steering wheel angle is limited/saturated
    self.steering_yaw = min(.75,max(-.75,self.steering_yaw))
    self.v = max(self.v,0)

