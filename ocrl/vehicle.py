import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def add_tuples(tuple1,tuple2):
    return tuple(map(lambda x, y: x + y, tuple1, tuple2))

# home-made function for reasonable graphics in matplotlib/python
def center_transform_matrix(angle, translation):
    x,y = translation
    #theta = np.radians(angle)
    theta = angle
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s, 0], [s, c, 0],[0, 0, 1]])
    T = np.array([[1, 0, x],[0, 1, y],[0, 0, 1]])
    T_reverse = np.array([[1, 0, -x],[0, 1, -y],[0, 0, 1]])
    T_intermediate = np.matmul(T,R)
    T_full = np.matmul(T_intermediate, T_reverse)
    return T_full

# home-made function for reasonable graphics in matplotlib/python    
def offsetRectangle(pos, offset, width = 0.0, height = 0.0, angle = 0.0, linewidth = 1, 
                    offset_angle = 0.0, edgecolor = 'k', facecolor = 'k'):
    x, y = pos
    x_,y_ = offset
    point1 = add_tuples((x+x_,y+y_),(width/2,height/2))
    point2 = add_tuples((x+x_,y+y_),(-width/2,height/2))
    point3 = add_tuples((x+x_,y+y_),(-width/2,-height/2))
    point4 = add_tuples((x+x_,y+y_),(width/2,-height/2))
    # rotation matrix
    T_full = center_transform_matrix(angle,(x,y))
    # transforming the points
    point1_center = np.matmul(T_full, np.asarray( (point1[0],point1[1],1) ))[0:-1]
    point2_center = np.matmul(T_full, np.asarray( (point2[0],point2[1],1) ))[0:-1]
    point3_center = np.matmul(T_full, np.asarray( (point3[0],point3[1],1) ))[0:-1]
    point4_center = np.matmul(T_full, np.asarray( (point4[0],point4[1],1) ))[0:-1]
    center_pos = (point1_center + point2_center + point3_center + point4_center) / 4
    # rotate around the center point
    point1 = np.matmul(center_transform_matrix(offset_angle, (center_pos[0],center_pos[1])),
                        np.asarray( (point1_center[0],point1_center[1], 1) ))
    point2 = np.matmul(center_transform_matrix(offset_angle, (center_pos[0],center_pos[1])),
                        np.asarray( (point2_center[0],point2_center[1], 1) ))
    point3 = np.matmul(center_transform_matrix(offset_angle, (center_pos[0],center_pos[1])),
                np.asarray( (point3_center[0],point3_center[1], 1) ))
    point4 = np.matmul(center_transform_matrix(offset_angle, (center_pos[0],center_pos[1])),
                        np.asarray( (point4_center[0],point4_center[1], 1) ))
    #plotting the lines in between the points
    plt.plot([point1[0], point2[0]], [point1[1], point2[1]], color='k', linestyle='-', linewidth=2)
    plt.plot([point2[0], point3[0]], [point2[1], point3[1]], color='k', linestyle='-', linewidth=2)
    plt.plot([point3[0], point4[0]], [point3[1], point4[1]], color='k', linestyle='-', linewidth=2)
    plt.plot([point4[0], point1[0]], [point4[1], point1[1]], color='k', linestyle='-', linewidth=2) 
    return 0
 
# home-made function for reasonable graphics in matplotlib/python
def centerRectangle(pos, width = 0.0, height = 0.0, angle = 0.0, linewidth = 1, edgecolor = 'k', facecolor = 'k'):
    x, y = pos
    point1 = add_tuples((x,y),(width/2,height/2))
    point2 = add_tuples((x,y),(-width/2,height/2))
    point3 = add_tuples((x,y),(-width/2,-height/2))
    point4 = add_tuples((x,y),(width/2,-height/2))
    # rotation matrix
    T_full = center_transform_matrix(angle,(x,y))
    # transforming the points
    point1 = np.matmul(T_full, np.asarray( (point1[0],point1[1],1) ))[0:-1]
    point2 = np.matmul(T_full, np.asarray( (point2[0],point2[1],1) ))[0:-1]
    point3 = np.matmul(T_full, np.asarray( (point3[0],point3[1],1) ))[0:-1]
    point4 = np.matmul(T_full, np.asarray( (point4[0],point4[1],1) ))[0:-1]
    #plotting the lines in between the points
    plt.plot([point1[0], point2[0]], [point1[1], point2[1]], color='k', linestyle='-', linewidth=2)
    plt.plot([point2[0], point3[0]], [point2[1], point3[1]], color='k', linestyle='-', linewidth=2)
    plt.plot([point3[0], point4[0]], [point3[1], point4[1]], color='k', linestyle='-', linewidth=2)
    plt.plot([point4[0], point1[0]], [point4[1], point1[1]], color='k', linestyle='-', linewidth=2) 
    return 0


class Cart:

    def __init__(self, x=0.0, y=0.0, v=0.0, L=0.0, R=0.0):
        self.x = x 
        self.y = y 
        self.yaw = 120 # vehicle angle
        self.steering_yaw = 0 # steering wheel angle
        self.v = v # velocity
        self.L = 15# length
        self.R = 20#
        self.W = 10# width
        self.steer_map_const = 1/25
        # full Ackermann model parameters
#         self.rear_x = self.x - ((self.L / 2) * math.cos(self.yaw))
#         self.rear_y = self.y - ((self.L / 2) * math.sin(self.yaw))
        
    def update(self,u):
        #     Ackermann -> Tricycle Vehicle Model
        self.x = self.x + self.v * math.cos(self.steering_yaw) * math.cos(self.yaw)
        self.y = self.y + self.v * math.cos(self.steering_yaw) * math.sin(self.yaw)
        self.v = self.v + u[0]
        self.yaw = self.yaw + self.v/self.L * math.sin(self.steering_yaw)
        self.steering_yaw = self.steering_yaw + u[1]
        # steering wheel angle is limited/saturated
        self.steering_yaw = min(.75,max(-.75,self.steering_yaw))
        self.v = max(self.v,0)
    
    def draw(self,axis):
        body_rect = centerRectangle((self.x, self.y), width = self.W, height = self.L, 
                                    angle = self.yaw - math.radians(90), linewidth=1,edgecolor='k',facecolor='none')
        front_wheel_ang = self.yaw + self.steering_yaw - math.radians(90)
        H = math.sqrt(math.pow(self.L/2,2) + math.pow(self.W/2,2)) # hypotenuse of the vehicle width and length
        # wheel positions
        # front wheels
        off_angle = self.steering_yaw
        main_angle = self.yaw - math.radians(90)
        offsetRectangle((self.x,self.y),(self.W/2.5,self.L/2.8), self.W/10, self.L/6, 
                                    angle = main_angle, edgecolor='k',facecolor='k',offset_angle = off_angle)
        offsetRectangle((self.x,self.y),(-self.W/2.5,self.L/2.8), self.W/10, self.L/6, 
                                    angle = main_angle, edgecolor='k',facecolor='k',offset_angle = off_angle)
        # back wheels
        offsetRectangle((self.x,self.y),(-self.W/2.5,-self.L/2.8), self.W/10, self.L/6, 
                                    angle = main_angle, edgecolor='k',facecolor='k')
        offsetRectangle((self.x,self.y),(self.W/2.5,-self.L/2.8), self.W/10, self.L/6, 
                                    angle = main_angle, edgecolor='k',facecolor='k')