# -*- coding: utf-8 -*-
"""
Created on Fri Apr  3 22:39:49 2020

@author: Ankur Bodhe
"""

import numpy as np
from scipy import interpolate
import math

import matplotlib.pyplot as plt


#------------------------------ README ---------------------------------------#

# The objective of the code is to generate Spline trajectroies for a given set
# of waypoints and waypoint poses 
# 
# Input Parameters
# 1) Waypoints (2D numpy array <(x_val, y_val))
# 2) angles (in radians)
# 3) pose_length (a hyperparameter for tuning the trajectory based on the pose angle)
# 4) degree (the degree of the spline function (range : 1 to 5))
# 5) sf (smoothness factor of the spline trajectory (enter a +ve integer))
# 6) graph_enable (1 - plot graph ; 0 - do not plot graph)
#
# Outputs
#  array of 2 Arrays : out[0] - array of the x values
#                      out[1] - array of the y values
#
# function call : bspline (waypoints, angles, pose_length, degree, sf, graph_enable)

#-----------------------------------------------------------------------------#



# ----------------- defining the function for b spline ----------------#

def bspline (waypoints, angles, pose_length, degree, sf, graph_enable):
    
    #---------- extracting the x and y values into different arrays----#
    
    x = waypoints[:,0]
    y = waypoints[:,1]
    
    x_t = []
    y_t = []
    
    #---------- generating the extra waypoints for pose ---------------#
    
    for i in range(len(x)):
        x_t.append(x[i])
        x_t.append(x[i]+pose_length*math.cos(angles[i]))
        y_t.append(y[i])
        y_t.append(y[i]+pose_length*math.sin(angles[i]))
    
    #----------- generating spline trajectory -------------------------#
    
    tck,u = interpolate.splprep([x_t,y_t],k=5,s=0)
    u=np.linspace(0,1,num=100,endpoint=True)
    out = interpolate.splev(u,tck)
    
    #----------- plot graph if graph_enable = 1 -----------------------#
    
    if (graph_enable == 1):
        plt.figure()
        plt.plot(x_t, y_t, 'ro', out[0], out[1], 'b')
        plt.plot(waypoints[:,0],waypoints[:,1],'yo',markersize = 7)
        plt.legend(['Appended Points', 'Interpolated B-spline',
 'Waypoints'],loc='best')
        plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
        plt.title('B-Spline interpolation')
        plt.show()
    
    return out
    
def main():
    
    #---------------- waypoint values for testing purposes ONLY --------------#
    waypoints = np.array( [(3 , 1), (2.5, 4), (0, 5), (-2, 0),
                (-3, 0), (-2.5, -4), (0, -1), (2.5, -4), (3, -1)])
    
    #------------------- angle values for testing purposes ONLY --------------#
    angles = np.array([math.pi/4, math.pi/2, math.pi/4, math.pi/2, math.pi/4, 
                       math.pi/4, math.pi/2, math.pi/4, math.pi/2, math.pi/4])
    
    #-------------------------------------------------------------------------#
    
    pose_length = 0.5 #pose_length values for testing purposes ONLY
    
    degree = 3        #degree values for testing purposes ONLY
    
    sf = 0            #smoothness factor values for testing purposes ONLY
    
    spline_vals = bspline(waypoints, angles, pose_length, degree, sf, 1)
    
    print (spline_vals)
    
if __name__ == '__main__':
    main()
