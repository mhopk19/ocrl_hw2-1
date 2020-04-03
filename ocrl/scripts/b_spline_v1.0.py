# -*- coding: utf-8 -*-
"""
Created on Fri Apr  3 09:33:12 2020

@author: Ankur Bodhe
"""

import numpy as np
from scipy import interpolate
import math

import matplotlib.pyplot as plt

x_t = [];
y_t = [];
#------------------ performing operations on waypoints --------------#
ctr =np.array( [(3 , 1), (2.5 , 2), (2.5, 4), (0, 5), (-2, 0),
                (-3, 0), (-2.5, -4), (0, -1), (2.5, -4), (3, -1)])
x=ctr[:,0]
y=ctr[:,1]

#--------------------------------------------------------------------#

#------------------- generating new points based on the angles --------------------------#
theta = math.pi/2 + math.pi/4
r = 1
for i in range(len(x)):
    x_t.append(x[i])
    x_t.append(x[i]+r*math.cos(theta))
    y_t.append(y[i])
    y_t.append(y[i]+r*math.sin(theta))

print (x_t)
tck,u = interpolate.splprep([x_t,y_t],k=5,s=0)
u=np.linspace(0,1,num=100,endpoint=True)
out = interpolate.splev(u,tck)




#---------------- for plotting purposes ------------------------------#
plt.figure()
plt.plot(x_t, y_t, 'ro', out[0], out[1], 'b')
plt.legend(['Points', 'Interpolated B-spline', 'True'],loc='best')
plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
plt.title('B-Spline interpolation')
plt.show()
#---------------------------------------------------------------------#