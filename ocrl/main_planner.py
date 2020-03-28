"""
Mobile robot motion planning sample with Dynamic Window Approach
author: Atsushi Sakai (@Atsushi_twi)
"""

import math, time, sys
import numpy as np
import vehicle as vehicle
import listener as listener
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from test.sample_doctest import x_is_one
from html5lib.utils import _x

def plot_arrow(x, y, yaw, length=0.5, width=0.1, clr = 'r'):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width, facecolor = clr)
    plt.plot(x, y)

"""
TODO:
- try to adapt the speed accordingly PID control (not needed for the demo)
"""
old_nearest_point_index = None
k = 0.1  # look forward gain
Lfc = 20.0  # look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle

def handle_close(evt):
    print("figure was closed")
        
def pure_pursuit_control(av, cx, cy, pind):

    ind = calc_target_index(av, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - av.y, tx - av.x) - av.yaw
    #print("alpha:{}".format(alpha))
    Lf = k * av.v + Lfc # adaptive based on velocity

    delta = math.atan2(av.L * math.sin(alpha) / Lf, 1.0)
    #print("delta:{}".format(delta))
    
    return delta, ind

def calc_target_index(av, cx, cy):
    last_waypointx = cx[len(cx) - 1]
    last_waypointy = cy[len(cy) - 1]
    Lf = k * av.v + Lfc
    
    global old_nearest_point_index

    if (calc_distance(av,last_waypointx,last_waypointy) >= Lf):
        if old_nearest_point_index is None:
            # search nearest point index (should happen at first call of this function
            dx = [av.x - icx for icx in cx]
            dy = [av.y - icy for icy in cy]
            d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
            ind = d.index(min(d))
            old_nearest_point_index = ind
        else:
            # what is done to find the most likely index if we have a previous index
            ind = old_nearest_point_index
            distance_this_index = calc_distance(av, cx[ind], cy[ind])
            while True:
                if ((ind + 1) < len(cx)):
                    # iterate
                    ind = ind + 1
                    distance_next_index = calc_distance(av, cx[ind], cy[ind])
                    if distance_this_index < distance_next_index:
                        break
                    distance_this_index = distance_next_index
                else:
                    break
            old_nearest_point_index = ind

    L = 0.0
    
    # search look ahead target point index
    # target last point if we are at the end of the path
    if (calc_distance(av,last_waypointx,last_waypointy) >= Lf):
        while Lf > L and (ind + 1) < len(cx):
            dx = cx[ind] - av.x
            dy = cy[ind] - av.y
            L = math.sqrt(dx ** 2 + dy ** 2)
            ind += 1
    else:
        ind = len(cx) - 1
        
    return ind

def calc_distance(av, point_x, point_y):
    dx = av.x - point_x
    dy = av.y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)


def main():
    paused = False
    #     Initialization
    # plotting variables
    simRunning = True
    time_till_end = 0
    fig = plt.figure()
    full_screen = False
    fig.canvas.mpl_connect('close_event', handle_close) # FIX THIS LATER
    ax = fig.gca()
    # vehicle initialization
    av = vehicle.Cart()
    av.v = 1
    # path trajectory
    pathy = np.array(np.arange(start=-10, stop=200, step=1)) # vertical line
    #pathx = np.array(np.zeros(pathy.shape)) # straight line path
    pathx = [math.sin(ix / 90.0) * ix / 2.0 for ix in pathy] # curved line path
    arduino_info = listener.Listener()
    yaw_estimate = 20 # variable realtime test for steering feedback with arduino/IMU
    desired_yaw = 90 # desired vehicle yaw
    # Pure Pursuit
    target_ind = calc_target_index(av, pathx, pathy)
    lastIndex = len(pathx) - 1

    # MAIN-SIMULATION LOOP
    while ((simRunning == True) or (time_till_end > 0)):
        time_till_end -= 1
        #     CALCULATE DESIRED YAW: PURE PURSUIT
        if (calc_distance(av,pathx[-1],pathy[-1]) > av.L):
            deltayaw, target_ind = pure_pursuit_control(av, pathx, pathy, target_ind)
            desired_yaw = av.yaw + deltayaw
            # steering delta that is affected by quantization
            steerdelta = np.sign(desired_yaw - (av.yaw + av.steering_yaw)) * av.steer_map_const
            av.v = 1 * math.cos(deltayaw) # adaptive speed
        else:
            # stop vehicle end the program 100 steps after vehicle reaches target
            if av.v!=0:
                time_till_end = 100
            av.v -= np.sign(av.v)
            av.steering_yaw = 0
            deltayaw = 0
            full_screen = True
            simRunning = False
        
        #     REALTIME PHYSICAL INTEGRATION
        # get values from the arduino, if possible
        new_yaw_estimate = arduino_info.getIntMessage()
        if (new_yaw_estimate != None):
            yaw_estimate = new_yaw_estimate
        # SEND THE ARDUINO THE DESIRED ANGLE BASED ON PURE PURSUIT
        arduino_info.sendMessage(str(desired_yaw))
        
        plt.cla()
        #    GRAPHICS
        # display path
        plt.plot(pathx, pathy, 'yo--', markersize=4, alpha = .25)
        plt.plot(pathx[target_ind],pathy[target_ind],'ro--', markersize=7)
        print("path x:{} pathy:{} deltayaw:{} steer delta:{} steer:{}".format(pathx[target_ind],pathy[target_ind],
                                                                              deltayaw,steerdelta,av.steering_yaw))

        # display the various yaw vectors
        plot_arrow(av.x, av.y, av.steering_yaw + av.yaw, length=10, width=2, clr = 'r')
        plot_arrow(av.x, av.y, desired_yaw, length=10, width=2, clr = 'b')
        
        # display the vehicle
        av.draw(ax)
        
        # display the safety margin
        plt.plot(av.x, av.y, 'bo--', markersize=(av.L * 4.85 * av.v) ,alpha =.25)
        
        # maintain a fixed axis size
        if (full_screen == False):
            ax.set(xlim=(av.x-50, av.x+50), ylim=(av.y-50, av.y+50)) 
        else:
            plt.grid()
        plt.pause(0.001)
        
        # UPDATE VEHICLE POSITION
        av.update(np.array([0,steerdelta]))
        
        if (paused == False):
            paused = True
            time.sleep(5)
    
    # end connection with arduino
    if (arduino_info.port != None):
        arduino_info.close() # end the connection if it has been made
        
if __name__ == '__main__':
    main()