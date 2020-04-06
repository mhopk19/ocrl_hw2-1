#!/usr/bin/env python

from common import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from angles import *
import matplotlib.pyplot as plt
import time
import pure_pursuit as pursuit
import bsplines

# global variables for plotting
extra_margins = (5,5)
fig = plt.figure()
ax = fig.gca()

# trajectory data
trajectory_data = [[],[],[]] # x-y-quaternion
start_time = 0
end_time = 0
yaw_differences = [] # difference between the vehicle yaw and the desired yaw at each waypoint

# pure pursuit variables
k = 0.1
Lfc = .33
Kp = 1.0
dt = 0.1 # ROS update rate
L = .335 # wheel base
old_nearest_point_index = None

# basis spline variables
planned_traj = None

advanced_method = 1 # whether or not we are using the advanced method

import tf

def waypointCallback(msg):
  global waypoints
  for i in range(len(msg.poses)):
    waypoints[i, 0] = msg.poses[i].position.x
    waypoints[i, 1] = msg.poses[i].position.y
    waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]

def vehicleStateCallback(msg):
  global rear_axle_center, rear_axle_theta, rear_axle_velocity
  rear_axle_center.position.x = msg.pose.pose.position.x
  rear_axle_center.position.y = msg.pose.pose.position.y
  rear_axle_center.orientation = msg.pose.pose.orientation
  rear_axle_theta = euler_from_quaternion(
    [rear_axle_center.orientation.x, rear_axle_center.orientation.y, rear_axle_center.orientation.z,
     rear_axle_center.orientation.w])[2]
  rear_axle_velocity.linear = msg.twist.twist.linear
  rear_axle_velocity.angular = msg.twist.twist.angular

def advancedPursuit(waypoint,traj,reverse = False):
  global rear_axle_center, rear_axle_theta, rear_axle_velocity, cmd_pub, waypoints, trajectory_data, old_nearest_point_index
  rospy.wait_for_message("/ackermann_vehicle/ground_truth/state",Odometry,5)
  init_velocity = np.linalg.norm([rear_axle_velocity.linear.x,rear_axle_velocity.linear.y],2)
  print("init velocity:{}".format(init_velocity))
  vstate = pursuit.State(x=rear_axle_center.position.x,y=rear_axle_center.position.y,yaw = rear_axle_theta, v=init_velocity)

  cmd = AckermannDriveStamped()
  cmd.header.stamp = rospy.Time.now()
  prev_time = cmd.header.stamp.to_sec()
  cmd.header.frame_id = "base_link"
  cmd.drive.speed = rear_axle_velocity.linear.x
  cmd.drive.acceleration = max_acc

  print("trajectory:{}".format(traj))
  old_nearest_point_index = None
  target_ind = pursuit.calc_target_index(vstate,traj[0],traj[1])
  dx,dy = waypoint[0] - rear_axle_center.position.x, waypoint[1] - rear_axle_center.position.y
  target_distance = math.sqrt(dx*dx + dy*dy)

  while (target_distance > waypoint_tol):
    dx,dy = (waypoint[0] - rear_axle_center.position.x, waypoint[1] - rear_axle_center.position.y)
    target_distance = math.sqrt(dx*dx + dy*dy)
    cmd.header.frame_id = "base_link"
    # time calculation
    cmd.header.stamp = rospy.Time.now()
    deltat = cmd.header.stamp.to_sec() - prev_time
    prev_time = cmd.header.stamp.to_sec()

    # pure_pursuit
    if (target_ind == len(traj[0]) - 1):
      tdx = dx
      tdy = dy
    else:
      tdx = traj[0][target_ind] - rear_axle_center.position.x
      tdy = traj[1][target_ind] - rear_axle_center.position.y
    lookahead_dist = np.sqrt(tdx * tdx + tdy * tdy)
    lookahead_theta = math.atan2(tdy, tdx)
    alpha = shortest_angular_distance(rear_axle_theta, lookahead_theta)

    # Reactive steering
    if alpha < 0:
      st_ang = max(-max_steering_angle, alpha)
    else:
      st_ang = min(max_steering_angle, alpha)

    prev_speed = np.linalg.norm([rear_axle_velocity.linear.z,rear_axle_velocity.linear.x,rear_axle_velocity.linear.y],2)
    # Reactive Speed
    Kp = .1
    target_speed = min(1 + 2/(abs(st_ang) + 1) ,target_distance)
    if (reverse == True):
      ai = np.sign(-target_speed - prev_speed) * max(-3,min(3,abs(Kp * (-target_speed - prev_speed))))
    else:
      ai = np.sign(target_speed - prev_speed) * max(-3,min(3,abs(Kp * (target_speed - prev_speed))))
    #new_speed = min(prev_speed + ai,lookahead_dist+.1)
    new_speed = prev_speed + ai
    #cmd.drive.speed = min(10,max(new_speed,-10))

    if (target_distance < .67 * math.pi):
      cmd.drive.speed = .5
      Lfc = .05
      k = 0
    else:
      Lfc = .33
      k = .2
      cmd.drive.speed = min(10,max(new_speed,-5))

    # steering
    di, target_ind = pursuit.pure_pursuit_control(vstate,traj[0],traj[1],target_ind)
    # set commands
    cmd.drive.steering_angle = st_ang
    #cmd.drive.speed = 1 # prev_speed + ai
    cmd_pub.publish(cmd)
    rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)

    plt.cla()
    plot2dtraj()
    # plotting the dead reckoning state
    plt.plot(vstate.x,vstate.y,'k',marker="p",markersize=10)
    # plotting the lookahead point
    plt.plot(traj[0][target_ind],traj[1][target_ind],'yo',markersize = 10)
    plt.pause(0.001)

    # then update state
    #vstate = pursuit.update(vstate,ai,di,dt = .1)
    vstate.x = rear_axle_center.position.x
    vstate.y = rear_axle_center.position.y
    vstate.v = new_speed
    print("ai:{} di:{} prev_speed:{} deltat:{}".format(ai,di,prev_speed,deltat))
    print("vstate:x:{} vstate:y:{} vstate:v:{}".format(vstate.x,vstate.y,vstate.v))

def pursuitToWaypoint(waypoint):
  print("waypoint:{}".format(waypoint))
  global rear_axle_center, rear_axle_theta, rear_axle_velocity, cmd_pub, waypoints, trajectory_data
  rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)
  dx = waypoint[0] - rear_axle_center.position.x
  dy = waypoint[1] - rear_axle_center.position.y
  target_distance = math.sqrt(dx*dx + dy*dy)

  cmd = AckermannDriveStamped()
  cmd.header.stamp = rospy.Time.now()
  cmd.header.frame_id = "base_link"
  cmd.drive.speed = rear_axle_velocity.linear.x
  cmd.drive.acceleration = max_acc
  while (target_distance > waypoint_tol):
    dx = waypoint[0] - rear_axle_center.position.x
    dy = waypoint[1] - rear_axle_center.position.y
    lookahead_dist = np.sqrt(dx * dx + dy * dy)
    lookahead_theta = math.atan2(dy, dx)
    alpha = shortest_angular_distance(rear_axle_theta, lookahead_theta)

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "base_link"
    # Publishing constant speed of 1m/s
    cmd.drive.speed = min(1,lookahead_dist+.1)

    # Reactive steering
    if alpha < 0:
      st_ang = max(-max_steering_angle, alpha)
    else:
      st_ang = min(max_steering_angle, alpha)

    cmd.drive.steering_angle = st_ang
    # adaptive speed component
    #cmd.drive.speed = 1 + math.cos(st_ang)

    target_distance = math.sqrt(dx * dx + dy * dy)

    cmd_pub.publish(cmd)
    rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)

    plt.cla()
    plot2dtraj()
    plt.pause(0.001)



def plot2dtraj():
    global rear_axle_center, rear_axle_theta, rear_axle_velocity, cmd_pub, waypoints, trajectory_data, planned_traj
    # plotting the planned trajectory
    if (planned_traj != None):
      plt.plot(planned_traj[0],planned_traj[1],'r',linewidth=1.5)
      plt.plot(planned_traj[0][0],planned_traj[1][0],'y',marker = "*", markersize = 15) # starting point
      plt.plot(planned_traj[0][-1],planned_traj[1][-1],'y',marker = "x",markersize = 15) # end point

    # once we get the new state we can plot the data
    plt.plot(trajectory_data[0],trajectory_data[1],'b-')
    plt.arrow(rear_axle_center.position.x,rear_axle_center.position.y,
math.cos(rear_axle_theta),math.sin(rear_axle_theta),'b-',
width = .05, head_width = .35)
    plt.plot(rear_axle_center.position.x,rear_axle_center.position.y,'go-',markersize = 12)
    trajectory_data[0].append(rear_axle_center.position.x)
    trajectory_data[1].append(rear_axle_center.position.y)
    trajectory_data[2].append(rear_axle_theta)
    # plot all the wayponts
    for i in range(num_waypoints):
      plt.text(waypoints[i,0],waypoints[i,1],"{}".format(i+1))
      plt.arrow(waypoints[i,0],waypoints[i,1],
math.cos(waypoints[i,2]),math.sin(waypoints[i,2]),
width = .05,head_width = .23)
      quaternion = waypoints[i,2]
      # plot an appropriate arrow
    ax.set(xlim=(x_lim[0] - extra_margins[0], x_lim[1] + extra_margins[0]), ylim=(y_lim[0] - extra_margins[1], y_lim[1] + extra_margins[1]) )
    plt.grid()
    plt.title("time:{}".format(time.clock() - start_time))


if __name__ == '__main__':
  rospy.init_node('pure_pursuit')
  cmd_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=10)
  waypoints = np.zeros((num_waypoints, 3))
  rospy.Subscriber("/ackermann_vehicle/waypoints",
                   PoseArray,
                   waypointCallback)
  rospy.wait_for_message("/ackermann_vehicle/waypoints", PoseArray, 5)
  rear_axle_center = Pose()
  rear_axle_velocity = Twist()
  rospy.Subscriber("/ackermann_vehicle/ground_truth/state",
                   Odometry, vehicleStateCallback)
  rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)


  # main control loop
  start_time = time.clock()

  if (advanced_method): # using the advanced robust method
    # once we have the waypoints we create a basis spline trajectory
    temp_waypoints = waypoints.copy()
    temp_waypoints = np.insert(waypoints, 0, [rear_axle_center.position.x,rear_axle_center.position.y,rear_axle_theta], axis = 0)
    temp_waypoints = temp_waypoints[0:1]
    print("temp waypoints:{}".format(temp_waypoints))
    # cycle through waypoints
    for i,w in enumerate(waypoints):
      print(i)
      temp_waypoints = np.append(temp_waypoints, [w], axis = 0)
      planned_traj = bsplines.bspline(np.array([(x,y) for x,y,yaw in temp_waypoints]),np.array([yaw for x,y,yaw in temp_waypoints]),L,3,0)
      old_nearest_point_index = None
      advancedPursuit(w,planned_traj)
      temp_waypoints = temp_waypoints[1:]
      print("temp waypoints:{}".format(temp_waypoints))
      yaw_differences.append(abs(rear_axle_theta - waypoints[i,2]))
  else:
    # using the naive original method
    for i,w in enumerate(waypoints):
      pursuitToWaypoint(w)
      # just reached the waypoint so we check the yaw difference
      yaw_differences.append(abs(rear_axle_theta - waypoints[i,2]))

  end_time = time.clock()
  plt.show()
  # display the results
  cost = [end_time - start_time, len(trajectory_data[0]), np.sum(yaw_differences)]
  print("total cost:total time{}|trajectory{}|poses{}".format(cost[0],cost[1],cost[2]))
