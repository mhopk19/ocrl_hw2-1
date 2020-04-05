#!/usr/bin/env python

from common import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from angles import *
import matplotlib.pyplot as plt
import time

# global variables for plotting
extra_margins = (5,5)
fig = plt.figure()
ax = fig.gca()
# trajectory data
trajectory_data = [[],[],[]] # x-y-quaternion
start_time = 0
end_time = 0
yaw_differences = [] # difference between the vehicle yaw and the desired yaw at each waypoint

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
    cmd.drive.speed = 1

    # Reactive steering
    if alpha < 0:
      st_ang = max(-max_steering_angle, alpha)
    else:
      st_ang = min(max_steering_angle, alpha)

    cmd.drive.steering_angle = st_ang
    # adaptive speed component
    cmd.drive.speed = 1 + math.cos(st_ang)

    target_distance = math.sqrt(dx * dx + dy * dy)

    cmd_pub.publish(cmd)
    rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)
    plt.cla()
    # once we get the new state we can plot the data
    plt.plot(trajectory_data[0],trajectory_data[1],'b-')
    plt.arrow(rear_axle_center.position.x,rear_axle_center.position.y,
math.cos(rear_axle_theta),math.sin(rear_axle_theta),'b-',
width = .05, head_width = .35)
    plt.plot(rear_axle_center.position.x,rear_axle_center.position.y,'go-')
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
    plt.pause(0.001)

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

  # timing the  trajectory planning
  start_time = time.clock()
  for i,w in enumerate(waypoints):
    pursuitToWaypoint(w)
    # just reached the waypoint so we check the yaw difference
    yaw_differences.append(abs(rear_axle_theta - waypoints[i,2]))
  end_time = time.clock()

  plt.show()
  # display the results
  # cost consist of: total time, points in trajectory (vehicle speed), yaw differences
  cost = [end_time - start_time, len(trajectory_data[0]), np.sum(yaw_differences)]
  print("total cost:total time{}|trajectory{}|poses{}".format(cost[0],cost[1],cost[2]))
