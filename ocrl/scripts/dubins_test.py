import dubins, math,random
import matplotlib.pyplot as plt


x_lim = [-10,10]
y_lim = [-10,10]

q0 = (0,0,.1)
q1 = (1,0,2.78)
turning_radius = 1.0
step_size = 0.5

path = dubins.shortest_path(q0,q1, turning_radius)
configurations, _ = path.sample_many(step_size)

print("path:{}".format(path))
print("configurations:{}".format(configurations))

x_trajectory = []
y_trajectory = []
for (x,y,yaw) in configurations:
  x_trajectory.append(x)
  y_trajectory.append(y)

def draw():
  plt.plot(x_trajectory,y_trajectory,'b',linewidth=2)
  plt.arrow(q0[0],q0[1],math.cos(q0[2]),math.sin(q0[2]),color = 'g',width = .2,
  head_width = .6)
  plt.arrow(q1[0],q1[1],math.cos(q1[2]),math.sin(q1[2]),color = 'r',width = .1,
  head_width = .3)
  plt.xlim(-13,13)
  plt.ylim(-13,13)
  # draw hard boundaries
  plt.plot([x_lim[0],x_lim[1],x_lim[1],x_lim[0],x_lim[0]],
  [y_lim[0],y_lim[0],y_lim[1],y_lim[1],y_lim[0]],'k')
  # draw waypoint boundaries
  plt.plot([x_lim[0]+1,x_lim[1]-1,x_lim[1]-1,x_lim[0]+1,x_lim[0]+1],
  [y_lim[0]+1,y_lim[0]+1,y_lim[1]-1,y_lim[1]-1,y_lim[0]+1],'y')


def generate_traj(reverse = False):
  global q0,q1,path, configurations,x_trajectory,y_trajectory
  if (reverse == True):
    q0[2] = q0[2] + 3.14
    q1[2] = q1[2] + 3.14
  # random path generation and resolving of path constraints
  path = dubins.shortest_path(q0,q1, turning_radius)
  configurations, _ = path.sample_many(step_size)
  x_trajectory = []
  y_trajectory = []
  for (x,y,yaw) in configurations:
    x_trajectory.append(x)
    y_trajectory.append(y)

def generate_mixed_traj(ind,f_to_r = True):
  global q0,q1,path,configurations,x_trajectory,y_trajectory
  # forward path
  x_trajectory = x_trajectory[0:ind]
  y_trajectory = y_trajectory[0:ind]
  # reverse direction
  q0 =  [x_trajectory[-1],y_trajectory[-1],q0[2]+3.14]
  q1[2] = q1[2] + 3.14
  path = dubins.shortest_path(q0,q1,turning_radius)
  configurations, _ = path.sample_many(step_size)
  for (x,y,yaw) in configurations:
    x_trajectory.append(x)
    y_trajectory.append(y)

def traj_inside(x_traj,y_traj,x_bounds,y_bounds):
  result = True
  for x in x_traj:
    if (x<x_bounds[0] or x>x_bounds[1]):
      result = False
  for y in y_traj:
    if (y<y_bounds[0] or y>y_bounds[1]):
      result = False
  return result

def index_before_boundary(x_traj,y_traj,x_bounds,y_bounds):
  ind = -1
  hit_bounds = False
  for i,point in enumerate(zip(x_traj,y_traj)):
    print("enumerating {}".format(i))
    if (hit_bounds == False):
      if ((point[0]<x_bounds[0] or point[0]>x_bounds[1])
      or (point[1]<y_bounds[0] or point[1]>y_bounds[1])):
        if (i == 0):
          ind = 0
        else:
          ind = i - 1
        hit_bounds = True

  return ind


win_count = 0
while (win_count < 1000):
  q0 = [random.uniform(.75,.99) * random.choice([1,-1]) * abs(x_lim[0]),
  random.uniform(.75,.99) * random.choice([1,-1]) * abs(y_lim[0]),
  random.uniform(0,6.28)]

  q1 = [random.uniform(.75,.9) * random.choice([1,-1]) * abs(x_lim[0]),
  random.uniform(.75,.9) * random.choice([1,-1]) * abs(y_lim[0]),
  random.uniform(0,6.28)]

  f_ind,r_ind = (-1,-1)
  generate_traj()
  plt.draw()
  draw()
  plt.pause(5)
  print("trajectories resolved: {}".format(win_count))
  plt.close()
  if (traj_inside(x_trajectory,y_trajectory,x_lim,y_lim) == False):
    print("forward trajectory went out of bounds, reverse trajectory")
    generate_traj(reverse = True)
    r_ind = index_before_boundary(x_trajectory,y_trajectory,x_lim,y_lim)
    if (r_ind == 0):
      print("cannot solve trajectory")
      break
    plt.draw()
    draw()
    plt.pause(5)
    plt.close()
    if (traj_inside(x_trajectory,y_trajectory,x_lim,y_lim) == False):
      break
  win_count = win_count + 1
