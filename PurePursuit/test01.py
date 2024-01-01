# imports
import matplotlib.pyplot as plt
import numpy as np

# constants
start = [5.20, 11.00]
lookahead = 0.50

# for better readability...
x = 0
y = 1

# waypoints points for PPC
cx = [5.15, 5.14, 4.95, 4.18, 3.79]
cy = [10.94, 10.72, 9.94, 9.75, 9.77]
waypoints = np.asarray([cx, cy])


# pure pursuit controller
# 1 get current position
pos = np.array(start)

# 2 find nearest waypoint
def closest_node(node, nodes):
    dist = np.sum((nodes.T - node)**2, axis=1)
    return np.argmin(dist)
nearest_wpid = closest_node(pos, waypoints)
print(nearest_wpid)

# 3 find the waypoint nearest to lookahead
def closest_lookahead(node, nodes, lookahead):
	dist = np.abs(np.sqrt(np.sum(np.square(nodes.T - node), axis=1)) - lookahead)
	return np.argmin(dist)
nearest_laid = closest_lookahead(pos, waypoints, lookahead)
print(nearest_laid)

# 4 calculate alpha

# 5 calculate curvature and steering angle

# 6 update



# display
plt.cla()
plt.gca().invert_yaxis()	# invert y axis
plt.plot(cx, cy, ".r", label="nodes")
plt.plot(start[x], start[y], ".g", label="vehicle")
circle = plt.Circle((start[x], start[y]), lookahead, color='b', fill=False)
plt.gca().add_patch(circle)
#plt.plot(states.x, states.y, "-b", label="trajectory")
plt.legend()
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.axis("equal")
plt.grid(True)
plt.show()