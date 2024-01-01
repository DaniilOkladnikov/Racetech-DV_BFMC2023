"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import numpy as np
import math
import matplotlib.pyplot as plt

# Parameters
k = 0.9  # look forward gain
Lfc = 0.1  # [m] look-ahead distance
Kp = 1.  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 0.2  # [m] wheel base of vehicle

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    #  target course
    #cx = np.arange(0, 50, 0.5)
    #cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    #cx = np.arange(0, 6, 2)
    #cy = [1,2,3]

    #cx = [5.15, 5.14, 4.95, 4.18, 3.79]
    #cy = [10.94, 10.72, 9.94, 9.75, 9.77]

    map_x = [4.18, 4.18, 4.78, 5.15, 5.74, 5.15, 4.78, 4.96, 4.96, 4.96, 4.96, 1.94, 1.94, 2.53, 2.9, 3.49, 3.49, 2.9, 2.53, 2.72, 2.72, 2.72, 2.72, 4.18, 4.18, 4.77, 5.14, 5.73, 5.14, 4.77, 4.95, 4.95, 4.95, 4.95, 3.47, 3.47, 2.89, 2.52, 1.95, 1.95, 2.71, 2.71, 2.71, 1.92, 1.92, 2.5, 2.87, 3.45, 3.45, 2.68, 2.68, 2.68, 4.16, 4.16, 4.74, 5.11, 5.69, 4.93, 4.93, 4.93, 0.36, 0.73, 1.31, 1.31, 0.36, 0.73, 0.55, 0.55, 0.55, 0.36, 0.74, 1.31, 1.31, 0.36, 0.74, 0.55, 0.55, 0.55, 0.36, 0.74, 1.31, 1.31, 0.36, 0.74, 0.55, 0.55, 0.55, 3.83, 3.78, 2.88, 2.89, 2.91, 2.52, 2.52, 2.51, 5.12, 5.14, 5.15, 4.76, 4.76, 4.75, 1.54, 1.16, 0.86, 0.72, 0.72, 0.72, 0.74, 0.36, 0.37, 0.36, 0.39, 0.5, 0.76, 1.09, 1.46, 1.69, 3.87, 3.79, 1.56, 5.14, 5.14, 5.14, 4.76, 4.76, 4.76, 2.89, 2.89, 2.9, 2.53, 2.52, 2.52, 0.74, 0.75, 0.75, 0.36, 0.36, 0.35, 3.85, 3.8, 1.56, 1.69, 0.74, 0.75, 0.75, 0.36, 0.36, 0.36, 5.14, 5.14, 5.06, 4.95, 4.74, 4.48, 4.19, 3.85, 3.49, 3.12, 2.73, 2.36, 1.99, 1.61, 1.68, 2.06, 2.42, 2.78, 3.16, 3.53, 3.87, 4.18, 4.45, 4.62, 4.71, 4.75, 0.73, 0.78, 1.01, 1.36, 1.74, 2.12, 2.49, 2.87, 3.25, 3.63, 4.01, 4.39, 4.76, 5.14, 5.52, 5.9, 6.28, 6.66, 6.68, 6.3, 5.92, 5.54, 5.16, 4.78, 4.39, 4.01, 3.63, 3.25, 2.87, 2.48, 2.1, 1.72, 1.34, 0.97, 0.65, 0.43, 0.34, 0.34, 7.03, 7.41, 7.79, 8.17, 8.55, 8.92, 9.28, 9.63, 9.96, 10.29, 10.61, 10.95, 11.3, 11.66, 12.03, 12.41, 12.79, 13.17, 13.55, 13.93, 14.3, 14.68, 15.06, 6.91, 7.25, 7.63, 8.01, 8.39, 8.75, 9.12, 9.48, 9.81, 10.13, 10.45, 10.8, 11.14, 11.51, 11.88, 12.26, 12.64, 13.02, 13.39, 13.77, 15.1, 14.72, 14.34, 13.95, 13.57, 13.19, 12.81, 12.43, 12.05, 11.67, 11.3, 10.96, 10.61, 10.29, 9.95, 9.61, 9.25, 8.88, 8.5, 8.12, 7.74, 7.36, 6.98, 13.75, 13.39, 13.01, 12.62, 12.24, 11.87, 11.5, 11.15, 10.81, 10.47, 10.14, 9.8, 9.45, 9.08, 8.71, 8.33, 7.94, 7.56, 7.18, 6.82, 17.14, 17.52, 17.9, 18.28, 18.65, 19.03, 19.41, 19.77, 20.08, 20.27, 20.37, 20.36, 20.34, 20.32, 20.18, 19.93, 19.57, 19.19, 18.81, 18.43, 18.04, 17.66, 17.28, 16.9, 16.52, 16.2, 15.98, 15.87, 15.84, 15.84, 16.27, 16.26, 16.27, 16.38, 16.7, 17.08, 17.46, 17.83, 18.21, 18.59, 18.97, 19.35, 19.71, 19.91, 19.97, 19.96, 19.97, 19.91, 19.61, 19.24, 18.85, 18.47, 18.09, 17.71, 17.33, 16.25, 16.29, 16.57, 16.74, 16.67, 16.37, 16.0, 15.63, 15.42, 15.34, 15.58, 15.9, 16.9, 16.95, 15.82, 15.87, 15.87, 15.86, 15.87, 15.87, 15.74, 15.54, 15.37, 15.29, 15.3, 15.3, 15.29, 15.29, 15.28, 15.69, 15.69, 15.69, 15.67, 15.68, 15.68, 15.78, 15.97, 16.15, 16.25, 16.26, 16.26, 16.26, 15.29, 15.3, 15.3, 15.3, 15.31, 15.31, 15.31, 15.31, 15.32, 15.33, 15.46, 15.84, 16.21, 16.59, 16.91, 17.09, 17.1, 17.09, 17.09, 17.1, 17.08, 17.06, 16.81, 16.46, 16.08, 15.7, 15.69, 14.92, 14.54, 14.16, 13.78, 13.39, 13.01, 12.63, 12.25, 11.87, 11.48, 11.1, 10.72, 10.34, 9.96, 9.58, 9.19, 8.81, 8.43, 8.05, 7.67, 7.28, 6.07, 6.45, 6.83, 7.21, 7.58, 7.96, 8.34, 8.72, 9.1, 9.48, 9.85, 10.23, 10.61, 10.99, 11.37, 11.75, 12.12, 12.5, 12.88, 13.26, 13.64, 14.02, 14.39, 14.77, 15.15, 11.71, 12.09, 12.47, 12.84, 13.19, 13.5, 13.8, 14.08, 6.61, 6.65, 6.65, 6.64, 6.63, 6.63, 6.63, 6.61, 6.6, 6.22, 6.6, 6.61, 6.62, 6.81, 7.01, 6.9, 7.06, 7.03, 7.02, 7.04, 7.0, 6.92, 6.64, 6.26, 6.6]
    map_y = [6.88, 7.25, 7.84, 7.84, 6.88, 6.29, 6.29, 7.07, 7.07, 7.07, 7.07, 9.74, 10.11, 10.71, 10.71, 10.11, 9.74, 9.15, 9.15, 9.93, 9.93, 9.93, 9.93, 9.75, 10.13, 10.72, 10.72, 9.75, 9.16, 9.16, 9.94, 9.94, 9.94, 9.94, 6.86, 7.23, 7.81, 7.81, 6.86, 7.23, 7.05, 7.05, 7.05, 13.03, 12.66, 12.08, 12.08, 13.03, 12.66, 12.85, 12.85, 12.85, 13.04, 12.67, 12.09, 12.09, 13.04, 12.85, 12.85, 12.85, 3.19, 3.19, 3.77, 4.14, 4.71, 4.71, 3.95, 3.95, 3.95, 6.26, 6.26, 6.84, 7.21, 7.79, 7.79, 7.03, 7.03, 7.03, 9.15, 9.15, 9.73, 10.1, 10.68, 10.68, 9.91, 9.91, 9.91, 13.04, 12.67, 11.7, 11.32, 10.94, 11.09, 11.46, 11.84, 11.71, 11.32, 10.94, 11.1, 11.47, 11.85, 12.67, 12.63, 12.39, 12.03, 11.65, 11.27, 10.89, 11.06, 11.43, 11.81, 12.19, 12.55, 12.83, 13.01, 13.07, 10.13, 10.13, 9.77, 9.75, 8.78, 8.4, 8.02, 8.22, 8.6, 8.98, 8.77, 8.39, 8.01, 8.19, 8.57, 8.95, 8.77, 8.39, 8.01, 8.17, 8.55, 8.93, 7.25, 6.9, 6.87, 7.23, 5.88, 5.5, 5.12, 5.09, 5.47, 5.85, 5.91, 5.52, 5.15, 4.79, 4.46, 4.18, 3.92, 3.76, 3.62, 3.58, 3.59, 3.65, 3.74, 3.75, 4.18, 4.16, 4.03, 3.94, 3.94, 4.03, 4.19, 4.41, 4.68, 5.01, 5.38, 5.75, 2.81, 2.43, 2.12, 1.99, 1.99, 1.99, 1.99, 1.99, 1.99, 2.0, 2.0, 2.0, 2.0, 2.0, 2.01, 2.01, 2.0, 1.99, 1.61, 1.6, 1.6, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.71, 1.92, 2.23, 2.59, 2.97, 2.0, 2.0, 1.99, 1.99, 2.02, 2.09, 2.2, 2.35, 2.53, 2.71, 2.91, 3.07, 3.22, 3.32, 3.38, 3.4, 3.41, 3.41, 3.41, 3.41, 3.4, 3.4, 3.4, 2.27, 2.42, 2.41, 2.39, 2.41, 2.49, 2.6, 2.72, 2.9, 3.09, 3.29, 3.45, 3.6, 3.7, 3.76, 3.78, 3.79, 3.8, 3.82, 3.75, 3.02, 3.02, 3.02, 3.02, 3.01, 3.0, 2.99, 2.98, 2.97, 2.91, 2.79, 2.63, 2.46, 2.26, 2.09, 1.91, 1.77, 1.66, 1.62, 1.6, 1.59, 1.6, 1.59, 2.7, 2.58, 2.6, 2.61, 2.6, 2.53, 2.42, 2.27, 2.09, 1.91, 1.71, 1.55, 1.4, 1.27, 1.21, 1.18, 1.17, 1.16, 1.19, 1.32, 3.41, 3.41, 3.41, 3.41, 3.41, 3.41, 3.39, 3.28, 3.05, 2.72, 2.36, 1.97, 1.59, 1.21, 0.86, 0.57, 0.43, 0.38, 0.37, 0.37, 0.38, 0.38, 0.38, 0.38, 0.44, 0.64, 0.95, 1.31, 1.69, 2.07, 2.1, 1.72, 1.33, 0.97, 0.77, 0.76, 0.75, 0.75, 0.75, 0.76, 0.77, 0.77, 0.89, 1.2, 1.58, 1.96, 2.33, 2.71, 2.94, 3.03, 3.03, 3.03, 3.03, 3.03, 3.02, 4.23, 3.85, 3.59, 3.25, 2.88, 2.64, 2.57, 2.68, 3.0, 3.37, 3.66, 3.87, 3.4, 3.0, 2.45, 4.25, 4.63, 5.0, 5.38, 5.76, 6.12, 6.44, 6.77, 7.15, 7.52, 7.9, 8.28, 8.66, 9.04, 9.06, 8.68, 8.3, 7.92, 7.53, 7.15, 6.78, 6.45, 6.12, 5.75, 5.37, 4.99, 4.61, 9.41, 9.68, 10.06, 10.44, 10.81, 11.19, 11.57, 11.95, 12.33, 12.71, 13.06, 13.08, 13.06, 13.03, 12.82, 12.49, 12.11, 11.72, 11.34, 10.96, 10.58, 10.2, 9.91, 9.76, 9.75, 9.7, 9.31, 9.71, 9.75, 9.75, 9.75, 9.75, 9.75, 9.75, 9.74, 9.74, 9.75, 9.75, 9.75, 9.75, 9.75, 9.76, 9.77, 9.78, 9.78, 9.78, 9.78, 9.78, 13.05, 13.06, 13.06, 13.06, 13.06, 13.07, 13.07, 13.07, 13.07, 13.06, 13.07, 13.07, 13.07, 13.07, 13.07, 13.07, 13.07, 13.07, 13.07, 13.08, 13.08, 13.08, 13.08, 13.08, 13.07, 11.89, 11.88, 11.9, 11.96, 12.1, 12.31, 12.54, 12.79, 12.71, 12.33, 11.95, 11.57, 11.19, 10.8, 10.42, 10.04, 9.76, 9.76, 9.38, 8.99, 8.61, 8.28, 7.96, 9.78, 9.47, 9.09, 8.7, 8.32, 7.58, 7.2, 6.95, 6.9, 9.76]

    cx = [1.54, 1.16, 0.86, 0.72, 0.72, 0.72, 0.74, 0.74, 0.55, 0.74, 0.74, 0.75, 0.75, 0.74, 0.55, 0.74, 0.74, 0.75, 0.75, 0.73, 0.55, 1.31, 1.68, 2.06, 2.42, 2.78, 3.16, 3.53, 3.87, 4.18, 4.45, 4.62, 4.71, 4.75]
    cy = [12.67, 12.63, 12.39, 12.03, 11.65, 11.27, 10.89, 10.68, 9.91, 9.15, 8.77, 8.39, 8.01, 7.79, 7.03, 6.26, 5.88, 5.5, 5.12, 4.71, 3.95, 4.14, 4.18, 4.16, 4.03, 3.94, 3.94, 4.03, 4.19, 4.41, 4.68, 5.01, 5.38, 5.75]

    target_speed = 1.0  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=1.54, y=12.67, yaw=180.0, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while T >= time and lastIndex > target_ind:

        # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle

        time += dt
        states.append(time, state)

        if not show_animation:  # pragma: no cover
            plt.cla()
            plt.gca().invert_yaxis()    # invert y axis
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(map_x, map_y, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.gca().invert_yaxis()    # invert y axis
        plt.plot(map_x, map_y, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        #plt.subplots(1)
        #plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        #plt.xlabel("Time[s]")
        #plt.ylabel("Speed[km/h]")
        #plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()