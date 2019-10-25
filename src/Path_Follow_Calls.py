import numpy as np
import math
import matplotlib.pyplot as plt


k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 2.0  # speed proportional gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle
max_ldist = 3
old_nearest_point_index = None

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))


class return_pose:
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    state.rear_x = state.x - ((L / 2) * math.cos(state.yaw))
    state.rear_y = state.y - ((L / 2) * math.sin(state.yaw))

    return state

def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_distance(state, point_x, point_y):

    dx = state.rear_x - point_x
    dy = state.rear_y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)


def calc_target_index(state, cx, cy):

    global old_nearest_point_index

    global max_ldist

    if old_nearest_point_index is None:
        # search nearest point index
        dx = [state.rear_x - icx for icx in cx]
        dy = [state.rear_y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        old_nearest_point_index = ind
    else:
        ind = old_nearest_point_index
        distance_this_index = calc_distance(state, cx[ind], cy[ind])
        old_dist_index = distance_this_index
        #print(distance_this_index)

        while True:
            ind = ind + 1 if (ind + 1) < len(cx) else ind
            distance_next_index = calc_distance(state, cx[ind], cy[ind])
            if distance_this_index < distance_next_index:
                break
            distance_this_index = distance_next_index

        #new added statement
        if distance_this_index >= max_ldist:
            ind = old_nearest_point_index
        else:
            old_nearest_point_index = ind

    L = 0.0

    Lf = k * state.v + Lfc


    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind] - state.rear_x
        dy = cy[ind] - state.rear_y
        L = math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
    return ind


def reset_dist(reset_dist = old_nearest_point_index):
    global old_nearest_point_index
    old_nearest_point_index = reset_dist


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)



