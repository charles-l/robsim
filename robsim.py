#!/usr/bin/env python3
import sys
import numpy as np
import importlib
import matplotlib.pyplot as plt
import re

DT = 0.5

cos = np.cos
sin = np.sin
pi = np.pi

def normalize_angle(t):
    return np.arctan2(sin(t), cos(t))

class Robot():
    def __init__(self, pose, axle, wheel_radius, max_speed):
        self.pose = pose
        self.d = axle
        self.r = wheel_radius
        self.past_poses = [pose]
        self.max_speed = max_speed

    # differential drive forward kinematics
    def _fk(self, v_lwheel, v_rwheel):
        theta = self.pose[2]
        R = np.array([[cos(theta), -sin(theta), 0],
                      [sin(theta),  cos(theta), 0],
                      [         0,           0, 1]])
        xi_prime_R = np.array([((self.r * v_lwheel) / 2) + ((self.r * v_rwheel) / 2),
                               0,
                               ((self.r * v_rwheel) / self.d) - ((self.r * v_lwheel) / self.d)]).transpose()
        delta = R.dot(xi_prime_R)

        newpose = self.pose + (delta * DT)
        # convert back to angle within the unit circle
        newpose[2] = normalize_angle(newpose[2])
        return newpose


    def _scalev(self, l, r):
        m = max(abs(l), abs(r))
        if m > self.max_speed:
            sys.stderr.write("warning: attempting to drive faster than max wheel speed. You may want to check your code.\n")
            l = (l / m) * self.max_speed
            r = (r / m) * self.max_speed
        return l, r

    def send_command(self, l_wheel_v, r_wheel_v):
        """This is the publicly exposed function for controlling the robot.
        Expects a left and right wheel velocity."""
        l, r = self._scalev(l_wheel_v, r_wheel_v)
        self.pose = self._fk(l, r)
        self.past_poses.append(self.pose)

    def read_waypoints(self):
        return [np.linalg.norm(self.pose[:2] - x) for x in [(-3, 5), (3, 5), (3, -5), (-3, -5)]]

def plot_robot_path(r):
    """Plots the robot's path using matplot"""
    plt.plot([p[0] for p in r.past_poses], [p[1] for p in r.past_poses], 'k')
    for p in r.past_poses:
        plt.quiver(p[0], p[1], cos(p[2]), sin(p[2]))


def load_map():
    """Returns the loaded map file as a matrix"""
    return np.load('map.npz')['arr_0']

def plot_map(M):
    """Plots the map file with the proper orientation"""
    plt.imshow(M, cmap=plt.cm.gray, interpolation='none', origin='upper', extent=(-3, 3, -5, 5))
    plt.xlabel('x')
    plt.ylabel('y')

PATH_WIDTH = 0.5

def on_path(p, robot_radius = 0):
    def in_arc(xy, r, is_convex):
        v = p - xy
        d = np.linalg.norm(v)
        if (v[1] >= 0 and is_convex) or v[1] <= 0 and not is_convex:
            return r - (PATH_WIDTH / 2) + robot_radius <= d <= r + (PATH_WIDTH / 2) - robot_radius
        return False
    def in_edge(p1, p2):
        v = (p2 - p1)
        l = np.array([p1, p2])
        theta = (np.pi / 2) - np.arctan2(v[1], v[0])
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        tl = R.dot(l.T)
        tp = R.dot(p.T)
        mi, ma = min(tl[1, 1], tl[1, 0]), max(tl[1, 1], tl[1, 0])
        if mi <= tp[1] <= ma and \
                tl[0, 0] - (PATH_WIDTH / 2) + robot_radius <= tp[0] <= tl[0, 0] + (PATH_WIDTH / 2) - robot_radius:
                    return True
        return False
    b = in_arc(np.array([0, 2]), 2, True) or \
            in_arc(np.array([0, -2]), 2, False) or \
            in_edge(np.array([-2, -2]), np.array([-2, 2])) or \
            in_edge(np.array([-2, 2]), np.array([2, 2])) or \
            in_edge(np.array([2, -2]), np.array([2, 2])) or \
            in_edge(np.array([-2, -2]), np.array([2, -2])) or \
            in_edge(np.array([-2, -2]), np.array([2, 2])) or \
            in_edge(np.array([0, -4]), np.array([0, 4]))
    return b

def plot_line_map():
    field = np.array([[-3, 5], [3, 5], [3, -5], [-3, -5], [-3, 5]])

    x = np.linspace(-2, 2)
    track = np.array([[2, 2], [2, -2], [-2, -2], [-2, 2], [2, 2], [-2, -2]])
    line = np.array([[0, 4], [0, -4]])
    # lower arc
    plt.plot(x, -np.sqrt(2**2 - x**2) - 2, color='lightgray')

    # upper arc
    plt.plot(x, np.sqrt(2**2 - x**2) + 2, color='lightgray')
    plt.plot(field[:,0], field[:,1], color='lightgray')
    plt.plot(track[:,0], track[:,1], color='lightgray')
    plt.plot(line[:,0], line[:,1], color='lightgray')

def make_occupancy_map():
    """Generate the occupancy map (size 600x1000). Returns a 2D array of booleans.
    VERY SLOW - generate the image and use it later with load_map() (rather than regenerating it every time)."""
    x = np.linspace(-3, 3, 600)
    y = np.linspace(5, -5, 1000)
    coords = np.transpose([np.tile(x, len(y)), np.repeat(y, len(x))])
    i = np.array([on_path(c) for c in coords])
    return i.reshape(1000, 600)

ROBOT_RADIUS = 0.25 / 2
ROBOT_AXLE_WIDTH = 0.25
MAX_SIMULATION_STEPS = 1000
WAYPOINT_TOLERANCE = 0.05

def load_robot_code(fname):
    """Loads a module file and asserts the the required functions are defined"""
    m = importlib.import_module(re.sub("\.py", "", fname))
    for x in ['INITIAL_POSE', 'receive_waypoints', 'update']:
        if x not in dir(m):
            raise Exception(f"You need to define " + x + " in your {fname} file!")
    return m


def simulate(m, waypoints):
    """Performs simulation on python module `m`
       Returns 4 values - (success, seconds, num_waypoints_hit, robot)
           success - whether the robot hit every waypoint without colliding with a wall
           seconds - the amount of time it took for the robot to complete the path
           num_waypoints_hit - how many waypoints were hit before failing or succeeding
           robot - the robot object for analysis"""

    waypoints_hit = 0
    m.receive_waypoints(waypoints)
    r = Robot(np.array(m.INITIAL_POSE), 0.25, ROBOT_RADIUS, 0.5)
    for i in range(MAX_SIMULATION_STEPS):
        r.send_command(*m.update(r.read_waypoints()))
        if np.linalg.norm(r.pose[:2] - waypoints[waypoints_hit]) <= WAYPOINT_TOLERANCE + ROBOT_RADIUS:
            waypoints_hit += 1
            if waypoints_hit == len(waypoints):
                print(f"success! (took {i * DT} seconds)")
                return True, i * DT, waypoints_hit, r
        if not on_path(r.pose[0:2], robot_radius = ROBOT_RADIUS):
            print(f"crashed at {r.pose[0:2]} (took {i * DT} seconds)")
            print(f"hit {waypoints_hit} waypoints out of {len(waypoints)}")
            return False, i * DT, waypoints_hit, r
    print("ran out of time")
    return False, i * DT, waypoints_hit, r


def plot_run(success, waypoints, robot, type='line', show_path = True):
    """Plots the map, waypoints, and robot travel path with matplotlib.
       success - boolean defining whether the run was a success not
       waypoints - a list of tuples defining waypoints for the robot to hit
       robot - the robot object that just completed a run (plots the path of that robot)
       type - 'line' for line graph or 'occ' for occupancy map. Defaults to 'line'
       show_path - toggle whether the robot path is shown in the plot or not. Defaults to True."""
    _, ax = plt.subplots()
    plt.axis('equal')
    if type == 'line':
        plot_line_map()
    elif type == 'occ':
        M = load_map()
        plot_map(M)
    w = np.array(waypoints)
    plt.plot(w[:,0], w[:,1], 'sg')
    if did_succeed:
        c = plt.Circle((r.pose[0], r.pose[1]), ROBOT_RADIUS, color='b')
    else:
        c = plt.Circle((r.pose[0], r.pose[1]), ROBOT_RADIUS, color='r')
    if show_path:
        plot_robot_path(r)
    ax.add_artist(c)
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("USAGE: robsim YOURFILE.py")
        sys.exit(0)

    # do a dumb test run
    m = load_robot_code(sys.argv[1])
    waypoints = [(1,1), (2,2)]
    did_succeed, _, _, r = simulate(m, waypoints)
    plot_run(did_succeed, waypoints, r, type='occ')

