import numpy as np
np.set_printoptions(precision=2, suppress=True)

# global variables
INITIAL_POSE = np.array([0, 0, 0]) # required

timestep = 0
waypoints_to_hit = []

# the simulator expects this function to be defined
# it gets called at the beginning of the simulation
def init(dt, waypoints, robot_radius, robot_wheel_radius, robot_axle_width, robot_max_speed, waypoint_tolerance, path_width):
    global INITIAL_POSE, waypoints_to_hit, timestep
    INITIAL_POSE = np.array([0, -4, np.pi/2])
    waypoints_to_hit = waypoints
    timestep = 0

# beacons in order clockwise (nw, ne, se, sw)
def update(beacons):
    global timestep, waypoints_to_hit
    print(timestep, beacons)
    # implement robot logic here!
    timestep += 1
    return 0.5, 0.5
