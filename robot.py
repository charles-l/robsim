import numpy as np

INITIAL_POSE = np.array([0, 0, np.pi/4])

waypoints_to_hit = []

# the simulator expects this function to be defined
# it gets called at the beginning of the simulation
def receive_waypoints(w):
    waypoints_to_hit = w

# waypoints in order clockwise (nw, ne, se, sw)
def update(waypoints):
    print(waypoints)
    # implement robot logic here!
    return 0.5, 0.5
