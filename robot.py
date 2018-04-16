import numpy as np

INITIAL_POSE = np.array([0, 0, np.pi/4])

waypoints_to_hit = []

def receive_waypoints(w):
    waypoints_to_hit = w

def update():
    # implement robot logic here!
    return 0.5, 0.5
