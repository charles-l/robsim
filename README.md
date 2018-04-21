# robsim

Simulator for Intro to Intelligent Robotics.

Using map:

![](map_screenshot.png)

# USAGE

Implement your robot logic in a separate python file (I'd recommend copying the `robots.py` file and using it as a starting point). To simulate it, run

`./robsim.py your-robot-file.py`

For instance, to run the test robot in `robot.py`, you would run

`./robsim.py robot.py`

# Robot API

The simulator expects the following to be defined in your implementation:

* `INITIAL_POSE` - a three-tuple that defines the starting (x, y, theta) of the robot.
* `init(dt, list_of_waypoints)` - a function that will accept the delta time, and waypoints of the robots path. Only is called once at the beginning of the simulation, so make sure to assign the list to a variable or something.
* `update(beacons)` - the main function for the robot. Called every tick (0.5 seconds), and expects the return value to be a tuple `(left_wheel_velocity, right_wheel_velocity)`. `beacons` is a list of distance readings to the beacons in the order (nw, ne, se, sw)
