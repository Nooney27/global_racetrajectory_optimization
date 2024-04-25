# MPC with Kinematic Bicycle model

# state space z = [x, y, theta, v], input vector u = [a, delta]
# kinematic bicycle model ODES: dx = v*cos(theta), dy = v*sin(theta), dtheta = v*tan(delta)/L, dv = a

# f(z,u) = A'z + B'u

# Objective function: minimize deviation from reference trajectory, minimize control effort, minimize deviation from reference velocity

# Constraints: future vehicle states must be feasible, control inputs must be feasible (follow linearized vehicle dynamic model)
#z_t+1 = Az_t + Bu_t + C
#initial state z_0 = zcurrent
# input inside vehicle limits: amin <= a <= amax, delta_min <= delta <= delta_max

import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd

# Load traj race cl:
file_path = r'C:\Users\inesl\OneDrive\Bureau\ETH 12\Thesis\global_racetrajectory_optimization\outputs\traj_race_cl.csv'
column_names = ['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2']
trajectory_data = pd.read_csv(file_path, delimiter=';', comment='#', names=column_names, skiprows=1)

plt.figure(figsize=(10, 5))
plt.plot(trajectory_data['x_m'], trajectory_data['y_m'], color='blue', linewidth=0.1, label='Race Trajectory', marker='o', markersize=4)
plt.title('Race Track Trajectory')
plt.xlabel('X position (meters)')
plt.ylabel('Y position (meters)')
plt.legend()
plt.grid(True)
plt.show()

# Parameters:

LENGTH = 0.50 # wheelbase
dt = 0.2 # time step
N = 20 # prediction horizon
WIDTH = 0.27 # width of vehicle
MIN_STEER = -0.4189
MAX_STEER = 0.4189
MAX_DSTEER = np.deg2rad(180.0) # max steering speed [rad/s]
MAX_SPEED = 12.0 # max speed [m/s]
MIN_SPEED = 0.0 # min speed [m/s]
MAX_ACCEL = 3.0 # max acceleration [m/ss]

# State and Input Definitions:

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


class MPC:
    def __init__(self, csv_file):
        self.waypoints = 
