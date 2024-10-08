import numpy as np
from RigidityMatrix2D import RigidityMatrix2D

b = 30.0
k = 2000.0
m = 1.975
g = 9.81
# Gravity acts in the -y direction
# Vector follows the order of x1, x2, x3, y1, y2, y3
g_vector = np.array([0.0, 0.0, 0.0, -m*g, -m*g, -m*g])

# Define the initial conditions
RM = RigidityMatrix2D()
x = RM.x
x1 = x[0, 0]
y1 = x[0, 1]
x2 = x[1, 0]
y2 = x[1, 1]
x3 = x[2, 0]
y3 = x[2, 1]

x1_dot = y1_dot = x2_dot = y2_dot = x3_dot = y3_dot = 0.0

# Define the time parameters
Ts = 0.01
t_end = 10.0
n_steps = int(t_end / Ts)
t_start = 0.0

if __name__ == "__main__":
    # run the model_1_sim file
    import triangle_1_sim