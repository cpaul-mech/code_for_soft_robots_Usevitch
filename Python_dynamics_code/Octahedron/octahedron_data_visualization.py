import numpy as np
import matplotlib.pyplot as plt

# Load data
node0 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node0.csv', delimiter=',')
node1 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node1.csv', delimiter=',')
node2 = np.loadtxt('Python_dynamics_code/Octahedron/octahedron_data_node2.csv', delimiter=',')

# plot the data in 3D space
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(node0[:,1], node0[:,2], node0[:,3], label='Node 0')
ax.plot(node1[:,1], node1[:,2], node1[:,3], label='Node 1')
ax.plot(node2[:,1], node2[:,2], node2[:,3], label='Node 2')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# calculate the vector from the original position of each node to the center of the truss
center = np.array([np.mean([node0[0,1], node1[0,1], node2[0,1]]), np.mean([node0[0,2], node1[0,2], node2[0,2]]), np.mean([node0[0,3], node1[0,3], node2[0,3]])])
ax.plot(center[0], center[1], center[2], 'ro', label='Center of Truss')

#Specify the z-axis limits
ax.set_zlim(0, 2.0)
ax.legend()
plt.show()

# map the data from each node onto the vector that points to the center of the truss
# from the original position of each node

# unit vector for node 0:
unit_vector0 = (center - node0[0,1:4]) / np.linalg.norm(center - node0[0,1:4])
# unit vector for node 1:
unit_vector1 = (center - node1[0,1:4]) / np.linalg.norm(center - node1[0,1:4])
# unit vector for node 2:
unit_vector2 = (center - node2[0,1:4]) / np.linalg.norm(center - node2[0,1:4])

# project the data from each node onto the unit vector
node0_proj = np.dot(node0[:,1:4], unit_vector0)
node1_proj = np.dot(node1[:,1:4], unit_vector1)
node2_proj = np.dot(node2[:,1:4], unit_vector2)

# Center the data by subtracting the mean

node0_proj = node0_proj - np.mean(node0_proj)
node1_proj = node1_proj - np.mean(node1_proj)
node2_proj = node2_proj - np.mean(node2_proj)

# plot the data on separate 2D plots
plt.figure()
plt.plot(node0[:,0], node0_proj, label='Node 0')
plt.plot(node1[:,0], node1_proj, label='Node 1')
plt.plot(node2[:,0], node2_proj, label='Node 2')
plt.xlabel('Time')
plt.ylabel('Displacement')
plt.legend()
plt.show()

