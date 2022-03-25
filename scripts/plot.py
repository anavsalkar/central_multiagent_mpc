# importing mplot3d toolkits, numpy and matplotlib
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

x0_log = np.load('six_agent.npy')
# xw_log = np.load('outfile_without_obstacle.npy')

fig = plt.figure()
 
x, y, z = np.indices((10, 10, 10))
cube1 = (x>2) & (x<7) & (y>2) & (y<7) & (z>2) & (z<7) 

colors = np.empty(cube1.shape, dtype=object)
colors[cube1] = 'red'

# ax = plt.axes(projection ='3d')
ax = plt.axes()

# defining all 3 axes
last = 880
x1 = x0_log[:last,0]
y1 = x0_log[:last,1]
z1 = x0_log[:last,2]

x2 = x0_log[:last,3]
y2 = x0_log[:last,4]
z2 = x0_log[:last,5]

x3 = x0_log[:last,6]
y3 = x0_log[:last,7]
z3 = x0_log[:last,8]

x4 = x0_log[:last,9]
y4 = x0_log[:last,10]
z4 = x0_log[:last,11]

x5 = x0_log[:last,12]
y5 = x0_log[:last,13]
z5 = x0_log[:last,14]

x6 = x0_log[:last,15]
y6 = x0_log[:last,16]
z6 = x0_log[:last,17]

# zw = xw_log[2,:]
# xw = xw_log[0,:]
# yw = xw_log[1,:]
 
# plotting
ax.plot(x1,y1, '-D', markevery=[0], label ='agent 1')
ax.plot(x2,y2, '-D', markevery=[0],  label = 'agent 2')
ax.plot(x3,y3, '-D', markevery=[0],  label = 'agent 3')
ax.plot(x4,y4, '-D', markevery=[0], label = 'agent 4')
ax.plot(x5,y5, '-D', markevery=[0],  label = 'agent 5')
ax.plot(x6,y6, '-D', markevery=[0], label = 'agent 6')

ax.axis('equal')
# ax.scatter([-10],[0],'blue')
# ax.scatter(x, y, z, 'green', label = 'with obstacle')
# ax.scatter(xw,yw,zw, 'red', label = 'without obstacle')
# ax.voxels(cube1, facecolors=colors)
#ax.set_title('3D line plot geeks for geeks')
ax.set_xlabel("X")
ax.set_ylabel("Y")
# ax.set_zlabel("Z")
plt.legend()
plt.show()
plt.savefig('hexa.png', dpi=300)

