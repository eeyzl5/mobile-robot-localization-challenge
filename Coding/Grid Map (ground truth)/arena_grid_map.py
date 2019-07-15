import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import matplotlib.collections as mcoll

################################################################################
# Creating the grid map of the arena
################################################################################

# 10x10 cm2 grid
size = (32,43)
x=size[1]
y=size[0]

# Create blank map
Map = np.zeros(size)

# Create obstacles

# Top right triangle
Map[0, 40:43] = 1
Map[1, 41:43] = 1
Map[2, 42:43] = 1
# Middle wall
Map[7:9, 10:31] = 1
# Middle triangle
Map[15, 13:15] = 1
Map[16, 13:15] = 1
Map[17, 12:16] = 1
Map[18, 12:16] = 1
Map[19, 11:17] = 1
Map[20, 11:17] = 1
Map[21, 10:18] = 1
# Bottom left triangle
Map[25, 0:1] = 1
Map[26, 0:2] = 1
Map[27, 0:3] = 1
Map[28, 0:3] = 1
Map[29, 0:4] = 1
Map[30, 0:5] = 1
Map[31, 0:6] = 1
# Bottom right square
Map[21:32, 32:43] = 1




################################################################################
# Below codes are just for visulizing the map, 
# not necessary to put them into raspberry pi
################################################################################

fig, ax = plt.subplots()
cmap = mcolors.ListedColormap(['white', 'gray'])
bounds = [-0.5, 0.5, 1.5]
norm = mcolors.BoundaryNorm(bounds, cmap.N)
data = Map
im = ax.imshow(data, cmap=cmap, norm=norm)



grid = np.arange(-0.5, x, 1)
xmin, xmax, ymin, ymax = -0.5, x, -0.5, y
lines = ([[(x, y) for y in (ymin, ymax)] for x in grid]
         + [[(x, y) for x in (xmin, xmax)] for y in grid])
grid = mcoll.LineCollection(lines, linestyles='solid', linewidths=2,
                            color='black')
ax.add_collection(grid)

plt.show()