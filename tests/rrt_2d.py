from vs068_pb.rrt import rrt, configs
from vs068_pb.rrt_connect import rrt_connect
from vs068_pb.rrt_star import rrt_star
from vs068_pb.utils import randomize
import matplotlib.pyplot as plt
import matplotlib.patches as patches


plot_tree=True

boxes = [
        [[0, 2], [3, 4]], 
        [[2.5, 4.5], [2.5, 4.5]],
        [[5, 6], [1, 2]],
        [[5, 10], [3, 4]]
        ]

def collision_fn(conf):
    x, y = conf
    for box in boxes:
        #print(box)
        if x >= box[0][0] and x <= box[0][1] and y >= box[1][0] and y <= box[1][1]:
            return True
    return False

start = [0,0]
end = [5,5]

if plot_tree:
    path, _, nodes = rrt_connect(start, end, collision_fn, step=0.5, n_it=1000, time_limit=10.0, return_tree=True, limits=[[0,0], [10,10]])
else:
    path, _, nodes = rrt_connect(start, end, collision_fn, step=0.5, n_it=1000, time_limit=10.0, limits=[[0,0], [10,10]]), []

fig, ax = plt.subplots()

nodes = randomize(nodes)

for node in nodes[:-1]: # Can limit the amount of nodes displayed
    path_a = configs(node.retrace())
    x = [p[0] for p in path_a]
    y = [p[1] for p in path_a]   

    plt.plot(x,y, 'k-')

x = [p[0] for p in path]
y = [p[1] for p in path]   

plt.plot(x,y, 'b-')
plt.plot(x,y, 'yx')


plt.plot(start[0], start[1], 'go')
plt.plot(end[0], end[1], 'ro')

# Create a Rectangle patch
for box in boxes:
    widths = ( (box[0][1] - box[0][0]), (box[1][1] - box[1][0]) )
    corner = ( box[0][0], box[1][0] )

    rect = patches.Rectangle(corner, widths[0], widths[1], linewidth=1, edgecolor='r', facecolor='none')
    ax.add_patch(rect)

plt.show()