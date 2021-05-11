from vs068_pb.rrt import rrt, configs, TreeNode
from vs068_pb.rrt_connect import rrt_connect
from vs068_pb.rrt_star import rrt_star, OptimalNode
from vs068_pb.rrt_star_connect import birrt_star
from vs068_pb.utils import randomize
from vs068_pb.naive_smooth import shortcut
import matplotlib.pyplot as plt
import matplotlib.patches as patches

RRT = 1
RRT_Connect = 2
RRT_Star = 3
BIRRT_Star = 4

def get_planner(select=RRT_Connect):
    switcher = {
        RRT : rrt,
        RRT_Connect : rrt_connect,
        RRT_Star : rrt_star,
        BIRRT_Star : birrt_star
    }

    return switcher.get(select)
planner_choice = 'RRT'

planner = get_planner(eval(planner_choice))


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
    path, _, nodes = planner(start, end, collision_fn, step=0.5, n_it=1000, time_limit=3.0, return_tree=True, limits=[[0,0], [10,10]], rad=1.0)
else:
    path, _, nodes = planner(start, end, collision_fn, step=0.5, n_it=1000, time_limit=3.0, limits=[[0,0], [10,10]], rad=1.0), []


#

fig, (ax1, ax2) = plt.subplots(1,2, sharex=True, sharey=True)

ax1.set_title("Paths : {}".format(planner_choice))
ax2.set_title("Smoothed")

nodes = randomize(nodes)

for node in nodes[:-1]: # Can limit the amount of nodes displayed
    if isinstance(nodes[-1], TreeNode):
        path_a = configs(node.retrace())
    else:
        path_a = node.retrace()

    x = [p[0] for p in path_a]
    y = [p[1] for p in path_a]   

    ax1.plot(x,y, 'k-')

x = [p[0] for p in path]
y = [p[1] for p in path]   

ax1.plot(x,y, 'b-')
ax1.plot(x,y, 'yx')

ax1.plot(start[0], start[1], 'go')
ax1.plot(end[0], end[1], 'ro')

# Create a Rectangle patch
for box in boxes:
    widths = ( (box[0][1] - box[0][0]), (box[1][1] - box[1][0]) )
    corner = ( box[0][0], box[1][0] )

    rect = patches.Rectangle(corner, widths[0], widths[1], linewidth=1, edgecolor='r', facecolor='none')
    ax1.add_patch(rect)


path = shortcut(path, collision_fn, step=0.5, time_limit=0.5)

x = [p[0] for p in path]
y = [p[1] for p in path]   

ax2.plot(x,y, 'b-')
ax2.plot(x,y, 'yx')

ax2.plot(start[0], start[1], 'go')
ax2.plot(end[0], end[1], 'ro')

# Create a Rectangle patch
for box in boxes:
    widths = ( (box[0][1] - box[0][0]), (box[1][1] - box[1][0]) )
    corner = ( box[0][0], box[1][0] )

    rect = patches.Rectangle(corner, widths[0], widths[1], linewidth=1, edgecolor='r', facecolor='none')
    ax2.add_patch(rect)

plt.show()