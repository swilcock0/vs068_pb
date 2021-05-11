import matplotlib.pyplot as plt
from vs068_pb.utils import spline_uni

x = list(range(10))
y = [0.1, 0.5, 0.3, 1, 2, 0.1, 0.5, 0.3, 1, 2]

fig, ax = plt.subplots()

ax.plot(x,y, 'b-')
ax.plot(x,y, 'yx')


y2 = spline_uni(y)

x2 = list(range(len(y2)))

ax.plot(x,y2, 'r-')



plt.show()