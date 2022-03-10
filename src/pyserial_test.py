import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
def animate(i):
    # xs = np.random.randint(10)
    # ys = np.random.randint(10)
    ax1.clear()
    ax1.quiver(0,0, xs, ys, color=['b'])#np.sqrt((x1*x1)+(x2*x2)))
    ax1.set_title('Quiver plot with one arrow')
    ax1.set_xlim([-10, 10])
    ax1.set_ylim([-10, 10])
    ax1.plot(xs, ys)

style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ani = animation.FuncAnimation(fig, animate, interval=1000)

plt.show()
