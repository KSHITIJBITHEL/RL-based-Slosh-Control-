import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

# make sure the 'COM#' is set according the Windows Device Manager
ser = serial.Serial('COM7', 115200, timeout=1)
time.sleep(0.1)
data = [0, 0]

def animate(i):
	# xs = np.random.randint(10)
	# ys = np.random.randint(10)

	line = ser.readline()   # read a byte
	if line:
		string = line.decode()  # convert the byte string to a unicode string
		string = string.split(",")
		data[0] = float(string[0])
		data[1] = float(string[1][0:-1])
		print(data)
	ax1.clear()
	ax1.quiver(0,0, data[0], data[1], color=['b'],scale = 5)#np.sqrt((x1*x1)+(x2*x2)))
	ax1.set_title('Slosh Direction')
	ax1.set_xlim([-20, 20])
	ax1.set_ylim([-20, 20])
	# ser.flushInput()
	# ax1.plot(data[0], data[1])





style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
# ser.flushInput()
ani = animation.FuncAnimation(fig, animate, interval=1)
plt.show()

ser.close()