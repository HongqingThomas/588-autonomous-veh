
import numpy as np
import matplotlib.pyplot as plt
# x1, y1 = 2, -1
# x2, y2 = 2+np.sqrt(3), -2
#
# angle = np.arctan((x1-x2)/(-y1+y2))
# # angle1 = np.arctan(np.pi / 3)
# print("angle:", angle, np.pi / 3)

# given a (x,y) coordinate
x, y = -1, 10

theta =  -1 * np.pi / 3

x_o, y_o = x * np.cos(theta) - y * np.sin(theta), x * np.sin(theta) + y * np.cos(theta)
#坐标轴
x1, y1 = -10 * np.cos(theta) - 0 * np.sin(theta), -10 * np.sin(theta) + 0 * np.cos(theta)
x2, y2 = 10 * np.cos(theta) - 0 * np.sin(theta), 10 * np.sin(theta) + 0 * np.cos(theta)
x3, y3 = 0 * np.cos(theta) + 10 * np.sin(theta), 0 * np.sin(theta) + -10 * np.cos(theta)
x4, y4 = 0 * np.cos(theta) - 10 * np.sin(theta), 0 * np.sin(theta) + 10 * np.cos(theta)
print("x_o, y_o:", x_o, y_o, x**2 + y**2, x_o**2 + y_o**2)
print((x3-x1)**2 + (y3 - y1)**2 - 2 * 10 **2 )
fig, ax = plt.subplots()
# ax.plot([0, x], [0, y], color='green') # 旋转后的
ax.plot([0, x_o], [0, y_o], color='blue')
ax.plot([-10, 10], [0, 0], color='green')
ax.plot([0, 0], [-10, 10], color='green')
ax.plot([x1, x2], [y1, y2], color='blue')
ax.plot([x3, x4], [y3, y4], color='blue')

# ax.scatter(x, y, color='green', s = 100)
ax.scatter(x_o, y_o, color='blue', s = 100)
ax.scatter(0, 10, color='red', s = 100)
ax.scatter(x4, y4, color='red', s = 100)
plt.show()