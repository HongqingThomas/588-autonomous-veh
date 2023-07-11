# from copy import deepcopy
# import math
# import numpy as np
# # Define the planning cube
# A = [[1, 1], [5, 5]]

# # Define the start and goal points
# start = (1, 1, 0)
# goal = (5, 5, 0)

# # Define the cylinder obstacles
# c1_x, c1_y, r1 = 2, 3, 1
# c2_x, c2_y, r2 = 4, 2, 1

# # Define the step size for the path planning algorithm
# step_size = 0.3

# # Define the maximum turning angle for the car
# max_turning_angle = 30


# # Define the function to check if a point is inside an obstacle
# def is_inside_obstacle(x, y):
#     if ((x - c1_x) ** 2 + (y - c1_y) ** 2) <= r1 ** 2:
#         return True
#     elif ((x - c2_x) ** 2 + (y - c2_y) ** 2) <= r2 ** 2:
#         return True
#     else:
#         return False


# # Define the function to generate the path
# def generate_path(start, goal, step_size, max_turning_angle):
#     # Initialize the path with the start point
#     path = [start]
#     # print("path: ", path)

#     # Initialize the current point as the start point
#     current_point = start
#     iter_count = 0
#     # Loop until the current point reaches the goal point
#     # while current_point != goal:
#     while ((current_point[0] - goal[0])**2 + (current_point[1] - goal[1])**2) > 0.1**2:
#         if iter_count > 1000:
#             path_list = [list(t) for t in path]
#             return path_list
#         iter_count += 1
#         # print("in loop", iter_count)
#         # Calculate the distance and angle to the goal point
#         dx = goal[0] - current_point[0]
#         dy = goal[1] - current_point[1]
#         distance = math.sqrt(dx ** 2 + dy ** 2)
#         angle = math.atan2(dy, dx)

#         # Calculate the maximum turning angle based on the distance to the goal point
#         # max_turning_angle = int(min(max_turning_angle, math.degrees(math.atan2(2 * step_size, distance))))

#         # Initialize the best point and best cost
#         best_point = None
#         best_cost = float('inf')

#         # Loop through all possible next points
#         for i in range(-max_turning_angle, max_turning_angle + 1, 5):
#             # Calculate the next point and cost
#             # print("i:", i, np.radians(i))
#             # print("angle:", angle)
#             next_point = (current_point[0] + step_size * math.cos(angle + np.radians(i)), current_point[1] + step_size * math.sin(angle + np.radians(i)), angle + np.radians(i))
#             cost = distance + math.sqrt((next_point[0] - goal[0]) ** 2 + (next_point[1] - goal[1]) ** 2)

#             # Check if the next point is inside an obstacle
#             if is_inside_obstacle(next_point[0], next_point[1]):
#                 continue

#             # Check if the cost is better than the current best cost
#             if cost < best_cost:
#                 best_point = next_point
#                 best_cost = cost

#         # Add the best point to the path
#         path.append(best_point)
#         # print("path: ", path)

#         # # Update the current point
#         current_point = best_point
#         # current_point[0] = best_point[0]
#         # current_point[1] = best_point[1]
#         # current_point[2] = best_point[2]
#     path_list = [list(t) for t in path]
#     # Return the path
#     return path_list

# def path_smoothing(path, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001):
#     new_path = deepcopy(path)
#     change = tolerance
#     while change >= tolerance:
#         change = 0.0
#         for i in range(1, len(path) - 1):
#             for j in range(len(path[0])):
#                 aux = new_path[i][j]
#                 new_path[i][j] += weight_data * (path[i][j] - new_path[i][j])
#                 new_path[i][j] += weight_smooth * (new_path[i - 1][j] + new_path[i + 1][j] - (2.0 * new_path[i][j]))
#                 change += abs(aux - new_path[i][j])
#     return new_path

# # Generate the path
# path = generate_path(start, goal, step_size, max_turning_angle)
# print("path:", path)
# # Smooth the path
# smooth_path = path_smoothing(path, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001)
# print("smooth_path:", smooth_path)

import numpy as np
import matplotlib.pyplot as plt
import csv
path = []
with open("/home/gem/demo_ws/src/vehicle_drivers/gem_gnss_control_/waypoints/smoothpath.csv", newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        row = [float(value) for value in row]
        path.append(row)


# Define the cylinder obstacles
# circles = [((2, 3), 1), ((4, 2), 1)]
circles = [((25.231465245116457, -4.923283873156118), 2), ((28.331879857381683, -10.412998011077805), 2)]
        # self.c1_x, self.c1_y, self.r1 = 25.231465245116457, -4.923283873156118, 2
        # self.c2_x, self.c2_y, self.r2 = 28.331879857381683, -10.412998011077805, 2
# Create the plot and axis objects
fig, ax = plt.subplots()

for i in range(len(path)-1):
    x1, y1 = path[i][:2]
    x2, y2 = path[i+1][:2]
    ax.plot([x1, x2], [y1, y2], color='blue')
# for i in range(len(smooth_path)-1):
#     x1, y1 = smooth_path[i][:2]
#     x2, y2 = smooth_path[i+1][:2]
#     ax.plot([x1, x2], [y1, y2], color='green')

# ax.plot(start[0], start[1], color = 'yellow')
# ax.scatter(start[0], start[1], color='red', s=100)
# ax.scatter(goal[0], goal[1], color='red', s=100)
# ax.scatter(path[:,0], path[:,1], color='blue')
# ax.scatter(smooth_path[:,0], smooth_path[:,1], color='yellow')
# Plot the circles
for circle in circles:
    (x, y), r = circle
    circle = plt.Circle((x, y), r, color='black', fill=False)
    ax.add_patch(circle)

# Set the axis limits
# ax.set_xlim([0, 6])
# ax.set_ylim([0, 6])

# Display the plot
plt.show()


