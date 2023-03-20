'''
****************************Project-3 | Phase-1*****************************

Course  : ENPM661 - Planning
Team    : Krishna Hundekari (119239049) & Shail Shah (119340547)
UIDs    : krishnah & sshah115
github link : https://github.com/sshah115/path-planning-A-star-algorithm.git

****************************************************************************
'''

# Importing required modules/libraries
from queue import PriorityQueue
import matplotlib.pyplot as plt
import numpy as np
import copy
import matplotlib.patches as patch
import time
from math import dist
import cv2 as cv

# Drawing polyline shapes
def polyShap(img, polyPts, color, type):
    polyPts = polyPts.reshape((-1,1,2))
    if type == "Obstacle":
        cv.fillPoly(img,[polyPts],color)
    else:
        cv.polylines(img,[polyPts],True,color, thickness = obsClea)

# Function to generate map with obstacles using cv functions
def genMap():

    # Generating a black background arena
    arena = np.zeros((250,600, 3), dtype="uint8")
    
    # Defining colors
    white = (255,255,255)
    blue = (255, 0, 0)
    orange = (0, 165, 255)
    red = (0, 0, 255)

    # Drawing rectangle obstacles and image border
    cv.rectangle(arena, (100,0), (150,100), blue, -1)
    cv.rectangle(arena, (100,0), (150,100), white, obsClea)
    cv.rectangle(arena, (100,150), (150,250), blue, -1)
    cv.rectangle(arena, (100,150), (150, 250), white, obsClea)
    cv.rectangle(arena, (-1, -1), (601, 251), white, obsClea)

    # Drawing all polygon shaped obstacles
    hexPts = np.array([[300,50], [365, 87.5], [365,162.5], 
                       [300,200], [235, 162.5], [235, 87.5]], np.int32)
    polyShap(arena, hexPts, orange, "Obstacle")
    hexBorPts = np.array([[300,45], [369, 87.5], [369,162.5], 
                          [300,205], [235, 162.5], [235, 87.5]], np.int32)
    polyShap(arena, hexBorPts, white, "Border")
    triPts = np.array([[460, 25], [460, 225], [510,125]], np.int32)
    polyShap(arena, triPts, red, "Obstacle")
    triBorPts = np.array([[456, 20], [456, 230], [514,125]], np.int32)
    polyShap(arena, triBorPts, white, "Border")

    

    return cv.resize(arena, (int(600/thresh_for_grid),int(250/thresh_for_grid)))


# Checking coordinates if it lies in obstacle space
def obstacle_check(node):
    # This function will check if the points are okay, on border or
    # inside obstacle
    xPt = int(node[1]/thresh_for_grid)

    yPt = int(node[0]/thresh_for_grid)
  
    if canvas[xPt, yPt].any() == np.array([0, 0, 0]).all():
        status = False
    elif canvas[xPt, yPt].all() == np.array([255, 255, 255]).all():
        status = True
    else:
        status = True     

    return status


def check_goal(current, goal_allowance):
    
    dt = dist((current[0], current[1]), (goal_x, goal_y))   

    if goal_allowance in range(1,4):
        compare_with_this = 1.5
    elif goal_allowance in range(4,7):
        compare_with_this = 3
    else:
        compare_with_this = 4
          
    if dt < compare_with_this:
        return True
    else:
        return False
    

def round_thresh(val):

    if val % 0.5 != 0:
        val = np.round(val/thresh_for_grid)*thresh_for_grid
    return val

#threshold for the grid
thresh_for_grid = 0.5

# Grid of the map

v= np.zeros((1200, 500, 12))


obstacle_real= int(round_thresh(float(input("\nEnter obstacle clearance: \n"))))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")

radius_robot = int(round_thresh(float(input("\nEnter Robot radius: \n"))))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")

#Total clearance taking into account robot radius and clearance.
obsClea = obstacle_real + radius_robot


#Generating map
canvas = genMap()

canvas = cv.flip(canvas, 0)

thresh = 0.5
deg_thresh = 30
#Getting home position
home_x = round_thresh(float(input("Hey!! Where to start? Please enter home 'x' coordinate:  \n")))
home_y = round_thresh(float(input("\nPlease enter home 'y' coordinate: \n")))
home_loc = (home_x, home_y)
while obstacle_check(home_loc):
    print("\nThe entered value is in the obstacle. Please enter new values\n")
    home_x = round_thresh(float(input("\nHey!! Where to start? Please enter home 'x' coordinate:  \n")))
    home_y = round_thresh(float(input("\nPlease enter home 'y' coordinate: \n")))
    home_loc = (home_x, home_y)
home_theta = int(input("\nGive home orientation:  \n"))
while home_theta % deg_thresh != 0:
    print("\nPleaes entere theta as multiple of 30 degrees.Please enter new values\n")
    home_theta = int(input("\nGive home orientation:  \n"))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")

#Getting Goal Position

goal_x= round_thresh(float(input("\nNow give the goal 'x' coordinate \n")))
goal_y = round_thresh(float(input("\nNow give the goal 'y' coordinate \n")))
goal_loc = (goal_x, goal_y)
while obstacle_check(goal_loc):
    print("\nThe entered value is in the obstacle. Please enter new values\n")
    goal_x= round_thresh(float(input("\nNow give the goal 'x' coordinate \n")))
    goal_y = round_thresh(float(input("\nNow give the goal 'y' coordinate \n")))
    goal_loc = (goal_x, goal_y)
    
goal_theta = int(input("\nGive goal orientation:  \n"))
while goal_theta % deg_thresh != 0:
    print("\nPleaes entere theta as multiple of 30 degrees.Please enter new values\n")
    goal_theta = int(input("\nGive goal orientation:  \n"))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")

L_step_size = round_thresh(float(input("\nEnter Step size of the robot(1 <= L <= 10): \n")))
while L_step_size not in range(1,11):
    print("\nThe entered the step size between 1 to 10. \n")
    L_step_size = round_thresh(float(input("\nEnter Step size of the robot(1 <= L <= 10): \n")))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")  

print("\nBe patient!!! I am coputing the shortest path!! \n")

x_visited = [] 
y_visited = []

start = time.time()

start_pose = (home_x, home_y, home_theta)
goal_pose = (goal_x, goal_y, goal_theta)

c2c = 0
parent_pose = None
total_cost = 0
adam_node = (total_cost,c2c,(parent_pose),(start_pose))
open_list = PriorityQueue()
open_list.put(adam_node)

closed_list = {} 

def two_time_theta_up(node):

    node_in_action = copy.deepcopy(node)
    c2c_moved = node_in_action[1] + 1
    current_parent = node_in_action[3]

    updated_theta = ((node_in_action[3][2]) + 60) % 360
    updated_x = round_thresh(node_in_action[3][0]+(L_step_size*np.cos(np.radians(updated_theta))))
    updated_y = round_thresh(node_in_action[3][1]+(L_step_size*np.sin(np.radians(updated_theta))))
      
    c2g = dist((updated_x, updated_y), (goal_x, goal_y))
    total_cost = c2c_moved + c2g

    current_child = (updated_x, updated_y, updated_theta)

    if v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] == 0:
        v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] = 1
        if obstacle_check((current_child[0],current_child[1])) == False:
            if current_child not in closed_list:
                graph_dict[current_parent].append(current_child)
                for i in range(0,(open_list.qsize())):
                    if open_list.queue[i][3] == current_child and open_list.queue[i][0] > total_cost:
                        open_list.queue[i][2] == current_parent
                open_list.put((total_cost, c2c_moved, current_parent, current_child))

def one_time_theta_up(node):

    node_in_action = copy.deepcopy(node)
    c2c_moved = node_in_action[1] + 1
    current_parent = node_in_action[3]

    updated_theta = ((node_in_action[3][2]) + 30)  % 360
    updated_x = round_thresh(node_in_action[3][0]+(L_step_size*np.cos(np.radians(updated_theta))))
    updated_y = round_thresh(node_in_action[3][1]+(L_step_size*np.sin(np.radians(updated_theta))))
      
    c2g = dist((updated_x, updated_y), (goal_x, goal_y))
    total_cost = c2c_moved + c2g

    current_child = (updated_x, updated_y, updated_theta)

    if v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] == 0:
        v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] = 1
        if obstacle_check((current_child[0],current_child[1])) == False:
            if current_child not in closed_list:
                graph_dict[current_parent].append(current_child)
                for i in range(0,(open_list.qsize())):
                    if open_list.queue[i][3] == current_child and open_list.queue[i][0] > total_cost:
                        open_list.queue[i][2] == current_parent
                open_list.put((total_cost, c2c_moved, current_parent, current_child))

def straight(node):

    node_in_action = copy.deepcopy(node)
    c2c_moved = node_in_action[1] + 1
    current_parent = node_in_action[3]

    updated_theta = ((node_in_action[3][2]))  % 360
    updated_x = round_thresh(node_in_action[3][0]+(L_step_size*np.cos(np.radians(updated_theta))))
    updated_y = round_thresh(node_in_action[3][1]+(L_step_size*np.sin(np.radians(updated_theta))))
      
    c2g = dist((updated_x, updated_y), (goal_x, goal_y))
    total_cost = c2c_moved + c2g

    current_child = (updated_x, updated_y, updated_theta)

    if v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] == 0:
        v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] = 1
        if obstacle_check((current_child[0],current_child[1])) == False:
            if current_child not in closed_list:
                graph_dict[current_parent].append(current_child)
                for i in range(0,(open_list.qsize())):
                    if open_list.queue[i][3] == current_child and open_list.queue[i][0] > total_cost:
                        open_list.queue[i][2] == current_parent
                open_list.put((total_cost, c2c_moved, current_parent, current_child))

def one_time_theta_down(node):

    node_in_action = copy.deepcopy(node)
    c2c_moved = node_in_action[1] + 1
    current_parent = node_in_action[3]

    updated_theta = ((node_in_action[3][2])- 30) %360
    updated_x = round_thresh(node_in_action[3][0]+(L_step_size*np.cos(np.radians(updated_theta))))
    updated_y = round_thresh(node_in_action[3][1]+(L_step_size*np.sin(np.radians(updated_theta))))
      
    c2g = dist((updated_x, updated_y), (goal_x, goal_y))
    total_cost = c2c_moved + c2g

    current_child = (updated_x, updated_y, updated_theta)

    if v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] == 0:
        v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] = 1
        if obstacle_check((current_child[0],current_child[1])) == False:
            if current_child not in closed_list:
                graph_dict[current_parent].append(current_child)
                for i in range(0,(open_list.qsize())):
                    if open_list.queue[i][3] == current_child and open_list.queue[i][0] > total_cost:
                        open_list.queue[i][2] == current_parent
                open_list.put((total_cost, c2c_moved, current_parent, current_child))

def two_time_theta_down(node):

    node_in_action = copy.deepcopy(node)
    c2c_moved = node_in_action[1] + 1
    current_parent = node_in_action[3]

    updated_theta = ((node_in_action[3][2]) - 60) %360
    
    updated_x = round_thresh(node_in_action[3][0]+(L_step_size*np.cos(np.radians(updated_theta))))
    updated_y = round_thresh(node_in_action[3][1]+(L_step_size*np.sin(np.radians(updated_theta))))
      
    c2g = dist((updated_x, updated_y), (goal_x, goal_y))
    total_cost = c2c_moved + c2g

    current_child = (updated_x, updated_y, updated_theta)

    if v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] == 0:
        v[int(updated_x/thresh)][int(updated_y/thresh)][int(updated_theta/deg_thresh)] = 1
        if obstacle_check((current_child[0],current_child[1])) == False:
            if current_child not in closed_list:
                graph_dict[current_parent].append(current_child)
                for i in range(0,(open_list.qsize())):
                    if open_list.queue[i][3] == current_child and open_list.queue[i][0] > total_cost:
                        open_list.queue[i][2] == current_parent
                open_list.put((total_cost, c2c_moved, current_parent, current_child))

graph_dict = {}

while True:
    
    current_node = open_list.get() 
    if current_node[3] in closed_list: 
        continue

    x_visited.append(current_node[3][0]) 
    y_visited.append(current_node[3][1])

    closed_list[current_node[3]] = (current_node[2]) 
    
    graph_dict[current_node[3]] = []

    if check_goal(current_node[3], L_step_size): 
        goal_pose1 = current_node[3]
        print("Mission Accomplished...Goal Reached")
        print(current_node)
        break
        
    else:

        two_time_theta_up(current_node)
        one_time_theta_up(current_node)
        straight(current_node)
        one_time_theta_down(current_node)
        two_time_theta_down(current_node)        

#backtracing the shortest path
shortest_planned_path=[]
check_this_node = goal_pose1
while check_this_node != start_pose:
    shortest_planned_path.append(check_this_node)
    check_this_node = closed_list[check_this_node]
shortest_planned_path.append(start_pose)
shortest_planned_path.reverse()

x_shortest = [] 
y_shortest = []
for i in range(len(shortest_planned_path)):
    x_shortest.append(shortest_planned_path[i][0])
    y_shortest.append(shortest_planned_path[i][1])

end = time.time()
print(f'Time needed for the algorithm: {end - start}\n')
print('\n')

fig, ax = plt.subplots(figsize=(6,2.5))

bottom_rectangle = patch.Rectangle((100, 150), 50, 100, linewidth=1, edgecolor='g', facecolor='g')
top_rectangle = patch.Rectangle((100, 0), 50, 100, linewidth=1, edgecolor='g', facecolor='g')
hexagon_obstacle = patch.RegularPolygon((300, 125), 6, 75, linewidth=1, edgecolor='g', facecolor='g')
tri_obstacle = patch.Polygon([(460, 25), (460, 225), (510, 125)], linewidth=1, edgecolor='g', facecolor='g')

ax.add_patch(bottom_rectangle)
ax.add_patch(top_rectangle)
ax.add_patch(hexagon_obstacle)
ax.add_patch(tri_obstacle)

#Plotting the path
plt.xlabel('X Axis')
plt.ylabel('Y Axis')
plt.title("Visualising Exploration")
plt.axis([0 , 600 , 0 ,250])

for key, value in graph_dict.items() :
    for index in value:
        plt.plot([key[0], index[0]],[key[1], index[1]], c='red')
        plt.pause(0.00005)

plt.title("Shortest Path traced by Point Robot")
for i in range(len(x_shortest)-1):
    plt.plot((x_shortest[i],x_shortest[i+1]) , (y_shortest[i],y_shortest[i+1]) , c='yellow')
    plt.pause(0.00005)

plt.show()
