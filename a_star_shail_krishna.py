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
    cv.rectangle(arena, (4, 4), (596, 246), white, obsClea)

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
