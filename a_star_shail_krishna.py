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
        print("Processing!!!")
        status = False
    elif canvas[xPt, yPt].all() == np.array([255, 255, 255]).all():
        print("The goal coordinate is on border")
        status = True
    else:
        print("The goal coordinate is inside obstacle")
        status = True       

    return status

#threshold for the grid
thresh_for_grid = 0.5

# Grid of the map

v= np.zeros((1200, 500, 12))

#Obstacle inputs
obstacle_real = 10
radius_robot = 10

obsClea = obstacle_real + radius_robot

#Generating map
canvas = genMap()

canvas = cv.flip(canvas, 0)
