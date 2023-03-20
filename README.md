# Project - 3 | Phase - 1
```
Course  : ENPM661 - Planning
github link : https://github.com/sshah115/path-planning-A-star-algorithm.git
```
Team: 

|Name|UID|Directory ID|
|:---:|:---:|:---:|
|Krishna Hundekari|119239049|krishnah|
|Shail Shah|119340547|sshah115|

# 1 A* Algorithm

Implementation A* algorithm for a mobile Robot

Traverse through map from user defined start node to goal node using path computed using A* algorithm along with backtracking and visualization.

## Python File 
```
It is stored in the directory proj3_shail_krishna.zip / a_star_shail_krishna.py
```
## Running the code
There are four ways of running a python script which are as follows:

 - You may run it in your operating system's terminal. For e.g., In windows - cmd.
 - Python interactive mode.
 - Integrated Development Environment (IDE) like VSC.
 - Opening the script file from folder

First check the version of python installed in your system by running following command:

*python --version*

If it yields result like this one:

*Python 3.8.10*

Then you may run script in terminal by typing following line in the directory it is located at:

*python3 a_star_shail_krishna.py*

## Dependencies

import following modules/library for the script to run correctly: 

*import  numpy as np*  			

*import cv2 as cv*  								

*from queue import PriorityQueue*  								

*import matplotlib.pyplot as plt*  								

*import time*  	

*import copy*

*import matplotlib.patches as patch*

*from math import dist*

## User input

Defining user defined following attributes in below described format:
```
Enter obstacle clearance:
```
Here user has to enter value to be bloated around obstacles.

```
Enter Robot radius:
```
Here user has to enter value of robot's radius for obstacle avoidance.

```
Note: Make sure the clearances i.e., robot radius and obstacle clearance doesn't add up to block the right end of the map since it will keep on computing childrens through action set.
```

```
Hey!! Where to start? Please enter home 'x' coordinate:
```
Here user has to enter value of home position's x coordinate.

Similarly, user has to provide complete home pose and goal pose i.e., x, y & orientation (Theta in degrees) to move foraward.

Next, the user will be prompted with following last input:
```
Enter Step size of the robot(1 <= L <= 10):
```
Here step size needs to be defined which will decide the action step in 5 different directions namely far right, right, straight, left, far left in increments of 30 degrees.

Reasonable threshold for checking duplicates is defined as shown in below table:

|Sr No|Step range|Duplicates threshold|
|:---:|:---:|:---:|
|1|1-3|1.5|
|2|4-7|2|
|3|7-10|3|

Kindly provide the input from your terminal and in the format it is explained above for correct implementation

The output file of video will be generated at the directory path your terminal is located at.

## Result

Examples of obstacle boundaries and videos of successful run are uploaded at following link:

*https://drive.google.com/drive/folders/11Lv9wgRlpY8Vu5_WhdYgizaq73AP6PNi?usp=share_link*
