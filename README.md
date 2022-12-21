# Dynamic-Programming-in-motioan-planning: Autonomous navigation in a ‘Door &amp; Key’ environment

<p align="justify">
This project focuses on autonomous navigation in a Door & Key environment, shown in Fig. 1. The objective is to get our agent (red triangle) to the goal location (green square). The environment may contain a door which blocks the way to the goal. If the door is closed, the agent needs to pick up a key to unlock the door. The agent has three regular actions, move forward (MF), turn left (TL), and turn right (TR), and two special actions, pick up key (PK) and unlock door (UD). Taking any of these five actions costs energy (positive cost)
</p>

Designing and implementing a Dynamic Programming algorithm that minimizes the cost of reaching the goal in two scenarios:
(a) ”Known Map”: you should compute a control policy on each of the 7 environments provided in the starter code and evaluate its performance on the same environment that it was computed for. Further description of the 7 environments and their functionality is provided in the accompanying starter code and README file.
(b) ”Random Map”: you should compute a single control policy, whose performance may be evaluated on any of the 36 random 8 × 8 environments. The size of the grid in the random maps is 8 × 8 and the perimeter is surrounded by walls.
</p>

## Project Report
[Orish Jindal, 'Autonomous navigation in a ‘Door & Key’ environment](https://github.com/ojindal/Dynamic-Programming-in-motioan-planning/blob/main/Orish%20PR1%20report.pdf)

For part A:
The optimal control sequences that were obtained for some of the given environments of part A, were used to plot the gif showing movement of the robot based on the given environment.
<p align="center">
  <img src = "https://user-images.githubusercontent.com/89351094/208841886-9837271e-000d-435a-bf4c-c2c71c50ab60.gif"/>
 </p>
 
 <p align="center">
  <img src = "https://user-images.githubusercontent.com/89351094/208841986-7135d067-0b4a-40cc-8b8c-950c5020ee36.gif"/>
 </p>!



For part B:
Some examples of random environments for important cases (both door closed, both open, one open one closed) are shown below.
 
<p align="center">
  <img src = "https://user-images.githubusercontent.com/89351094/208842088-de9b4798-49c1-4076-9ed6-dc5e6391feb9.gif"/>
</p>

<p align="center">
  <img src = "https://user-images.githubusercontent.com/89351094/208842152-6b3e55f0-d59d-4613-b5d5-834a5cd6bad5.gif"/>
</p>


## Details of code files (This is a class homework: ECE276B SP22 PR2)

Run the main.py
Current setting: it will plot the final position as well as path for the map 3.
To run other maps, change map name in two places: 

1) test_map
2) argument of 'plotting(path)'. (give path location of the map that you want to run)

#### Folder attached: Results
Contains the results

Other files attached:
#### 1. robotplanner.py

This file contains the algorythm RTAAstar which is used accordingly in the function robotplanner.
Also contains other small helper functions needed for the algorith.

#### 2. targetplanner.py
This file is not modified in any way (as instructed).

#### 3. main.py

This file contains test functions.

This file is almost same as given.
Changes:

1) from robotplanner import *
2) from path import * 
3) Arrays to store the states at each timestamp in order to use the data for plotting the path later
4) tqdm to track the time and number of iterations in real time
5) calling the 'plotting(path)' function at the end that plots the complete path (need to feed the map location)

#### 4. path.py

This contains 'plotting(path)' function that plots the stored path

