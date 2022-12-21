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
<p align="left">
  <img src = "https://user-images.githubusercontent.com/89351094/208841886-9837271e-000d-435a-bf4c-c2c71c50ab60.gif"/>
 </p>
 
 <p align="right">
  <img src = "https://user-images.githubusercontent.com/89351094/208841986-7135d067-0b4a-40cc-8b8c-950c5020ee36.gif"/>
 </p>!



For part B:
Some examples of random environments for important cases (both door closed, both open, one open one closed) are shown below.
 
<p align="left">
  <img src = "https://user-images.githubusercontent.com/89351094/208842088-de9b4798-49c1-4076-9ed6-dc5e6391feb9.gif"/>
</p>

<p align="right">
  <img src = "https://user-images.githubusercontent.com/89351094/208842152-6b3e55f0-d59d-4613-b5d5-834a5cd6bad5.gif"/>
</p>


## Details of code files (This is a class homework: ECE276B SP22 PR1)

There are two '.py' files:

1) utils.py -- This needs to be run first as its functions are utilized in the main file
		This file includes the Label Correcting Algorithm for both parts (named: dp)
		along with some helper functions.

2) doorkey.py -- This is the main file and is executing part A and B
		The Label Correcting Algorithm of utils.py is used in
			this file's function "doorkey_problem"

		Just uncomment the part which need to be run (A or B)
			and comment the other after "if __name__ == '__main__':"

