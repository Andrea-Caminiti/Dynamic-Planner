Repository to store the code for the Robot Programming Final Project at Sapienza Università di Roma, accademic year 2024/2025

Commit 1 - Implemented planner

Implemented planner that once received a laser scan of the surroundings of the robot, computes the local distance map, compares it with its current global map
and plans a path from start to goal using A*. 

To be tested with dummy laser scan -- Builds ok

Commit 2 - Implemented laser scanner simulation and fixed bug in the A* loop.

Implemented a package in Python to handle generation of a dummy laser scanner message. 
Fixed a bug in the A* loop that blocked the entrance in the loop due to a wrong comparison (was using > instead of <).

To be implemented: 
    Publishing of obstacles so that they can be visualized through rviz
    Drawing of obstacles directly on the map

Commit 3 - Updated Laser Scanner to receive Interactive Marker position

Updated the laser scanner so that it receives the marker position, understands that there is now an obstacle there and publish a new scan.
