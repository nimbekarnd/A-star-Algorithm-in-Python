"""
 *  MIT License
 *
 *  Copyright (c) 2019 Nipur, Markose Jacob
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
"""

# header files
from utils_project3 import *
import sys

startRow = int(input("Enter the row coordinate for start node (between 1 and 200) : "))
startCol = int(input("Enter the column coordinate for start node (between 1 and 300) : "))
startOrientation = int(input("Enter the Orientation of the robot (0/30/60/90/120/150/180/210/240/270/300/330) : "))
# goalRow = int(input("Enter the row coordinate for goal node (between 1 and 200) : "))
goalRow = 185
# goalCol = int(input("Enter the column coordinate for goal node (between 1 and 300) : "))
goalCol = 285
# radius = int(input("Enter the radius for the robot : "))
radius = 1
# clearance = int(input("Enter the clearance for the robot : "))
clearance = 1

# take start and goal node as input
start = (startRow, startCol, startOrientation)
goal = (goalRow, goalCol)
astar = Astar(start, goal, clearance, radius)

if(astar.IsValid(start[0], start[1])):
    if(astar.IsValid(goal[0], goal[1])):
        if(astar.IsObstacle(start[0],start[1]) == False):
            if(astar.IsObstacle(goal[0], goal[1]) == False):
                (explored_states, backtrack_states, distance_from_start_to_goal) = astar.Astar()
                astar.animate(explored_states, backtrack_states, "./astar_rigid.avi")
                # print optimal path found or not
                if(distance_from_start_to_goal == float('inf')):
                    print("\nNo optimal path found.")
                else:
                    print("\nOptimal path found. Distance is " + str(distance_from_start_to_goal))
            else:
                print("The entered goal node is an obstacle ")
                print("Please check README.md file for running Astar_rigid.py file.")
        else:
            print("The entered initial node is an obstacle ")
            print("Please check README.md file for running Astar_rigid.py file.")
    else:
        print("The entered goal node outside the map ")
        print("Please check README.md file for running Astar_rigid.py file.")
else:
    print("The entered initial node is outside the map ")
    print("Please check README.md file for running Astar_rigid.py file.")
