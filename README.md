# Pen challenge

Before starting the MS in Robotics program at Northwestern University, students participate in a Hackathon where they are presented with several challenges. The pen challenge is one of them.

Using an Intel Realsense D435 and an Interbotix PX100 arm, students have to locate a Purple pen in space, and get the arm to grab it.

## Files & structure
- `/dev` contains python code which was used for developing functionalities and performing tests
- `/examples` contains example source code which was used as starting point
- `robot_arm.py` contains a wrapper class for the Interbotix robot arm functions used
- `pen_tracker.py` is the code that runs the pen detection, interacting with the Realsense camera and displaying the results
- `kinematics.py` contains functions that aid in the calculations needed to get the robot to grab the pen, mainly conversion between frames
- `main.py` is the main code that orchestrates the flow

## Notes

This was implemented using Python's multiprocessing package to get a fluent view of the tracking while interacting with the robot.

When tested as a single process, the tracking preview was stuttery.
