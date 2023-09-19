from multiprocessing import Process, Lock, Array
from time import sleep
import numpy as np
import math

from robot_arm import Robot
import kinematics
from pen_tracker import vision_thread

lock = Lock()
camera_pos = Array('d', [0.0,0.0,0.0])
# although array provides a lock, we'll be using multiprocessing's lock as well be iterating over
# the arrays elements and want to guarantee we get all the values of the array at a single state

# launch the vision process
process = Process(target = vision_thread, args=(camera_pos,lock))
process.start()

# wait for a bit
sleep(5)

# instance the manipulator object
robot = Robot()

# go home
robot.go_home()

# close gripper
robot.open_gripper()

input("place pen on center of tool, hold still and then hit Enter")

robot_home = robot.get_end_effector_pos()

robot.go_sleep()
print(f"robot home: {robot_home}")
sleep(1)
# capture the pen's location with the camera

measurements = []
while len(measurements)<50:
    measurements.append(np.asarray(camera_pos))
    sleep(0.05)
        
average_meas = np.array(measurements).mean(axis=0)

print(f"average measurement of pen {average_meas}")

while True:
    with lock:
        robot_pos = kinematics.get_robot_pos(camera_pos, average_meas, robot_home)
    print(robot_pos)
    #input("press enter to grab pen")
    theta = kinematics.calculate_waist_angle(robot_pos[0],robot_pos[1])
    print(theta)
    robot.move_joint('waist', theta)
    robot.move_to(robot_pos[0], robot_pos[1], robot_pos[2])
    sleep(1)
    robot.close_gripper()
    sleep(1)
    robot.go_sleep()
    #input("press enter to go home and release")
    sleep(1)
    robot.go_home()
    robot.open_gripper()
    robot.go_sleep()
    input("press enter when pen in position")
    sleep(2)

