from multiprocessing import Process, Lock, Array
from time import sleep
import numpy as np

from robot_arm import Robot
import kinematics
from pen_tracker import vision_thread


def main():
    lock = Lock()
    camera_pos = Array('d', [0.0,0.0,0.0])
    # although array provides a lock, we'll be using multiprocessing's lock as well be iterating over
    # the arrays elements and want to guarantee we get all the values of the array at a single state

    # launch the vision process
    process = Process(target = vision_thread, args=(camera_pos,lock))
    process.start()

    # wait for a bit
    sleep(5)

    # instance the robot object
    robot = Robot()

    # go home
    robot.go_home()

    # open gripper
    robot.open_gripper()

    # request the user to place the pen at the center of the gripper,
    # we will use the detected pen centroid's as a reference point for
    # conversion between frames
    input("place pen on center of tool, hold still and then hit Enter")

    # get the robot's position wrt robot base frame
    robot_home = robot.get_end_effector_pos()

    # send the robot to sleep position so we get a clear view of the pen
    robot.go_sleep()

    print("hold the pen still, measuring position...")
    sleep(1)

    # capture the pen's location with the camera
    # we're averaging 50 measurements
    measurements = []
    while len(measurements)<50:
        measurements.append(np.asarray(camera_pos))
        sleep(0.05)
            
    average_meas = np.array(measurements).mean(axis=0)

    print("done! robot will grab pen!")

    while True:
        # get the robot's desired position
        with lock:
            robot_pos = kinematics.get_robot_pos(camera_pos, average_meas, robot_home)
        
        #print(robot_pos)
        
        # calculate the robot's waist angle so that it faces the pen.
        # if we dont do this, the robot might collide with the pen
        # when going for it.
        theta = kinematics.calculate_waist_angle(robot_pos[0],robot_pos[1])
        
        # move the waist to the calculated angle
        robot.move_joint('waist', theta)

        # move the end effector up to the pen
        robot.move_to(robot_pos[0], robot_pos[1], robot_pos[2])

        sleep(1)
        robot.close_gripper()
        sleep(1)
        robot.go_sleep()
        sleep(1)
        robot.go_home()
        robot.open_gripper()
        robot.go_sleep()
        input("press enter when pen in position")
        sleep(1)

if __name__ == "__main__":
    main()

