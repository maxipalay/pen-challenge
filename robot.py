from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS, InterbotixRobotXSCore
# The robot object is what you use to control the robot

robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'

robot.core.robot_torque_enable("group","arm",False)

input("guide robot to pen with your hand, when done, hit Enter")

import modern_robotics as mr
robot.core.robot_torque_enable("group","arm",True)
robot.arm.capture_joint_positions()

joints = robot.arm.get_joint_commands()
T = mr.FKinSpace(robot.arm.robot_des.M, robot.arm.robot_des.Slist, joints)
[R, p] = mr.TransToRp(T) # get the rotation matrix and the displacement

print(f"pen position: {p}")

'''# Let the user select the position
while mode != 'q':
    mode=input("[h]ome, [s]leep, [c] close, [o] open, [q]uit ")
    if mode == "h":
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()
    elif mode == "c":
        robot.gripper.grasp()
    elif mode == "o":
        robot.gripper.release()
    '''

# move the robot to xyz in world coordinates -> robot.arm.set_ee_pose_components(0.1, -0.1, 0.15)
# grab object robot.gripper.grasp()
# release object robot.gripper.release()

