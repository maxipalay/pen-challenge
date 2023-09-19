from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr

class Robot(InterbotixManipulatorXS):
    """
    Robot: wrapper for the InterbotixManipulatorXS class.
    """
    def __init__(self) -> None:
        # set some default parameters
        super().__init__("px100", "arm", "gripper", moving_time = 1.0, accel_time = 0.1, gripper_pressure = 1.0)

    def open_gripper(self):
        self.gripper.release()

    def close_gripper(self):
        self.gripper.grasp()

    def go_home(self):
        self.arm.go_to_home_pose()

    def go_sleep(self):
        self.arm.go_to_sleep_pose()

    def get_end_effector_pos(self):
        # capture the joint positions
        self.arm.capture_joint_positions()
        # get the joints positions
        joints = self.arm.get_joint_commands()
        # perform the forward kinematics to get the end effector position
        T = mr.FKinSpace(self.arm.robot_des.M, self.arm.robot_des.Slist, joints)
        # get the rotation matrix and the displacement
        [R, pos] = mr.TransToRp(T)
        return pos

    def move_to(self, x, y, z):
        self.arm.set_ee_pose_components(x, y, z, blocking=True, moving_time=1.0, accel_time=0.1)

    def move_joint(self, joint, angle):
        self.arm.set_single_joint_position(joint, angle)