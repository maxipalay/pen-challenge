import math

def get_robot_pos(camera_pos, average_meas, robot_home):
    # translate the camera position to the robot position
    # given an xyz coordinate in the camera frame, 
    # we want to know the zyx coordinates in the robot frame
    camera_x = camera_pos[0]
    camera_y = camera_pos[1]
    camera_z = camera_pos[2]
    #robot_x = -camera_x + (average_meas[0] + robot_home[0])
    robot_x = camera_x + (-average_meas[0] + robot_home[0])
    robot_y = camera_z *-1 + (average_meas[2] - robot_home[1])
    robot_y = -robot_y
    #robot_z = camera_y *-1 + (-average_meas[1] + robot_home[2])
    robot_z = -camera_y + (average_meas[1] + robot_home[2])
    return [robot_x, robot_y, robot_z]

def calculate_waist_angle(x, y):
    return math.atan2(y, x)