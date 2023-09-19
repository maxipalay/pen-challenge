import math

def get_robot_pos(camera_pos, ref_wrt_camera, ref_wrt_robot):
    """
    Translate between camera frame and robot frame.
    
    We're getting the pen's position wrt camera frame. This 
    function translates the pen's position to the robot's 
    base frame.

    param camera_pos: the pen's position in meters [x,y,z] in the camera's frame
    param ref_wrt_camera: one reference point [x,y,z] wrt the camera frame
    param ref_wrt_robot: one reference point [x,y,z] wrt the robot base frame

    Note: this function assumes the camera is placed 90 degrees offset from the
    robot, at the robot's right hand side.
    """
    # we're expanding the array into variables for ease of readiness.
    camera_x, camera_y, camera_z = camera_pos[0], camera_pos[1], camera_pos[2]
    ref_camera_x, ref_camera_y, ref_camera_z = ref_wrt_camera[0], ref_wrt_camera[1], ref_wrt_camera[2]
    ref_robot_x, ref_robot_y, ref_robot_z = ref_wrt_robot[0], ref_wrt_robot[1], ref_wrt_robot[2]

    # calculations
    robot_x =  camera_x + (-ref_camera_x + ref_robot_x)
    robot_y =  camera_z + (-ref_camera_z + ref_robot_y)
    robot_z = -camera_y + ( ref_camera_y + ref_robot_z)

    return [robot_x, robot_y, robot_z]

def calculate_waist_angle(x, y):
    """
    Calculate the waist angle so the robot faces a the direction of a point.

    param x: the point's x coordinate wrt robot base
    param y: the point's y coordinate wrt robot base
    """
    return math.atan2(y, x)