# https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

p = profile.get_stream(rs.stream.color)
intrinsics = p.as_video_stream_profile().get_intrinsics()

def threshol_hue(matrix, matrix2, threshold):
    # threshold for the hue value, like a BPF
    # we want to show only the pixels withing a range
    # threshold - 10 < matrix < threshold + 10
    global tolerance
    return np.where((matrix > threshold - tolerance) & (matrix < threshold + tolerance), matrix2, 0)

def threshol_sat(matrix, matrix2, threshold):
    # threshold for saturation, like a HPF
    return np.where(matrix > threshold, matrix2, 0)

def biggest_contour(contours):
    # returns the biggest contour in a list of contours
    largest_area = 0
    largest_area_cnt = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > largest_area:
            largest_area = area
            largest_area_cnt = contour
    return largest_area_cnt

# Streaming loop
try:
    title_window = 'hey'
    threshold_hue = 171
    threshold_sat = 135
    tolerance = 20

    hue_slider_max = 180
    sat_slider_max = 255
    tol_slider_max = 50
    
    trackbar_name_hue = 'Hue x %d' % hue_slider_max
    trackbar_name_sat = 'Sat x %d' % sat_slider_max
    trackbar_name_tol = 'Tol x %d' % tol_slider_max
   
    def mod_threshold_hue(val):
        global threshold_hue
        threshold_hue = val

    def mod_threshold_sat(val):
        global threshold_sat
        threshold_sat = val

    def mod_tol(val):
        global tolerance
        tolerance = val
    
    cv2.namedWindow(title_window, cv2.WINDOW_NORMAL)
    cv2.namedWindow(title_window, cv2.WINDOW_NORMAL)
    cv2.createTrackbar(trackbar_name_hue, title_window , threshold_hue, hue_slider_max, mod_threshold_hue)
    cv2.createTrackbar(trackbar_name_sat, title_window , threshold_sat, sat_slider_max, mod_threshold_sat)
    cv2.createTrackbar(trackbar_name_tol, title_window , tolerance, tol_slider_max, mod_tol)

    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue
        
        # get color image
        color_image = np.asanyarray(color_frame.get_data())
        # apply gaussian filter
        color_image = cv2.GaussianBlur(color_image,(5,5),0)
        # convert to hsv
        hsv_color_image = np.asanyarray(cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV))

        # apply thresholds
        hsv_color_image[:,:,2] = np.asanyarray(threshol_hue(hsv_color_image[:,:,0], hsv_color_image[:,:,2], threshold_hue)) #val
        hsv_color_image[:,:,2] = np.asanyarray(threshol_sat(hsv_color_image[:,:,1], hsv_color_image[:,:,2], threshold_sat)) #val

        # get our thresholded RGB image  
        color_image = cv2.cvtColor(hsv_color_image, cv2.COLOR_HSV2BGR)

        # convert our thresholded image to grayscale (mask)
        bw_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # apply erosion filter to mask
        kernel = np.ones((5, 5), np.uint8)
        img_erosion = cv2.erode(bw_image, kernel, iterations=1)
        
        # get contours
        contours, hierarchy = cv2.findContours(img_erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # if there are any contours
        if len(contours)>0:
            try:
                # get the biggest contour
                cnt = biggest_contour(contours)

                # get the moments & centroid
                M = cv2.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # get the depth for the centroid
                d = aligned_depth_frame.get_distance(cx,cy)

                # draw boinding box and center on image
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(color_image,[box],0,(0,0,255),2)
                cv2.circle(color_image, (cx,cy), radius=4, color=(255, 255, 255), thickness=-1)

                # if the distance to the center is > 0 we consider it valid
                if d > 0:
                    print(rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], d))

            except Exception:
                pass
        
        # show our image
        cv2.imshow(title_window, color_image)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop()