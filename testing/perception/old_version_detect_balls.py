import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from collections import defaultdict

# from main import robot, court_length, court_width

################################# FUNCTIONS #################################

def convert_pixels_robot_coord(x_pixel, radius):
    # function converts the pixel counts to the x distance and y distance from the robot in cm
    # x_pixel = hoziontal pixel number from centre of image; radius = radius of circle drawn around tennis ball 

    avg_tennis_diameter = 6.3                       # cm TODO check size 
    scale_factor = avg_tennis_diameter / (2*radius) # cm/pixel 
    focal_length = 655                              # need to calibrate focal length
    camera_height = 8                              # TODO need to measure camera height 

    # calculate horizontal distance in cm from directly ahead of camera 
    tx_robot = x_pixel * scale_factor
    
    # get distance from robot to ball 
    measured_distance = (avg_tennis_diameter * focal_length ) / (2*radius)     # distance = (known width of tennis ball * focal length) / pixel width

    ty_robot = np.sqrt(measured_distance**2 - camera_height**2)
    
    # ground_distance = np.sqrt(measured_distance**2 - camera_height**2)
    # ty_robot = np.sqrt(ground_distance**2 - tx_robot**2)

    return tx_robot, ty_robot

def convert_robot_world_coord(tx_robot,ty_robot):
    # function converts the tennis ball coordinates from the robot to the world frame axis 
    # tx_robot = the tennis ball x coord from the robot frame, rx_origin = the robot x coord from the frame frame, robot = robot class 

    tx_world = tx_robot*np.cos(robot.th) - ty_robot*np.sin(robot.th) + robot.x
    ty_world = tx_robot*np.sin(robot.th) + ty_robot*np.cos(robot.th) + robot.y

    # NOTE will need to check for None values before adding to final target array 
    tx_world, ty_world = check_boundaries(tx_world, ty_world, court_length, court_width)
    
    return tx_world, ty_world

def check_boundaries(tx_world, ty_world, court_length, court_width):
    # functions allows for 10cm bias on each side of the court as its probably not very accurate 

    if tx_world < -10 or tx_world > court_width + 10:
        tx_world = None 
  
    if ty_world < -10 or ty_world > court_length + 10: 
        ty_world = None

    return tx_world, ty_world

def coords_ULHS_to_middle(circle_array, image):
    rows, cols, _ = image.shape
    x, y, radius = circle_array
    img_mid_x, img_mid_y = np.floor(rows/2), np.floor(cols/2)
    x_c, y_c = x-img_mid_x, y-img_mid_y
  
    return np.array([x_c, y_c, radius])

def take_image(no_frames = 1, warmup_time = 30):
    """
    Will take a specified amount of images and return the frames. If 1 image, it will return just the frame, otherwise it will return an array.
    
    Parameters:
    - no_frames (int): Number of frames to capture
    
    Returns:
    If no_frames = 1:
        - frame: image taken
    If no_frames > 1:
    - captured_frames (array): images taken
    """
    # Initialize the camera (use 0 for default camera)
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    # If more than one frame is required
    if no_frames != 1:
        # Warm-up period: Capture a few dummy frames to allow the camera to adjust
        for i in range(warmup_time):
            ret, _ = cap.read()

        captured_frames = []
        for i in range(no_frames):
            ret, frame = cap.read()
            captured_frames.append(frame)
        return captured_frames

    # If one only frame is captured
    elif no_frames == 1:
        # Warm-up period: Capture a few dummy frames to allow the camera to adjust
        for i in range(warmup_time):
            ret, _ = cap.read()

        # Capture frame
        ret, frame = cap.read()

        # If the frame is captured successfully, ret will be True
        if not ret:
            print("Error: Failed to capture frame.")
            return

        return frame

def detect_circles(frame, minDist_tune = 50, param2_tune = 40):
    """
    Takes an image in the form of a numpy array and runs Hough circle detector on it.
    
    Parameters:
    - frame: image frame taken using openCV
    - minDist_tune (int): Hough circle detector tuning parameter - Minimum distance between the centers of detected circles
    - param2_tune (int): Hough circle detector tuning parameter - LOWER = MORE SENSITIVE, HIGHER = LESS SENSITIVE
    
    Returns:
    - circles (numpy array): 3 x n array of n detected circles in the form [x, y, radius]
    """

    # Convert the image to a NumPy array
    image_array = np.array(frame)

    # Normalising image
    # image_array = image_array.astype(np.float32) / 255.0

    # Altering colour channels ######################################################################
    hsv_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for green and yellow in HSV
    lower_green = np.array([35, 50, 50])  # Lower bound for green
    upper_green = np.array([85, 255, 255])  # Upper bound for green

    lower_yellow = np.array([20, 50, 50])  # Lower bound for yellow
    upper_yellow = np.array([35, 255, 255])  # Upper bound for yellow

    # Create masks for green and yellow
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    # Combine the masks
    mask = cv2.bitwise_or(mask_green, mask_yellow)

    # Use the mask to extract the colored regions
    colored_regions = cv2.bitwise_and(image_array, image_array, mask=mask)

    blurred_img = cv2.GaussianBlur(colored_regions, (5, 5), 1)

    # Edge detection ###############################################################
    find_edge_img = blurred_img

    # Split the image into its R, G, B channels
    b, g, r = cv2.split(find_edge_img)

    # Apply Canny edge detection on each channel
    edges_b = cv2.Canny(b, 100, 200)
    edges_g = cv2.Canny(g, 100, 200)
    edges_r = cv2.Canny(r, 100, 200)

    # # Combine the edges from each channel
    edges_combined = cv2.bitwise_or(cv2.bitwise_or(edges_b, edges_g), edges_r)

    # Circle detection ############################################################### 
    circle_det_image_gray = edges_combined

    # NOTE: TUNE THESE PARAMETERS, SPECIFICALLY PARAM2, LOWER = MORE SENSITIVE, HIGHER = LESS SENSITIVE
    circles = cv2.HoughCircles(
        circle_det_image_gray,                   # Input image
        cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available), NOTE: needs GREYSCALE IMAGE
        dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
        minDist= minDist_tune,                       # NOTE: TUNE THIS Minimum distance between the centers of detected circles
        param1=50,                        # Higher threshold for Canny edge detection
        param2= param2_tune,                        # NOTE: TUNE THIS Accumulator threshold for the circle centers at the detection stage
        minRadius=0,                      # Minimum radius of the circles
        maxRadius=0                       # Maximum radius of the circles
    )

    return circles

def clean_circles(circles_array, eps_tune=10):
    """
    Takes an array of circles that are detected in a specified number of image frames, and takes the average of them to get rid of outliers.
    
    Parameters:
    - circles_array (array): 4D array of detected circles [3 x no_circles_detected x no_frames]
    
    Returns:
    - average_circles (np array): Array of averaged circles
    """
    # Convert the circles data into a feature matrix (x, y, radius)
    X = np.array([(x, y, r) for x, y, r in circles_array])

    # Apply DBSCAN clustering
    dbscan = DBSCAN(eps=eps_tune, min_samples=1).fit(X[:, :2])  # Only cluster by x and y coordinates

    # Get labels for each detected circle
    labels = dbscan.labels_

    # Dictionary to store clusters of circles
    clusters = defaultdict(list)

    # Group circles by their cluster label
    for label, circle in zip(labels, X):
        if label != -1:  # Ignore noise
            clusters[label].append(circle)

    # Compute average circle for each cluster
    average_circles = []
    for cluster_id, circles in clusters.items():
        circles = np.array(circles)
        avg_x = np.mean(circles[:, 0])
        avg_y = np.mean(circles[:, 1])
        avg_r = np.mean(circles[:, 2])
        average_circles.append((avg_x, avg_y, avg_r))

    # Convert feature matrix into numpy array
    average_circles = np.array([average_circles])

    return average_circles

def draw_circles(circles, frame):
    """
    FOR VISUALISATION PURPOSES ONLY: drawing detected circles on any input image
    
    Parameters:
    - frame: image frame taken using openCV
    - circles (numpy array): 3 x n array of n detected circles
    
    Returns:
    - disp_img (numpy array): image to display
    """

    image_array = np.array(frame)

    img_to_draw = image_array

    if circles is not None:
        circles = np.uint16(np.around(circles))

    if circles is not None:
        for circle in circles[0, :]:
            # Extract the center (x, y) and radius (r) of each circle
            center_x, center_y, radius = circle
            
            # Draw the outer circle
            cv2.circle(img_to_draw, (center_x, center_y), radius, (0, 255, 0), 2)
            
            # Draw the center of the circle
            cv2.circle(img_to_draw, (center_x, center_y), 2, (0, 0, 255), 3)
    
    # Display the resulting frame ###################################################
    disp_img = img_to_draw

    return disp_img

def view_live_cam():
    """
    Opens live camera feed in a window, press q to quit.
    
    Parameters:
    - none
    
    Returns:
    - none
    """

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    # Loop to continuously capture frames from the camera
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # If the frame is captured successfully, ret will be True
        if not ret:
            print("Error: Failed to capture frame.")
            break
        
        cv2.imshow('Live Camera Feed', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close the window
    cap.release()
    cv2.destroyAllWindows()

    return

def tune_params(frames, minDist_tune, param2_tune, eps_tune):
    '''
    Prints out the frames taken as well as circles detected in them. Include minDist_tune, param2_tune, eps_tune as arguements to tune model.
    '''
    all_circles = []

    for idx, img in enumerate(frames):
        circles = detect_circles(img, minDist_tune, param2_tune)

        # Draw detected circles
        disp_img = draw_circles(circles, img) 
        cv2.imshow(f'RAW DETECTED IMAGE {idx}', disp_img)

        all_circles.extend(circles[0])

    # Get rid of falsely detected circles
    average_circles = clean_circles(all_circles, eps_tune)

    # Display detected image
    disp_img = draw_circles(average_circles, frames[1])
    cv2.imshow("AVERAGED IMAGE", disp_img)
        
    return

def detect_tennis_balls_in_images(num_frames, court_length, court_width):
    frames = take_image(num_frames)

    # Detect circles in each image
    all_circles = []

    for img in frames:
        circles = detect_circles(img, minDist_tune = 50, param2_tune = 35)
        if circles is not None:
            all_circles.extend(circles[0])
        else:
            print("no circles detected in frame")
            return

    # Get rid of falsely detected circles
    average_circles = clean_circles(all_circles) # [x, y, radius] from the top left corner
    ball_coords = average_circles
    
    # call distance functions 
    for idx, ball in enumerate(average_circles):
        x_robot, y_robot = convert_pixels_robot_coord(ball[0], ball[2])
        x_world, y_world = convert_robot_world_coord(x_robot,y_robot)
        x_world, y_world = check_boundaries(x_world, y_world, court_length, court_width)

        if x_world == None or y_world == None: 
            ball_coords = np.delete(ball_coords, idx)

    return average_circles

