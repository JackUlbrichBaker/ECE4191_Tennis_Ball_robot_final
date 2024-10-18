# Written by Kelly, last updated 04/10/24

import cv2
import numpy as np
import time
#from picamera2 import Picamera2

#################################### TUNING PARAMETERS ########################################
#global picam2
#picam2 = Picamera2()
number_of_balls_to_detect = 3

# TUNING PARAMETERS FOR BALL DETECTION
circle_detect_wait_time = 5 # How many FRAMES you want to capture of a circle until it is considered detected
threshold = 10 # Pixel variance allowance between frames of a detected circle, for it to be considered the same circle
time_limit = 8 # Time limit where robot can detect ball before pivoting

# TUNING PARAMETERS FOR VISION MODEL
minDist_tune = 30 # Minimum distance between two circles to be considered separate circles
param2_tune = 20 # Sensitivity - lower = more sensitive
minRadius_tune = 5 # Minimum radius of valid circle
gauss_tune = 0 # Level of gaussian blur of raw image, to get rid of background noise
canny_tune = True # A different type of edge detecting method
STREAM_MODE = False

#################################### HELPER FUNCTIONS ########################################
# Helper function to draw circles on an opencv frame
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
        for circle in circles:
            # Extract the center (x, y) and radius (r) of each circle
            center_x, center_y, radius = circle
            
            # Draw the outer circle
            cv2.circle(img_to_draw, (center_x, center_y), radius, (0, 255, 0), 2)
            
            # Draw the center of the circle
            cv2.circle(img_to_draw, (center_x, center_y), 2, (0, 0, 255), 3)

            # Display the coordinates and radius on the image
            text = f"({center_x}, {center_y}), r={radius}"
            cv2.putText(img_to_draw, text, (center_x - 50, center_y - radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # Display the resulting frame ###################################################
    disp_img = img_to_draw

    return disp_img 

# Calculate euclidean distance (checking within radius)
def within_radius(x_rad, y_rad, radius, x1, y1):
    euclidean_dist = np.sqrt((x1 - x_rad)**2 + (y1 - y_rad)**2)
    within_radius = euclidean_dist <= radius

    return within_radius

def process_image(image_array):
    #################################### IMAGE PROCESSING ########################################
    # Image processing techniques to detect the ball based on colour

    # Blurring image
    avg_img = cv2.blur(image_array, (10, 10))
    blurred_img = cv2.GaussianBlur(avg_img, (15, 15), gauss_tune)

    # Altering colour channels
    hsv_image = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for green and yellow in HSV
    lower_green = np.array([35, 50, 50])  # Lower bound for green
    upper_green = np.array([65, 255, 255])  # Upper bound for green

    lower_yellow = np.array([25, 50, 50])  # Lower bound for yellow
    upper_yellow = np.array([40, 255, 255])  # Upper bound for yellow

    # Create masks for green and yellow
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    # Combine the masks
    mask = cv2.bitwise_or(mask_green, mask_yellow)

    # Morphological operations, removes the smaller blobs caused by noise
    mask = cv2.erode(mask,None,iterations=2)
    mask = cv2.dilate(mask,None,iterations=2)
  
    # Use the mask to extract the colored regions
    colored_regions = cv2.bitwise_and(image_array, image_array, mask=mask)
    # cv2.imshow('coloured regions', colored_regions)

    # Second blurring
    blurred_img = cv2.GaussianBlur(colored_regions, (5, 5), gauss_tune)

    # Edge detection using Canny
    find_edge_img = blurred_img[:, :, 2]

    if canny_tune == True:
        # Combine the edges from each channel
        edges_combined = cv2.Canny(find_edge_img, 110, 200,5, L2gradient=True)
    else:
        edges_combined = cv2.Canny(find_edge_img, 110, 200, L2gradient=False)

    #################################### CIRCLE DETECTION ########################################
    # Using Hough circle detector to detect balls

    circle_det_image = edges_combined

    return circle_det_image


################################### this function is running ########################################
def detect_single_ball():
    # Initialize the camera (use 0 for default camera)
    cap = cv2.VideoCapture(0)
    time.sleep(3)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera. Restart application")
        return

    ##################### PARAMETERS FOR BALL VALIDATION ####################################
    circle_det_log = [] # Counting how long a ball is detected for
    no_det_count = 0 # Used to count period of no. detected circles, used to reset the circle_det_log
    prev_det_circle = np.array([0,0,0]) # For comparison to previous frame
    final_circle = None

    #################################### MAIN LOOP #########################################
    start_time = time.time()
    first_iteration = True

    while sum(circle_det_log) < circle_detect_wait_time: # While circle has not succesfully appeared across 'x' frames WITHIN a time limit

        # Check if the time limit has been exceeded
        elapsed_time = time.time() - start_time
        if elapsed_time > time_limit: # If it has, return None
            print("Time limit exceeded...")
            return None

        # Capture frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break
        # Convert the image to a NumPy array
        image_array = np.array(frame)

        global disp_img
        disp_img = image_array
        STREAM_MODE = False
        #################### DISPLAY IMAGE ####################
#        if STREAM_MODE:
            #generate_stream(disp_img)
#            print("oops trying to stream")
#        else:
#            cv2.imshow('Circle Detector, press ''q'' to exit', disp_img)
#            #Break the loop on 'q' key press
#            if cv2.waitKey(1) & 0xFF == ord('q'):
#                break

        #################### PRE-PROCESS IMAGE ####################
        circle_det_image = process_image(image_array)

        #################### CIRCLE DETECTION ####################
        circles = cv2.HoughCircles(
            circle_det_image,                 # Input image
            cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
            dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
            minDist=minDist_tune,             # Minimum distance between the centers of detected circles
            param1=50,                        # Higher threshold for Canny edge detection
            param2=param2_tune,               # Accumulator threshold for the circle centers at the detection stage
            minRadius=minRadius_tune,         # Minimum radius of the circles
            maxRadius=0                       # Maximum radius of the circles
        )

        #################### PICK BEST CIRCLE ####################
        if circles is not None: # If circle is detected in current frame

            circles = np.uint16(np.around(circles))
            circles = np.round(circles[0, :]).astype("int")

            # Sort circles from largest to smallest radius
            circles_sorted = sorted(circles, key=lambda c: c[2], reverse=True)

            # Since we expect only one ball, pick the circle with the largest radius
            best_circle = circles_sorted[0]

            #################### UPDATE IMAGE ####################
 #           img_to_display = image_array # Draw circles on raw image
 #           disp_img = draw_circles([best_circle], img_to_display)
 #           STREAM_MODE = False
 #           if STREAM_MODE:
                #generate_stream(disp_img)
 #               print("oops trying to stream")
 #           else:
 #               cv2.imshow('Circle Detector, press ''q'' to exit', disp_img)
 #               #Break the loop on 'q' key press
 #               if cv2.waitKey(1) & 0xFF == ord('q'):
 #                   break

            #################### VALIDATE CIRCLE ACROSS FRAMES ####################
            if first_iteration: # Initiate comparison to previous circle on first iteration
                prev_det_circle = best_circle
                first_iteration = False

            if sum( abs(prev_det_circle - best_circle) < threshold) == 3: # Compare to previous circle in previous frame and log as detected if it is within the pixel threshold
                final_circle = best_circle
                circle_det_log.append(1) # Log detected circle
            else:
                no_det_count+=1 # Increase failure count

            prev_det_circle = best_circle # Update previous circle to current circle for comparison in next iteration

            if PRINT_DETECT:
                print(f"Number of validated circles: {sum(circle_det_log)}/{circle_detect_wait_time}, Failure count: {no_det_count}/50")

        else: # No circle detected in current frame
            no_det_count+=1 # Increase failure count


        #################### FAILED VALIDATION ####################
        # If no ball is sucessfully tracked within 50 frames
        if no_det_count >= 50:
            print("Failed to stabilise ball detection, trying again...")
            circle_det_log = [] # Reset log of successful detects
            no_det_count = 0 # Reset failure count

        # else: # Update final circle
        #     final_circle = best_circle

    #################### SUCCESSFUL VALIDATION ####################
    if sum(circle_det_log) >= circle_detect_wait_time: # Successful detection
        print(f"CIRCLE DETECTED! Coordinates [x, y, radius] = {final_circle}")

        # Release the camera and close the window
        cap.release()
        cv2.destroyAllWindows()

        return final_circle

    else:
        print("Aborting...")
        time.sleep(5)
        return None





#################################### CV2 CAM VISION FUNCTIONS ########################################
# OPENCV VERSION that only works with USB camera and laptop camera

def detect_ball_cv2():
    view_mode = True
    stream_mode = False

    # Initialize the camera (use 0 for default camera)
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    # #################################### MAIN LOOP ###############################################################################

    # Loop to continuously capture frames from the camera
    start_time = time.time()
    while sum(circle_det_log) < circle_detect_wait_time: # While circle has not reached detection time threshold

        # Check if the time limit has been exceeded
        elapsed_time = time.time() - start_time

        if elapsed_time > time_limit: # Break out of loop and return final_circle = None
            timeout_flag = True
            break
        else:
            timeout_flag = False

        # INITIALISE CAMERA Capture frame-by-frame
        ret, frame = cap.read()
        # If the frame is captured successfully, ret will be True
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Convert the image to a NumPy array
        image_array = np.array(frame)

        #################################### IMAGE PROCESSING ########################################
        # Image processing techniques to detect the ball based on colour

        # Blurring image
        blurred_img = cv2.GaussianBlur(image_array, (11, 11), gauss_tune)

        # Altering colour channels
        hsv_image = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for green and yellow in HSV
        lower_green = np.array([35, 50, 50])  # Lower bound for green
        upper_green = np.array([65, 255, 255])  # Upper bound for green

        lower_yellow = np.array([25, 50, 50])  # Lower bound for yellow
        upper_yellow = np.array([40, 255, 255])  # Upper bound for yellow

        # Create masks for green and yellow
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Combine the masks
        mask = cv2.bitwise_or(mask_green, mask_yellow)

        # Morphological operations, removes the smaller blobs caused by noise
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask,None,iterations=2)

        # Use the mask to extract the colored regions
        colored_regions = cv2.bitwise_and(image_array, image_array, mask=mask)
        cv2.imshow('coloured regions', colored_regions)

        # Second blurring
        blurred_img = cv2.GaussianBlur(colored_regions, (5, 5), gauss_tune)

        # Edge detection using Canny
        find_edge_img = blurred_img[:, :, 2]

        if canny_tune == True:
            # Combine the edges from each channel
            edges_combined = cv2.Canny(find_edge_img, 110, 200,5, L2gradient=True)
        else:
            edges_combined = cv2.Canny(find_edge_img, 110, 200, L2gradient=False)
        
        #################################### CIRCLE DETECTION ########################################
        # Using Hough circle detector to detect balls

        circle_det_image = edges_combined

        # Hough circle detector, returns detected circles as an array called circles
        circles = cv2.HoughCircles(
            circle_det_image,                 # Input image
            cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
            dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
            minDist=minDist_tune,             # Minimum distance between the centers of detected circles
            param1=50,                        # Higher threshold for Canny edge detection
            param2=param2_tune,               # Accumulator threshold for the circle centers at the detection stage
            minRadius=minRadius_tune,         # Minimum radius of the circles
            maxRadius=0                       # Maximum radius of the circles
        )

        # Frames to draw circles on
        img_to_draw = image_array
        raw_circle_det = image_array
        final_circle_det = image_array

        # Make circle into processable array
        if circles is not None:
            circles = np.uint16(np.around(circles))
            circles = np.round(circles[0, :]).astype("int")

            #################################### CIRCLE ARRAY FILTERING ########################################
            # Deleting invalid detected balls based on radius

            # Sort detected circles by radius, largest first
            circles_sorted = sorted(circles, key=lambda c: c[2], reverse=True)

            # Keep circles with the highest radius only = BEST CIRCLES
            if len(circles_sorted) > number_of_balls_to_detect:
                best_circles = circles_sorted[:number_of_balls_to_detect]
            else: # If no. balls detected is less than the maximum
                best_circles = circles

            #################################### VISUALISATION ########################################
            raw_circle_det = draw_circles(circles, img_to_draw) # Draw all the balls
            final_circle_det = draw_circles(best_circles, img_to_draw) # Draw only the best balls

        #################################### DETECTION VALIDATION ########################################
        # Only balls that are detected for a certain period of time are considered valid
        # best_circles.shape = n x 3, where n = detected circles
        best_circles = np.array(best_circles)
        no_balls, _ = best_circles.shape
        # TODO:
        circle_det_log = np.empty((1,no_balls)) # Counting how long each ball is detected for
        no_det_count = np.empty((1,no_balls)) # Used to count period of no. detected circles, used to reset the circle_det_log
        final_circles = []
        prev_det_circle = best_circles # For comparison to previous frame

        if circles is not None: # If a circle is detected
            print("Validating balls...")


            for circle in prev_det_circle:
                x_rad, y_rad, _ = circle
                if within_radius():
                    #TODO:
                    break
            
            # Compare to previous circle and log as detected if it is within the threshold
            if sum( abs(prev_det_circle - final_circle) < threshold) == 3:
                # Log detected circle
                circle_det_log.append(1)
            else:
                no_det_count+=1 # Increase failure count

            prev_det_circle = final_circle # Update previous circle to current circle for comparison in next loop

        else: 
            final_circle = [] # No circle is detected
            no_det_count+=1 # Increase failure count

        # Reset circle_det_log if no ball is detected over a certain time
        if no_det_count >= 50:
            circle_det_log = []
            no_det_count = 0

        if view_mode == True and stream_mode == False:
            cv2.imshow('Final Circles Detected', final_circle_det)
            cv2.imshow('Edge Detector', edges_combined)
            cv2.imshow('Raw Circles Detected', raw_circle_det)

            #Break the loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # elif view_mode == False and stream_mode == True:
        #     # Encode the frame in JPEG format
        #     ret, buffer = cv2.imencode('.jpg', disp_img)
        #     frame = buffer.tobytes()

        #     # Yield the frame as part of the stream
        #     yield (b'--frame\r\n'
        #            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
    #################################### END OF MAIN LOOP ###############################################################################

    if timeout_flag == True: # If loop was broken due to no circle detected...
        print("Time limit exceeded: No ball has been detected")
        final_circle = None

    elif final_circle == []:
        print("SCRIPT HAS CRASHED")

    else: # If loop was broken due to circle successfully detected ...
        # Release the camera and close the window
        print(f"CIRCLE DETECTED! Coordinates (x y radius): {final_circle}")

    cap.release()
    cv2.destroyAllWindows()
   
    return final_circle

def detect_single_ball_cv2():
    print("Running ball detect")

    view_mode = False

    # Initialize the camera (use 0 for default camera)
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    # PARAMETERS FOR BALL DETECTION
    circle_det_log = [] # Counting how long a ball is detected for
    circle_detect_wait_time = 5 # How long you want to wait until a ball is considered to be detected
    no_det_count = 0 # Used to count period of no. detected circles, used to reset the circle_det_log
    prev_det_circle = np.array([0,0,0]) # For comparison to previous frame
    threshold = 30 # Pixel variance allowance between frames of a detected circle, for it to be considered the same circle
    final_circle = []
    time_limit = 8 # Time limit where robot can detect ball

    # PARAMETERS FOR TUNING VISION MODEL
    minDist_tune = 1
    param2_tune = 20
    minRadius_tune = 1
    gauss_tune = 7
    canny_tune = True

    start_time = time.time()

    # Loop to continuously capture frames from the camera
    while sum(circle_det_log) < circle_detect_wait_time:

        # Check if the time limit has been exceeded
        elapsed_time = time.time() - start_time

        if elapsed_time > time_limit:
            timeout_flag = True
            break
        else:
            timeout_flag = False

        # Capture frame-by-frame
        ret, frame = cap.read()
        # If the frame is captured successfully, ret will be True
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Convert the image to a NumPy array
        image_array = np.array(frame)
        avg_img = cv2.blur(image_array,(10,10))
        gauss_blur = cv2.GaussianBlur(avg_img,(15,15),gauss_tune) 

        # Altering colour channels ######################################################################
        hsv_image = cv2.cvtColor(gauss_blur, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for green and yellow in HSV
        lower_green = np.array([35, 50, 50])  # Lower bound for green
        upper_green = np.array([65, 255, 255])  # Upper bound for green

        lower_yellow = np.array([25, 50, 50])  # Lower bound for yellow
        upper_yellow = np.array([40, 255, 255])  # Upper bound for yellow

        # Create masks for green and yellow
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Combine the masks
        mask = cv2.bitwise_or(mask_green, mask_yellow)

        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask,None,iterations=2)

        # Use the mask to extract the colored regions
        colored_regions = cv2.bitwise_and(image_array, image_array, mask=mask)

        blurred_img = cv2.GaussianBlur(colored_regions, (5, 5), gauss_tune)

        # Edge detection ###############################################################
        find_edge_img = blurred_img[:, :, 2]

        if canny_tune == True:
            # ombine the edges from each channel
            edges_combined = cv2.Canny(find_edge_img, 110, 200,5, L2gradient=True)
        else:
            edges_combined = cv2.Canny(find_edge_img, 110, 200, L2gradient=False)

        # Circle detection ###############################################################
        circle_det_image = edges_combined

        circles = cv2.HoughCircles(
            circle_det_image,                 # Input image
            cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
            dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
            minDist=minDist_tune,                       # Minimum distance between the centers of detected circles
            param1=50,                        # Higher threshold for Canny edge detection
            param2=param2_tune,                        # Accumulator threshold for the circle centers at the detection stage
            minRadius=minRadius_tune,                      # Minimum radius of the circles
            maxRadius=0                       # Maximum radius of the circles
        )

        # FOR VISUALISATION PURPOSES ONLY: drawing detected circles on any input image ################

        img_to_draw = image_array

        if circles is not None:
            circles = np.uint16(np.around(circles))

        # If circles are detected, process them
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            # Sort circles by the accumulator value (confidence) if available
            circles_sorted = sorted(circles, key=lambda c: c[2], reverse=True)

            # Since we expect only one ball, pick the circle with the highest confidence
            best_circle = circles_sorted[0]

            # Draw the circle and its center on the original image
            cv2.circle(img_to_draw, (best_circle[0], best_circle[1]), best_circle[2], (0, 255, 0), 4)
            cv2.circle(img_to_draw, (best_circle[0], best_circle[1]), 2, (0, 0, 255), 3)

            center_x, center_y, radius = best_circle

            # Display the coordinates and radius on the image
            text = f"({center_x}, {center_y}), r={radius}"
            cv2.putText(img_to_draw, text, (center_x - 50, center_y - radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Display the resulting frame
        disp_img = img_to_draw


        # Processing resulting frame ######################################################################

        if circles is not None: # If a circle is detected
            final_circle = best_circle
            #print(final_circle)
            # Compare to previous circle and log as detected if it is within the threshold
            if sum( abs(prev_det_circle - final_circle) < threshold) == 3:
                # Log detected circle
                circle_det_log.append(1)
                print(f"Successful detect count: {sum(circle_det_log)}/10")
            else:
                no_det_count+=1

            prev_det_circle = final_circle # Update previous circle to current circle for comparison in next loop

        else:
        #    final_circle = []
            no_det_count+=1

        # Reset circle_det_log if no ball is detected over a certain time
        if no_det_count >= 50:
            circle_det_log = []
            no_det_count = 0
        print(f"Failure count: {no_det_count}/50")

 #       if view_mode == True:
 #           cv2.imshow('Circle Detector', disp_img)
 #           cv2.imshow('Edge Detector', edges_combined)
#
#            #Break the loop on 'q' key press
#            if cv2.waitKey(1) & 0xFF == ord('q'):
#                break
    if sum(circle_det_log) >= circle_detect_wait_time:
        print("CIRCLE DETECTED!")
        return final_circle
    else:
        print("Aborting...")
        return
    
    if timeout_flag == True:
        print("Time limit exceeded: No ball has been detected")
        final_circle = None

#    else:
#        # Release the camera and close the window
#        print(f"CIRCLE DETECTED! Coordinates: x:{final_circle}")

    cap.release()
#    cv2.destroyAllWindows()

    return final_circle

#################################### PICAM VISION FUNCTIONS ########################################

# Need to run this before detect_ball_picam
def start_picam():
    # Initialize the camera (use 0 for default camera)
    # Configure picamera to receive raw bayer format
    config = picam2.create_still_configuration(raw={"format": "SRGGB10"})
    picam2.set_controls({"AwbEnable": False, "ColourGains": (1.5, 1.5)})  # Adjust the gains
    picam2.configure(config)
    # Start camera
    picam2.start()
    # Wait for camera to warm up
    time.sleep(3)

def detect_ball_picam():

    # #################################### MAIN LOOP ###############################################################################
    final_circles = None
    # Loop to continuously capture frames from the camera
    start_time = time.time()
    while final_circles is None: #sum(circle_det_log) < circle_detect_wait_time: # While circle has not reached detection time threshold

        # Check if the time limit has been exceeded
        elapsed_time = time.time() - start_time

        if elapsed_time > time_limit: # Break out of loop and return final_circle = None
            timeout_flag = True
            break
        else:
            timeout_flag = False

        # INITIALISE CAMERA Capture frame-by-frame
        raw_bayer = picam2.capture_array("raw")
        rgb_image = cv2.cvtColor(raw_bayer, cv2.COLOR_BAYER_RG2BGR)

        # Convert the image to a NumPy array
        image_array = np.array(rgb_image)

        #################################### IMAGE PROCESSING ########################################
        # Image processing techniques to detect the ball based on colour

        # Blurring image
        blurred_img = cv2.GaussianBlur(image_array, (11, 11), gauss_tune)

        # Altering colour channels
        hsv_image = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for green and yellow in HSV
        lower_green = np.array([35, 50, 50])  # Lower bound for green
        upper_green = np.array([65, 255, 255])  # Upper bound for green

        lower_yellow = np.array([25, 50, 50])  # Lower bound for yellow
        upper_yellow = np.array([40, 255, 255])  # Upper bound for yellow

        # Create masks for green and yellow
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Combine the masks
        mask = cv2.bitwise_or(mask_green, mask_yellow)

        # Morphological operations, removes the smaller blobs caused by noise
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask,None,iterations=2)

        # Use the mask to extract the colored regions
        colored_regions = cv2.bitwise_and(image_array, image_array, mask=mask)

        # Second blurring
        blurred_img = cv2.GaussianBlur(colored_regions, (5, 5), gauss_tune)

        # Edge detection using Canny
        find_edge_img = blurred_img[:, :, 2]

        if canny_tune == True:
            # Combine the edges from each channel
            edges_combined = cv2.Canny(find_edge_img, 110, 200,5, L2gradient=True)
        else:
            edges_combined = cv2.Canny(find_edge_img, 110, 200, L2gradient=False)
        
        #################################### CIRCLE DETECTION ########################################
        # Using Hough circle detector to detect balls

        circle_det_image = edges_combined

        # Hough circle detector, returns detected circles as an array called circles
        circles = cv2.HoughCircles(
            circle_det_image,                 # Input image
            cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
            dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
            minDist=minDist_tune,             # Minimum distance between the centers of detected circles
            param1=50,                        # Higher threshold for Canny edge detection
            param2=param2_tune,               # Accumulator threshold for the circle centers at the detection stage
            minRadius=minRadius_tune,         # Minimum radius of the circles
            maxRadius=0                       # Maximum radius of the circles
        )

        # Frames to draw circles on
        img_to_draw = image_array

        # Make circle into processable array
        if circles is not None:
            circles = np.uint16(np.around(circles))
            circles = np.round(circles[0, :]).astype("int")

            #################################### CIRCLE ARRAY FILTERING ########################################
            # Deleting invalid detected balls based on radius

            # Sort detected circles by radius, largest first
            circles_sorted = sorted(circles, key=lambda c: c[2], reverse=True)

            # Keep circles with the highest radius only = BEST CIRCLES
            if len(circles_sorted) > number_of_balls_to_detect:
                best_circles = circles_sorted[:number_of_balls_to_detect]
            else: # If no. balls detected is less than the maximum
                best_circles = circles

            #################################### VISUALISATION ########################################
            final_circle_det = draw_circles(best_circles, img_to_draw) # Draw only the best balls

        #################################### STREAMING ########################################
            # Encode the frame in JPEG format
            ret, buffer = cv2.imencode('.jpg', final_circle_det)
            frame = buffer.tobytes()

            # Yield the frame as part of the stream
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
        #################################### DETECTION VALIDATION ########################################
        # Only balls that are detected for a certain period of time are considered valid
        # best_circles.shape = n x 3, where n = detected circles

        #TODO: paste code from cv version into this

        # for now
        take_image = input("Press q to capture circles...")
        if take_image == 'q':
            final_circles = best_circles
            
    #################################### END OF MAIN LOOP ###############################################################################

    # if timeout_flag == True: # If loop was broken due to no circle detected...
    #     print("Time limit exceeded: No ball has been detected")
    #     final_circles = None

    # # elif final_circles == []:
    # #     print("SCRIPT HAS CRASHED")

    # else: # If loop was broken due to circle successfully detected ...
    #     # Release the camera and close the window
    #     print(f"CIRCLE DETECTED! Coordinates (x y radius): {final_circles}")
    print(f"CIRCLE DETECTED! Coordinates (x y radius): {final_circles}")

    return final_circles
