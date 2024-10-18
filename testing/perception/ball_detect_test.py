# LIVE CAMERA FEED
# EDITING COLOUR CHANNELS
# EDGE DETECTOR

import cv2
import numpy as np
import colorsys

# Initialize the camera (use 0 for default camera)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

circle_det_log = []
no_det_count = 0
prev_det_circle = np.array([0,0,0])
threshold = 8 # allowance of 5 pixels
final_circle = []

circle_detect_wait_time = 80 # how long you want to wait until a ball is considered to be detected

# Loop to continuously capture frames from the camera
while sum(circle_det_log) < circle_detect_wait_time:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # If the frame is captured successfully, ret will be True
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Convert the image to a NumPy array
    image_array = np.array(frame)
    image_array_blurred = cv2.GaussianBlur(image_array , (29,29),0)

    # Altering colour channels ######################################################################
    hsv_image = cv2.cvtColor(image_array_blurred, cv2.COLOR_BGR2HSV)

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

    # Get rid of spots of noise
    mask = cv2.erode(mask,None,iterations=2)
    mask = cv2.dilate(mask,None,iterations=2)

    # Use the mask to extract the colored regions
    colored_regions = cv2.bitwise_and(image_array, image_array, mask=mask)

    blurred_img = cv2.GaussianBlur(colored_regions, (5, 5), 7)

    # Edge detection ###############################################################
    find_edge_img = blurred_img[:, :, 2]

    # # Combine the edges from each channel
    edges_combined = cv2.Canny(find_edge_img, 100, 200, 5, L2gradient=True)

    # Circle detection ###############################################################
    circle_det_image = edges_combined

    circles = cv2.HoughCircles(
        circle_det_image,                 # Input image
        cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
        dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
        minDist=1,                       # Minimum distance between the centers of detected circles
        param1=50,                        # Higher threshold for Canny edge detection
        param2=20,                        # Accumulator threshold for the circle centers at the detection stage
        minRadius=0,                      # Minimum radius of the circles
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

    if circles is not None:
        final_circle = best_circle
        # Compare to previous circle and log as detected if it is within the threshold
        if sum( abs(prev_det_circle - final_circle) < threshold) == 3:
            # Log detected circle
            circle_det_log.append(1)
        else:
            no_det_count+=1

        prev_det_circle = final_circle # Update previous circle to current circle for comparison in next loop

    else:
        final_circle = []
        no_det_count+=1
    
    # Reset circle_det_log if no ball is detected over a certain time
    if no_det_count >= 50:
        circle_det_log = []
        no_det_count = 0

    print(sum(circle_det_log))
    
    cv2.imshow('Circle Detector', disp_img)
    cv2.imshow('Edge Detector', edges_combined)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
print(f"CIRCLE DETECTED! Coordinates: x:{final_circle}")

cv2.waitKey(0)
cap.release()
cv2.destroyAllWindows()