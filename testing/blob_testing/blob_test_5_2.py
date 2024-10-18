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

# Loop to continuously capture frames from the camera
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # If the frame is captured successfully, ret will be True
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Convert the image to a NumPy array
    image_array = np.array(frame)

    # Altering colour channels ######################################################################
    hsv_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2HSV)

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

    # Use the mask to extract the colored regions
    colored_regions = cv2.bitwise_and(image_array, image_array, mask=mask)

    blurred_img = cv2.GaussianBlur(colored_regions, (5, 5), 7)

    # Edge detection ###############################################################
    find_edge_img = blurred_img[:, :, 2]

    # # Combine the edges from each channel
    edges_combined = cv2.Canny(find_edge_img, 110, 200,5, L2gradient=True)

    # Circle detection ###############################################################
    circle_det_image = edges_combined

    circles = cv2.HoughCircles(
        circle_det_image,                 # Input image
        cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
        dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
        minDist=50,                       # Minimum distance between the centers of detected circles
        param1=50,                        # Higher threshold for Canny edge detection
        param2=20,                        # Accumulator threshold for the circle centers at the detection stage
        minRadius=10,                      # Minimum radius of the circles
        maxRadius=0                       # Maximum radius of the circles
    )

    # FOR VISUALISATION PURPOSES ONLY: drawing detected circles on any input image
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

            # Display the coordinates and radius on the image
            text = f"({center_x}, {center_y}), r={radius}"
            cv2.putText(image_array, text, (center_x - 50, center_y - radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # Display the resulting frame ###################################################
    disp_img = img_to_draw

    cv2.imshow('Circle Detector', disp_img)
    cv2.imshow('Edge Detector', edges_combined)


    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()