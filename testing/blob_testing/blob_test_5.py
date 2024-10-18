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

    # Increasing image saturation ##############################################################

    # hsv_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2HSV)

    # # Split the HSV image into its channels (Hue, Saturation, Value)
    # h, s, v = cv2.split(hsv_image)

    # # Increase the saturation by a factor (for example, 1.5x)
    # s = cv2.multiply(s, 1.5)
    # s = np.clip(s, 0, 255).astype(np.uint8)  # Ensure the saturation values are within [0, 255]

    # # Merge the channels back
    # hsv_image = cv2.merge([h, s, v])

    # # Convert the HSV image back to BGR
    # bgr_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

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

    # # Create a grayscale version of the image
    # gray_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)

    # # Convert the grayscale image back to BGR (so it has three channels)
    # gray_image_bgr = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

    # Use the mask to extract the colored regions
    colored_regions = cv2.bitwise_and(image_array, image_array, mask=mask)

    blurred_img = cv2.GaussianBlur(colored_regions, (5, 5), 10)

    # # Use the inverse of the mask to make the other regions grayscale
    # mask_inv = cv2.bitwise_not(mask)
    # gray_regions = cv2.bitwise_and(gray_image_bgr, gray_image_bgr, mask=mask_inv)

    # # Combine the colored and grayscale regions
    # result = cv2.add(colored_regions, gray_regions)

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
    circle_det_image = edges_combined

    # Convert image to RGB before converting to greyscale
    # rgb_image = cv2.cvtColor(circle_det_image, cv2.COLOR_BGR2RGB)
    # circle_det_image_gray = cv2.cvtColor(circle_det_image, cv2.COLOR_RGB2GRAY) # Hough circle detector requires a single channel input image
    circle_det_image_gray = edges_combined

    circles = cv2.HoughCircles(
        circle_det_image_gray,                   # Input image
        cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
        dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
        minDist=100,                       # Minimum distance between the centers of detected circles
        param1=50,                        # Higher threshold for Canny edge detection
        param2=40,                        # Accumulator threshold for the circle centers at the detection stage
        minRadius=0,                      # Minimum radius of the circles
        maxRadius=0                       # Maximum radius of the circles
    )

    # Convert the circles' coordinates and radii to integers
    '''
    next step:
    figure out how to throw away the false detections
    - averaging over 5 images etc
    - using some critera
    '''
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
    
    # Display the resulting frame ###################################################
    disp_img = img_to_draw

    cv2.imshow('Live Camera Feed', disp_img)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()

'''
NOTES:
- Removed other colours besides yellow and green
- Running edge detction after this works, but causes lots of noise and is slow

IMPROVEMENTS:
- false positives need to be deleted
'''