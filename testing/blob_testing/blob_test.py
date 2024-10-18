# EXPLORING COLOUR DETECTION

import cv2
import numpy as np
import colorsys


# Load the image
img = cv2.cvtColor(cv2.imread('images.jpg'), cv2.COLOR_BGR2RGB)

# Print shape
height, width, channels = img.shape
print(img.shape)

# Convert the image to a NumPy array
image_array = np.array(img)

#################### PRE-PROCESSING AND THRESHOLDING ####################

gray_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
averaged_image = cv2.blur(gray_image, (5, 5))
# _, thresholded_image = cv2.threshold(averaged_image, 150, 255, cv2.THRESH_BINARY)

cv2.imshow('Image', averaged_image)

# Wait until a key is pressed and then close the image window
cv2.waitKey( )
cv2.destroyAllWindows()


#################### DETECTING CIRCLES ####################

circles = cv2.HoughCircles(
    averaged_image,                   # Input image
    cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
    dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
    minDist=50,                       # Minimum distance between the centers of detected circles
    param1=50,                        # Higher threshold for Canny edge detection
    param2=80,                        # Accumulator threshold for the circle centers at the detection stage
    minRadius=0,                      # Minimum radius of the circles
    maxRadius=0                       # Maximum radius of the circles
)

# Convert the circles' coordinates and radii to integers
if circles is not None:
    circles = np.uint16(np.around(circles))

original_image = img

if circles is not None:
    for circle in circles[0, :]:
        # Extract the center (x, y) and radius (r) of each circle
        center_x, center_y, radius = circle
        
        # Draw the outer circle
        cv2.circle(original_image, (center_x, center_y), radius, (0, 255, 0), 2)
        
        # Draw the center of the circle
        cv2.circle(original_image, (center_x, center_y), 2, (0, 0, 255), 3)

# Display the result
cv2.imshow('Detected Circles', original_image)
cv2.waitKey(0)
cv2.destroyAllWindows()


# # #################### DETERMINING AVERAGE COLOUR OF BALL ####################

# # Option 2: Use OpenCV to select the rectangle area interactively (uncomment below if you want to use it)
# rect = cv2.selectROI("Select ROI", image_array, fromCenter=False, showCrosshair=True)
# x1, y1, w, h = rect
# x2, y2 = x1 + w, y1 + h

# # Crop the selected region from the image
# selected_region = image_array[y1:y2, x1:x2]

# # Calculate the average of each channel (B, G, R)
# average_color = cv2.mean(selected_region)[:3]  # cv2.mean returns (B, G, R, A), so we slice [:3] to ignore alpha
# cv2.destroyAllWindows()

# # Convert the result to a NumPy array
# average_color_array = np.array(average_color)

# # Print the average color values
# print(f"Average color (B, G, R): {average_color_array}")

# # Average color (B, G, R): [183.68149646 216.88725986 152.51971689]
# # = roughly 184, 217, 153

# # Display ball colour
# ball_colour = average_color_array
# square_size = 100  # You can adjust the size of the square
# square_image = np.zeros((square_size, square_size, 3), dtype=np.uint8)

# # Fill the image with the RGB color
# square_image[:] = ball_colour

# # Display the square using OpenCV
# cv2.imshow('RGB Square', square_image)
# cv2.waitKey( )
# cv2.destroyAllWindows()




# # #################### DETERMINING BALL COLOUR ####################

# def is_yellow_or_green(r, g, b):
#     # Convert RGB to HSV
#     h, s, v = colorsys.rgb_to_hsv(r/255.0, g/255.0, b/255.0)
#     h_degrees = h * 360  # Convert hue to degrees
    
#     # Check if the hue is within the yellow or green range
#     if 50 <= h_degrees <= 160:
#         return True, h_degrees, s, v
#     else:
#         return False, h_degrees, s, v

# r, g, b = ball_colour[2],ball_colour[1],ball_colour[0]
# is_within_range, hue, saturation, value = is_yellow_or_green(r, g, b)

# print(f"Is within yellow-green range: {is_within_range}")
# print(f"Hue: {hue:.2f}°, Saturation: {saturation:.2f}, Value: {value:.2f}")
