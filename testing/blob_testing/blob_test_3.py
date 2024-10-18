#EDGE DETECTION

import cv2
import numpy as np
import colorsys


# Load the image
filename = 'testimg3.jpg'
img = cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2RGB)

# Print shape
height, width, channels = img.shape
print(img.shape)

# Convert the image to a NumPy array
image_array = np.array(img)

gray_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
averaged_image = cv2.blur(gray_image, (10, 10))
# _, thresholded_image = cv2.threshold(averaged_image, 150, 255, cv2.THRESH_BINARY)
cv2.imshow('Original Image', image_array)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Split the image into its R, G, B channels
b, g, r = cv2.split(image_array)

# Apply Canny edge detection on each channel
edges_b = cv2.Canny(b, 100, 200)
edges_g = cv2.Canny(g, 100, 200)
edges_r = cv2.Canny(r, 100, 200)

# Combine the edges from each channel
edges_combined = cv2.bitwise_or(cv2.bitwise_or(edges_b, edges_g), edges_r)


# Display the results
cv2.imshow('Edges Combined', edges_combined)
cv2.waitKey(0)
cv2.destroyAllWindows()

# # Apply Sobel operator to each color channel
# sobel_img = image_array
# sobel_x = cv2.Sobel(sobel_img, cv2.CV_64F, 1, 0, ksize=3)
# sobel_y = cv2.Sobel(sobel_img, cv2.CV_64F, 0, 1, ksize=3)

# # Compute the gradient magnitude
# magnitude = cv2.magnitude(sobel_x, sobel_y)

# # Normalize and convert to 8-bit image
# edges = cv2.convertScaleAbs(magnitude)

# # Display the results
# cv2.imshow('Sobel Edges', edges)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

#################### DETECTING CIRCLES ####################

circles = cv2.HoughCircles(
    edges_combined,                   # Input image
    cv2.HOUGH_GRADIENT,               # Method used (HOUGH_GRADIENT is the only one available)
    dp=1,                             # Inverse ratio of the accumulator resolution to the image resolution
    minDist=200,                       # Minimum distance between the centers of detected circles
    param1=50,                        # Higher threshold for Canny edge detection
    param2=30,                        # Accumulator threshold for the circle centers at the detection stage
    minRadius=0,                      # Minimum radius of the circles
    maxRadius=0                       # Maximum radius of the circles
)

# Convert the circles' coordinates and radii to integers
if circles is not None:
    circles = np.uint16(np.around(circles))

circle_det_image = image_array

if circles is not None:
    for circle in circles[0, :]:
        # Extract the center (x, y) and radius (r) of each circle
        center_x, center_y, radius = circle
        
        # Draw the outer circle
        cv2.circle(circle_det_image, (center_x, center_y), radius, (0, 255, 0), 2)
        
        # Draw the center of the circle
        cv2.circle(circle_det_image, (center_x, center_y), 2, (0, 0, 255), 3)

# Display the result
cv2.imshow('Detected Circles', circle_det_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

'''
NOTES:
- Edge detector works, but it is unreliable
- Doesn't work when lighting is too bright on ball
'''