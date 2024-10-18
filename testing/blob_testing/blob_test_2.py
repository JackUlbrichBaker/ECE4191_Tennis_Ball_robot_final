# EXPLORING COLOUR DETECTION

import cv2
import numpy as np
import colorsys


# Load the image
filename = 'testing\blob_testing\colourtestimg.jpg'
img = cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2RGB)

# Print shape
height, width, channels = img.shape
print(img.shape)

# Convert the image to a NumPy array
image_array = np.array(img)

gray_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
averaged_image = cv2.blur(gray_image, (10, 10))
_, thresholded_image = cv2.threshold(averaged_image, 150, 255, cv2.THRESH_BINARY)
cv2.imshow('Original Image', thresholded_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

#################### CONTOUR FINDER ####################

# # Apply thresholding or Canny edge detection
# # Example: Using Canny edge detection
# edges = cv2.Canny(thresholded_image, 100, 200)

# # Find contours
# contours, hierarchy = cv2.findContours(
#     edges,                            # Binary image
#     cv2.RETR_TREE,                    # Contour retrieval mode
#     cv2.CHAIN_APPROX_SIMPLE           # Contour approximation method
# )

# # Draw all contours on the original image (for visualization)
# image_with_contours = cv2.cvtColor(thresholded_image, cv2.COLOR_GRAY2BGR)  # Convert to BGR for color drawing
# cv2.drawContours(image_with_contours, contours, -1, (0, 255, 0), 2)

# # Display the results
# cv2.imshow('Edges', edges)
# cv2.imshow('Contours', image_with_contours)
# cv2.waitKey(0)
# cv2.destroyAllWindows()



# # #################### DIFFERENCE OF GAUSSIANS EDGE DETECTOR ####################

# image = averaged_image

# # Apply Gaussian blur with two different sigma values
# sigma1 = 1.0
# sigma2 = 2.0

# blur1 = cv2.GaussianBlur(image, (0, 0), sigma1)
# blur2 = cv2.GaussianBlur(image, (0, 0), sigma2)

# # Subtract the two blurred images
# dog = blur1 - blur2

# # Normalize the result to ensure it is within the correct range
# dog_normalized = cv2.normalize(dog, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

# # Display the results
# cv2.imshow('Original Image', image)
# cv2.imshow('Difference of Gaussians', dog_normalized)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# #################### DETERMINING BALL COLOUR AND CREATING A MASK ####################

hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
# Define the lower and upper bounds for yellow in HSV space
green_lower_bound = np.array([35, 50, 50])  # Lower bound for green
green_upper_bound = np.array([65, 255, 255])  # Upper bound for green

yellow_lower_bound = np.array([25, 50, 50])  # Lower bound for yellow
yellow_upper_bound = np.array([40, 255, 255])  # Upper bound for yellow

# Create masks for yellow and green
yellow_mask = cv2.inRange(hsv_img, yellow_lower_bound, yellow_upper_bound)
green_mask = cv2.inRange(hsv_img, green_lower_bound, green_upper_bound)

# Combine the masks using bitwise OR
combined_mask = cv2.bitwise_or(yellow_mask, green_mask)

# Apply the combined mask to the original image
result = cv2.bitwise_and(img, img, mask=combined_mask)

# Display the results
cv2.imshow('Original Image', img)
cv2.imshow('Combined Mask', combined_mask)
cv2.imshow('Thresholded Image (Yellow + Green)', result)
cv2.waitKey(0)
cv2.destroyAllWindows()

'''
NOTES:
- Applying a colour mask on the ball (by detecting yellow/green) works on high res images but not on the test images we took
'''