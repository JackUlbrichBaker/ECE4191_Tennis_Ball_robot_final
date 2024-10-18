# LIVE CAMERA FEED, EDGE DETECTOR ONLY

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

    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert the image to a NumPy array
    image_array = np.array(img)

    # gray_image = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
    # averaged_image = cv2.blur(gray_image, (10, 10))

    # Split the image into its R, G, B channels
    b, g, r = cv2.split(image_array)

    # Apply Canny edge detection on each channel
    edges_b = cv2.Canny(b, 100, 200)
    edges_g = cv2.Canny(g, 100, 200)
    edges_r = cv2.Canny(r, 100, 200)

    # Combine the edges from each channel
    edges_combined = cv2.bitwise_or(cv2.bitwise_or(edges_b, edges_g), edges_r)

    # Display the resulting frame
    cv2.imshow('Live Camera Feed', edges_combined)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()

'''
NOTES:
- Edge detector works, but it picks up too many edges
- Camera saturation is too low
'''

