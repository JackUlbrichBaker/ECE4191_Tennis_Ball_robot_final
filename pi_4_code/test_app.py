from picamera2 import Picamera2
import time

# Initialize the camera
picam2 = Picamera2()

# Start the camera
picam2.start()

# Allow the camera to warm up for a couple of seconds
time.sleep(2)

# Capture an image and save it directly to a file
picam2.capture_file("captured_image.jpg")

# Stop the camera
picam2.stop()

print("Image saved as 'captured_image.jpg'")
