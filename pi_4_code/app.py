from flask import Flask, Response
import cv2
from picamera2 import Picamera2
import time

# app = Flask(__name__)

# # Initialize the camera (0 for the default camera)
# camera = cv2.VideoCapture(0)

# def generate_frames():
#     while True:
#         # Read a frame from the camera
#         success, frame = camera.read()
#         if not success:
#             break
#         else:
#             # Encode the frame in JPEG format
#             ret, buffer = cv2.imencode('.jpg', frame)
#             frame = buffer.tobytes()  # Convert to bytes

#             # Yield the frame in the required format for streaming
#             yield (b'--frame\r\n'
#                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# @app.route('/video_feed')
# def video_feed():
#     return Response(generate_frames(),
#                     mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/')
# def index():
#     return "Welcome to the Camera Stream! Go to /video_feed to view the feed."

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port=5000)  # Make it accessible on your network

app = Flask(__name__)

# Initialize the picamera
picam2 = Picamera2()
# Time allowance for camera to initialise
time.sleep(2)

# Configure picamera to receive raw bayer format
config = picam2.create_still_configuration(raw={"format": "SRGGB8"})
picam2.configure(config)

#picam2.camera_configuration()['sensor']
#{'bit_depth': 10, 'output_size': (3280, 2464)}


#config = picam2.create_preview_configuration(
#    raw={'format': 'SBGGR8'},
#    #colour_space = "raw",
#    sensor={
#    'output_size':(1640, 1232),
#    'bit_depth': 8
#})

picam2.set_controls({
    "ExposureTime": 50000,  # Adjust based on your lighting conditions
    "AnalogueGain": 2.0,    # Adjust gain as needed
    "AwbEnable": True,     # Disable Auto White Balance
    "ColourGains": (1.5, 1.3)  # Manual color gain adjustments (if needed)    
})

picam2.start()

def generate_frames():
    while True:
    	# Capture raw bayer image and convert into RGB
        raw_bayer = picam2.capture_array("raw")
        bgr_image = cv2.cvtColor(raw_bayer, cv2.COLOR_BayerRG2BGR_VNG)
        # rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    	# Encode the frame in JPEG format
        ret, buffer = cv2.imencode('.jpg', bgr_image)
        frame = buffer.tobytes()  # Convert to bytes

    	# Yield the frame in the required format for streaming
        yield (b'--frame\r\n'
            	b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "Welcome to the Camera Stream! Go to /video_feed to view the feed."

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # Make it accessible on your network
