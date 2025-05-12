from flask import Flask, Response
import cv2
from picamera2 import Picamera2

app = Flask(__name__)

# Initialize the Picamera2 with 4608x2592 still configuration
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (4608, 2592)})
picam2.configure(config)
picam2.start()

@app.route('/')
def capture_image():
    # Capture an image as a numpy array (RGB format)
    frame = picam2.capture_array()
    # Convert RGB to BGR for OpenCV compatibility
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # Encode to JPEG
    ret, jpeg = cv2.imencode('.jpg', frame_bgr)
    if not ret:
        return "Failed to encode image", 500
    # Return JPEG byte stream
    return Response(jpeg.tobytes(), mimetype='image/jpeg')

if __name__ == '__main__':
    # Serve on all interfaces, port 5000
    app.run(host='0.0.0.0', port=5000)
