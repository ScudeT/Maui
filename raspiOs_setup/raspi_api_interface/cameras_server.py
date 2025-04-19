import threading
import io
import base64
import cv2
from flask import Flask, jsonify
from picamera2 import Picamera2

# Initialize both cameras (adjust camera_num if necessary)
camera1 = Picamera2(camera_num=0)
camera2 = Picamera2(camera_num=1)

# Select a sensor mode for each camera.
# (Often the last mode in sensor_modes gives the full active area.)
mode1 = camera1.sensor_modes[-1]
mode2 = camera2.sensor_modes[-1]

# Create a still configuration for each camera that explicitly sets the sensor parameters.
config1 = camera1.create_still_configuration(
    main={"size": mode1["size"]},
    sensor={"output_size": mode1["size"], "bit_depth": mode1["bit_depth"]}
)
config2 = camera2.create_still_configuration(
    main={"size": mode2["size"]},
    sensor={"output_size": mode2["size"], "bit_depth": mode2["bit_depth"]}
)

# Optionally, align the configurations for optimal performance.
camera1.align_configuration(config1)
camera2.align_configuration(config2)

# Configure and start both cameras.
camera1.configure(config1)
camera2.configure(config2)
camera1.start()
camera2.start()

# Lock to synchronize captures from both cameras.
capture_lock = threading.Lock()

def capture_frame1(camera):
    # Capture the current frame from the left camera
    frame = camera.capture_array()
    # Rotate the frame 90 degrees clockwise
    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    ret, buffer = cv2.imencode('.jpg', rotated_frame)
    return buffer if ret else None

def capture_frame2(camera):
    # Capture the current frame from the right camera
    frame = camera.capture_array()
    # Rotate the frame 90 degrees counter clockwise
    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    ret, buffer = cv2.imencode('.jpg', rotated_frame)
    return buffer if ret else None

app = Flask(__name__)

@app.route('/capture', methods=['GET'])
def capture():
    with capture_lock:
        buffer1 = capture_frame1(camera1)
        buffer2 = capture_frame2(camera2)
    
    if buffer1 is None or buffer2 is None:
        return jsonify({"error": "Failed to capture frame(s)"}), 500

    with open("camera1.jpg", "wb") as f:
        f.write(buffer1.tobytes())
    
    with open("camera2.jpg", "wb") as f:
        f.write(buffer2.tobytes())
    
    # Convert JPEG buffers to base64 strings for transmission.
    b64_image1 = base64.b64encode(buffer1).decode('utf-8')
    b64_image2 = base64.b64encode(buffer2).decode('utf-8')
    return jsonify({
        "camera1": b64_image1,
        "camera2": b64_image2
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
