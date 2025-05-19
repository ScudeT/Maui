import threading
import cv2
from flask import Flask, Response
from picamera2 import Picamera2

camera1 = Picamera2(camera_num=0)
camera2 = Picamera2(camera_num=1)

mode1 = camera1.sensor_modes[0]
mode2 = camera2.sensor_modes[0]

config1 = camera1.create_video_configuration(main={"size": mode1["size"]})
config2 = camera2.create_video_configuration(main={"size": mode2["size"]})

camera1.align_configuration(config1)
camera2.align_configuration(config2)

camera1.configure(config1)
camera2.configure(config2)

camera1.start()
camera2.start()

capture_lock = threading.Lock()

def capture_frame(camera, rotate_code):
    frame = camera.capture_array()
    rotated_frame = cv2.rotate(frame, rotate_code)
    ret, jpeg = cv2.imencode('.jpg', rotated_frame)
    if not ret:
        return None
    return jpeg.tobytes()

app = Flask(__name__)

@app.route('/capture/cam1')
def capture_cam1():
    with capture_lock:
        data = capture_frame(camera1, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if data is None:
        return Response("Failed to capture frame", status=500)
    return Response(data, mimetype='image/jpeg')

@app.route('/capture/cam2')
def capture_cam2():
    with capture_lock:
        data = capture_frame(camera2, cv2.ROTATE_90_CLOCKWISE)
    if data is None:
        return Response("Failed to capture frame", status=500)
    return Response(data, mimetype='image/jpeg')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
