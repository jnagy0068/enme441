import cv2
import numpy as np
import time
from picamera2 import Picamera2

colorLower = (29, 70, 6)
colorUpper = (75, 255, 255)

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.configure("preview")

picam2.start()
time.sleep(2)

# Set FPS
fps = 20.0
duration = 30  # seconds
total_frames = int(fps * duration)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('stoplight.mp4', fourcc, fps, (640, 480))

frame_count = 0

while frame_count < total_frames:
    frame = picam2.capture_array()

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        M = cv2.moments(c)
        if M["m00"] != 0:
            center = (int(M["m10"] / M["m00"]),
                      int(M["m01"] / M["m00"]))

            if radius > 5:
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

    out.write(frame)
    frame_count += 1

    cv2.imshow("Recording", frame)
    if cv2.waitKey(1) == ord('q'):
        break

out.release()
cv2.destroyAllWindows()
picam2.stop()
picam2.close()

print("✅ Exact 30-second video saved")
