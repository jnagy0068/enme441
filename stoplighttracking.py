import cv2
import numpy as np
import time
from picamera2 import Picamera2

# HSV range for green
colorLower = (29, 70, 6)
colorUpper = (75, 255, 255)

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "BGR888"
picam2.configure("preview")

picam2.start()
time.sleep(2)  # warmup

while True:
    frame = picam2.capture_array()

    # Blur + HSV
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Mask for green
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours
    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        M = cv2.moments(c)
        if M["m00"] != 0:  # جلوگیری از تقسیم بر صفر
            center = (int(M["m10"] / M["m00"]),
                      int(M["m01"] / M["m00"]))

            if radius > 5:
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # Show frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)

    if key == ord("q"):
        break

cv2.destroyAllWindows()
picam2.stop()
picam2.close()
