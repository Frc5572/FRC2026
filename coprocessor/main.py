#!/usr/bin/env python3
import math
import sys

import cv2
import ntcore
import numpy as np


print("camera matrix")
camera_matrix = np.array(
    [
        [940.7360710926395, 0, 615.5884770322365],
        [0, 939.9932393907364, 328.53938300868],
        [0, 0, 1],
    ],
)

print("dist coeffs")
dist_coeffs = np.array(
    [
        0.054834081023049625,
        -0.15994111706817074,
        -0.0017587106009926158,
        -0.0014671022483263552,
        0.049742166267499596,
    ]
)


f_x = camera_matrix[0, 0]

TEAM = 5572
TABLE = "ColorPI"
KEY = "yaw"

try:
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable(TABLE)
    pub_yaw = table.getDoubleTopic(KEY).publish()
    pub_sees_yellow = table.getBooleanTopic("seesYellow").publish()
    pub_error = table.getStringTopic("error").publish()
    inst.startClient4("pi-color-client")
    inst.setServerTeam(TEAM)
except Exception as e:
    print(f"Error initializing NetworkTables: {e}")
    sys.exit(1)


def calculate_yaw(x, f_x):
    try:
        center_x = 720 / 2
        return math.atan((x - center_x) / f_x)
    except (ZeroDivisionError, TypeError) as e:
        print(f"Error in calculate_yaw: {e}")
        pub_error.set(e)
        return None


cap = cv2.VideoCapture(
    "/dev/v4l/by-path/platform-5200000.usb-usbv2-0:1:1.0-video-index0"
)
print("video start")
if not cap.isOpened():
    print("Cannot open camera")
    pub_error.set("Cannot open camera")
    sys.exit(1)

try:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("set w/h")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            continue

        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, (16, 123, 169), (32, 156, 244))
            height, width = frame.shape[:2]
            start_x, start_y = 0, height - 1
            output = cv2.bitwise_and(frame, frame, mask=mask1)
            colored_pixels = np.where(output > 0)
            if len(colored_pixels[0]) > 0:
                distances = np.sqrt(
                    (colored_pixels[1] - start_x) ** 2
                    + (colored_pixels[0] - start_y) ** 2
                )

                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                eroded = cv2.erode(output, kernel, iterations=2)
                nearest_idx = np.argmin(distances)
                nearest_x = colored_pixels[1][nearest_idx]
                nearest_y = colored_pixels[0][nearest_idx]
                nearest_distance = distances[nearest_idx]

                yaw = calculate_yaw(nearest_x, f_x)
                if yaw is not None:
                    pub_sees_yellow.set(True)
                    pub_yaw.set(yaw)
                    print(yaw, True)
                else:
                    pub_sees_yellow.set(False)
            else:
                pub_sees_yellow.set(False)

        except Exception as e:
            print(f"Error processing frame: {e}")
            pub_error.set(str(e))
            continue
        if cv2.waitKey(1) == ord("q"):
            break


except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print(f"Unexpected error in main loop: {e}")
    pub_error.set(str(e))
    sys.exit(1)
finally:
    cap.release()
    cv2.destroyAllWindows()
