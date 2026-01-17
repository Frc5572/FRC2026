#!/usr/bin/env python3
import math
import time

import cv2
import ntcore
import numpy as np


def calculate_yaw(
    x,
    f_x,
):
    try:
        x /= 2
        x -= 720
        return math.atan(x / f_x)
    except (ZeroDivisionError, TypeError) as e:
        print(f"Error in calculate_yaw: {e}")
        return None


print("camera matrix")
camera_matrix = np.array(
    [
        [1057.3980959579437, 0, 656.8556544891763],
        [0, 1058.2448544114698, 375.100928453989],
        [0, 0, 1],
    ]
)

print("dist coeffs")
dist_coeffs = np.array(
    [
        0.03932968718495995,
        -0.05757208162810665,
        -0.0016503334944551454,
        -0.000619417970035607,
        -0.016908101094226472,
    ]
)

principal_point = (camera_matrix[0, 2], camera_matrix[1, 2])

f_x = camera_matrix[0, 0]

TEAM = 5572
TABLE = "ColorPI"
KEY = "yaw"

try:
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable(TABLE)
    pub_yaw = table.getDoubleTopic(KEY).publish()
    pub_sees_yellow = table.getBooleanTopic("seesYellow").publish()
    inst.startClient4("pi-color-client")
    inst.setServerTeam(TEAM)
    inst.startDSClient()
except Exception as e:
    print(f"Error initializing NetworkTables: {e}")
    exit(1)


cap = cv2.VideoCapture(0)
print("video start")
if not cap.isOpened():
    print("Cannot open camera")
    exit(1)

try:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("set w/h")
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, (17, 45, 144), (19, 184, 245))
            height, width = frame.shape[:2]
            start_x, start_y = 0, height - 1
            output = cv2.bitwise_and(frame, frame, mask=mask1)
            colored_pixels = np.where(output > 0)

            if len(colored_pixels[0]) > 0:
                distances = np.sqrt(
                    (colored_pixels[1] - start_x) ** 2
                    + (colored_pixels[0] - start_y) ** 2
                )

                nearest_idx = np.argmin(distances)
                nearest_x = colored_pixels[1][nearest_idx]
                nearest_y = colored_pixels[0][nearest_idx]
                nearest_distance = distances[nearest_idx]
                yaw = calculate_yaw(nearest_x, f_x)
                if yaw is not None:
                    pub_sees_yellow.set(True)
                    pub_yaw.set(yaw)
                else:
                    pub_sees_yellow.set(False)
            else:
                pub_sees_yellow.set(False)

        except Exception as e:
            print(f"Error processing frame: {e}")
            continue

        if cv2.waitKey(1) == ord("q"):
            break

except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print(f"Unexpected error in main loop: {e}")
finally:
    cap.release()
    cv2.destroyAllWindows()
