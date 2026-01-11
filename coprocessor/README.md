# HSV Target Yaw Publisher (FRC)

This program captures video from a camera, detects a specific HSV color range,
computes the yaw angle to the nearest detected pixel, and publishes that yaw
value to NetworkTables for use in FRC robot code.

It is designed to run on a coprocessor (Raspberry Pi, Jetson, etc.) connected to
an FRC robot.

---

## What This Program Does

1. Opens a camera feed (1280×720)
2. Converts frames to HSV
3. Thresholds for a fixed HSV color range
4. Finds the nearest detected pixel to the bottom-left of the image
5. Computes yaw using the camera focal length
6. Publishes the yaw value to NetworkTables

---

## NetworkTables Output

- **Team:** `5572`
- **Table:** `SmartDashboard`
- **Key:** `yaw`
- **Type:** `double`
- **Units:** radians
