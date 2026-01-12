# HSV Target Yaw Publisher (FRC)

This program captures video from a camera, detects a specific HSV color range,
computes the yaw angle to the nearest detected pixel. Runs on [OPI zero 3](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-Zero-3.html) 

---

## What This Program Does

1. Opens a camera feed (1280×720)
2. Converts frames to HSV
3. Thresholds for a fixed HSV color range
4. Finds the nearest detected pixel to the bottom-left of the image
5. Computes yaw using the camera focal length
6. Publishes the yaw and if a color is detected to NetworkTables

---

## How to use

 1. flash OPI zero 3 with [this image](https://drive.google.com/file/d/1PwQO2kPwfo2WdLOxZBkQHVTuooV64KH3/view?usp=drive_link)
 2. clone coprossesor code onto pi
 3. create virtual enviroment `python3 -m venv .venv`
 3. install dependaces `pip install -r requirements.txt`


