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

 1. Make sure jetpack 6.2 is installed 
 2. compile https://github.com/FRC-Team-4143/GpuDetectorJNI then https://github.com/FRC-Team-4143/photonvision/tree/jetson-orin
 3. scp service onto jetson
    - `scp -r coprocessor frc5572@192.168.1.134:/home/frc5572/` CHANGE TO HOSTNAME 
 4. move `coprocessor.service` to `/etc/systemd/system/` 
    - `mv coprocessor.service /etc/systemd/system/`
 5. enable service<
    - `sudo systemctl daemon-reload`<br>
      `sudo systemctl enable coprocessor`<br>
      `sudo systemctl start coprocessor`
 6. verify if running
    - `sudo systemctl status coprocessor`


