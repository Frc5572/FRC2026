// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.viz.SdfDrawer;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>
     * If you change your main robot class, change the parameter type.
     *
     * @param args String args
     */
    public static void main(String... args) {
        // drawSdfTest();
        RobotBase.startRobot(Robot::new);
    }

    /** Test for interp2d + bilinear gridding to make sure they're correct. */
    @SuppressWarnings("unused")
    private static void drawSdfTest() {
        var res = SdfDrawer.drawSdf(ShotData.distanceRange, ShotData.flywheelRange,
            (t) -> ShotData.distanceFlywheelSpeed.query(t).sdf(), 1080, 1080);
        try {
            ImageIO.write(res, "png", new File("interp2d.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        int[] discretizations = new int[] {20, 60, 100, 120};

        for (var discretization : discretizations) {

            var bil = ShotData.distanceFlywheelSpeed.surrogate(ShotData.distanceRange,
                ShotData.flywheelRange);
            res = SdfDrawer.drawSdf(ShotData.distanceRange, ShotData.flywheelRange,
                (t) -> bil.query(t).sdf(), 1080, 1080);
            try {
                ImageIO.write(res, "png", new File("bilinear" + discretization + ".png"));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
