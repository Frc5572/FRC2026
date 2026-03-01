// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.viz.DrawColorMap;

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
        // drawImages();
        RobotBase.startRobot(Robot::new);
    }

    private static void drawImages() {
        try {
            DrawColorMap.saveKey("key.png");
            var res = DrawColorMap.draw("dist_flywheel_to_hood.png",
                (t) -> ShotData.distanceFlywheelSpeed.query(t).value().hoodAngleDeg(), 4, 20, 40,
                75);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());
            res = DrawColorMap.draw("dist_flywheel_to_hood_rbf.png",
                (t) -> ShotData.distanceFlywheelToHood.query(t.getX(), t.getY()), 4, 20, 40, 75);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());
            res = DrawColorMap.draw("dist_flywheel_to_tof_rbf.png",
                (t) -> ShotData.distanceFlywheelToTof.query(t.getX(), t.getY()), 4, 20, 40, 75);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());
            res = DrawColorMap.draw("dist_flywheel_to_tof.png",
                (t) -> ShotData.distanceFlywheelSpeed.query(t).value().timeOfFlight(), 4, 20, 40,
                75);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());
            res = DrawColorMap.draw("dist_flywheel_sdf.png",
                (t) -> ShotData.distanceFlywheelSpeed.query(t).sdf(), 4, 20, 40, 75);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());

            res = DrawColorMap.draw("flywheel_hood_to_dist.png",
                (t) -> ShotData.flywheelSpeedHoodAngle.query(t).value().distanceMeters(), 40, 75, 0,
                30);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());
            res = DrawColorMap.draw("flywheel_hood_to_tof.png",
                (t) -> ShotData.flywheelSpeedHoodAngle.query(t).value().timeOfFlight(), 40, 75, 0,
                30);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());
            res = DrawColorMap.draw("flywheel_hood_to_horizontal_vel.png",
                (t) -> ShotData.flywheelSpeedHoodAngle.query(t).value().horizontalVelocity(), 40,
                75, 0, 30);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());
            res = DrawColorMap.draw("flywheel_hood_sdf.png",
                (t) -> ShotData.flywheelSpeedHoodAngle.query(t).sdf(), 40, 75, 0, 30);
            System.out.println("minValue: " + res.getFirst());
            System.out.println("maxValue: " + res.getSecond());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
