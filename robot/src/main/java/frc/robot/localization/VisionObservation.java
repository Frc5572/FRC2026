package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionObservation(Pose2d robotPose, double xStdDev, double yStdDev,
    double thetaStdDev, double timestamp, String sourceName) {

}
