package frc.robot.localization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometryObservation(SwerveModulePosition[] wheelPositions, Rotation2d gyroReading,
    double timestamp) {

}
