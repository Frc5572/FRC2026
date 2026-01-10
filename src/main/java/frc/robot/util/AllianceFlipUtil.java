package frc.robot.util;

import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/** Utilities for flipping based on alliance */
@NullMarked
public class AllianceFlipUtil {
    public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
    public static double fieldLength = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

    /** Possibly flip */
    public static double applyX(double x) {
        return shouldFlip() ? fieldLength - x : x;
    }

    /** Possibly flip */
    public static double applyY(double y) {
        return shouldFlip() ? fieldWidth - y : y;
    }

    /** Possibly flip */
    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    /** Possibly flip */
    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    /** Possibly flip */
    public static Pose2d apply(Pose2d pose) {
        return shouldFlip() ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
            : pose;
    }

    /** Possibly flip */
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

}
