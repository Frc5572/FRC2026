package frc.robot.targeting;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TargetingState {

    private final Supplier<Pose2d> robotPose;
    private final DoubleSupplier flywheelSpeedRps;

    public TargetingState(Supplier<Pose2d> robotPose, DoubleSupplier flywheelSpeedRps) {
        this.robotPose = robotPose;
        this.flywheelSpeedRps = flywheelSpeedRps;
    }

    public void periodic() {
        // TODO
    }

    public Rotation2d getDesiredTurretHeading() {
        // TODO
        return null;
    }

    public double getDesiredFlywheelRps() {
        // TODO
        return 0.0;
    }

    public double getDesiredHoodAngleDeg() {
        // TODO
        return 0.0;
    }

    public boolean isOkayToShoot() {
        // TODO
        return false;
    }

}
