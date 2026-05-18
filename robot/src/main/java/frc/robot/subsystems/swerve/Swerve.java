package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.localization.DrivetrainState;

public class Swerve extends SubsystemBase {

    private final DrivetrainState drivetrainState;

    public Swerve(DrivetrainState drivetrainState) {
        this.drivetrainState = drivetrainState;
    }

    @Override
    public void periodic() {
        // TODO
    }

    public Pose2d getPose() {
        // TODO
        return null;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        // TODO
        return null;
    }

}
