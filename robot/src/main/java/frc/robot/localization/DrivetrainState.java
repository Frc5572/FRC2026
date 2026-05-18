package frc.robot.localization;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DrivetrainState {

    public void addOdometryObservation(OdometryObservation obs) {
        // TODO
    }

    public void addVisionObservation(VisionObservation obs) {
        // TODO
    }

    public void resetPose(Pose2d pose) {
        // TODO
    }

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        // TODO
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        // TODO
        return null;
    }

    public Pose2d getPose() {
        // TODO
        return null;
    }

    private Optional<Pose2d> sampleAt(double timestamp) {
        // TODO
        return Optional.empty();
    }

}
