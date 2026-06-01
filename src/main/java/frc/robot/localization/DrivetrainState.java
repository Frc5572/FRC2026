package frc.robot.localization;

import static edu.wpi.first.units.Units.Meters;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.math.geometry.Rectangle;
import frc.robot.subsystems.swerve.util.SwerveArcOdometry;

public class DrivetrainState {
    /** Whether the pose estimator has been initialized from vision */
    private boolean initted = false;
    private final PoseEstimator<SwerveModulePosition[]> visionAdjustedOdometry;
    private Rotation2d gyroOffset = Rotation2d.kZero;
    private Rotation2d prevGyroReading = Rotation2d.kZero;
    private ChassisSpeeds currentSpeeds;
    private double lastTimeMoved = 0.0;

    /**
     * Creates a new swerve state estimator.
     *
     * @param wheelPositions the initial swerve module positions used to seed odometry
     * @param gyroYaw the initial reported gyro yaw
     */
    public DrivetrainState(SwerveModulePosition[] wheelPositions, Rotation2d gyroYaw) {
        prevGyroReading = gyroYaw;
        SwerveDriveOdometry swerveOdometry =
            new SwerveArcOdometry(Constants.Swerve.swerveKinematics, gyroYaw, wheelPositions);
        visionAdjustedOdometry = new PoseEstimator<>(Constants.Swerve.swerveKinematics,
            swerveOdometry, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9));
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroYaw,
        double timestamp) {
        prevGyroReading = gyroYaw;
        Logger.recordOutput("State/prevRot", getGlobalPoseEstimate().getRotation());
        var before = getGlobalPoseEstimate();
        visionAdjustedOdometry.update(gyroYaw.minus(gyroOffset), wheelPositions);
        var after = getGlobalPoseEstimate();
        if (FieldConstants.isOnBump(before)) {
            Logger.recordOutput("State/isOnBump", true);
            var diff = after.minus(before);
            diff = new Transform2d(diff.getX() * 0.6, diff.getY(), diff.getRotation());
            if (RobotBase.isReal()) {
                visionAdjustedOdometry.resetPose(before.plus(diff));
            }
        } else {
            Logger.recordOutput("State/isOnBump", false);
        }
        Logger.recordOutput("State/nextRot", getGlobalPoseEstimate().getRotation());
        if (Constants.keepInField) {
            limitPosition(getGlobalPoseEstimate(), visionAdjustedOdometry::resetPose);
        }
    }

    public void resetPose(Pose2d pose) {
        this.visionAdjustedOdometry.resetPose(pose);
    }

    /**
     * Returns the current best estimate of the robot's global field pose.
     *
     * @return estimated robot pose in field coordinates
     */
    public Pose2d getGlobalPoseEstimate() {
        return visionAdjustedOdometry.getEstimatedPosition();
    }

    private final Rectangle robotRect = new Rectangle("pose", Pose2d.kZero,
        Constants.Swerve.bumperFront.in(Meters) * 2, Constants.Swerve.bumperRight.in(Meters) * 2);

    /**
     * limits position of a given pose
     *
     * @param pose new pose of robot reactangle
     * @param resetPose reset pose
     */
    public void limitPosition(Pose2d pose, Consumer<Pose2d> resetPose) {
        robotRect.setPose(pose);
        double offsetX = 0.0;
        double offsetY = 0.0;
        var corners = robotRect.getCorners();
        for (var corner : corners) {
            if (corner.getX() < 0) {
                offsetX = Math.max(offsetX, -corner.getX());
            }
            if (corner.getX() > FieldConstants.fieldLength) {
                offsetX = Math.min(offsetX, FieldConstants.fieldLength - corner.getX());
            }
            if (corner.getY() < 0) {
                offsetY = Math.max(offsetY, -corner.getY());
            }
            if (corner.getY() > FieldConstants.fieldWidth) {
                offsetY = Math.min(offsetY, FieldConstants.fieldWidth - corner.getY());
            }
        }

        if (Math.abs(offsetX) > 1e-3 || Math.abs(offsetY) > 1e-3) {
            resetPose.accept(
                new Pose2d(pose.getX() + offsetX, pose.getY() + offsetY, pose.getRotation()));
        }
    }

    public boolean isInitted() {
        return initted;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return currentSpeeds;
    }

    /**
     * Forcibly initializes the pose estimator using a known robot pose.
     *
     * @param pose the known robot pose in field coordinates to initialize the estimator with
     */
    public void overrideInit(Pose2d pose) {
        visionAdjustedOdometry.resetPose(pose);
        initted = true;
    }

    /**
     * Updates the robot's current chassis speeds.
     *
     * @param speeds the current robot-relative chassis speeds
     */
    public void updateMeasuredSpeeds(ChassisSpeeds speeds) {
        this.currentSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGlobalPoseEstimate().getRotation());
        Logger.recordOutput("State/currentSpeeds", this.currentSpeeds);
        if (Math.abs(this.currentSpeeds.vxMetersPerSecond) > 0.3
            || Math.abs(this.currentSpeeds.vyMetersPerSecond) > 0.3
            || Math.abs(this.currentSpeeds.omegaRadiansPerSecond) > Units.degreesToRadians(10)) {
            this.lastTimeMoved = MathSharedStore.getTimestamp();
            Logger.recordOutput("State/stationary/speeds", true);
        } else {
            Logger.recordOutput("State/stationary/speeds", false);
        }
    }

    /** Add potentially asequent observation from camera */
    public void addVisionObservation(VisionObservation observations) {
        Pose2d robotPose =
            observations.cameraPose().plus(observations.robotToCamera().inverse()).toPose2d();
        Pose2d before = visionAdjustedOdometry.getEstimatedPosition();
        visionAdjustedOdometry.addVisionMeasurement(robotPose, observations.timestamp(),
            observations.getStdDev());
        Pose2d after = visionAdjustedOdometry.getEstimatedPosition();
        double correction = after.getTranslation().getDistance(before.getTranslation());
        Logger.recordOutput("State/Correction", correction);
        Logger.recordOutput("State/VisionRobotPose", robotPose);
    }
}

