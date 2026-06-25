package frc.robot.localization;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.math.geometry.Rectangle;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.SwerveArcOdometry;

/** Total state of the robot */
public class DrivetrainState {

    /** Whether the pose estimator has been initialized from vision */
    private boolean initted = false;

    private final PoseEstimator<SwerveModulePosition[]> visionAdjustedOdometry;

    private final TimeInterpolatableBuffer<Rotation2d> currentTurretAngle =
        TimeInterpolatableBuffer.createBuffer(1.5);

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

    /**
     * Resets the internal pose estimate to a known field pose.
     *
     * <p>
     * This method forces the underlying swerve odometry to the specified pose, effectively
     * redefining the robot's position on the field. It should be used when the robot pose is known
     * with high confidence, such as:
     * <ul>
     * <li>At the start of autonomous</li>
     * <li>After a field-aligned reset</li>
     * <li>Following a trusted vision-based localization event</li>
     * </ul>
     *
     * <p>
     * This method updates only the pose estimator / odometry state owned by {@code RobotState}. It
     * does <b>not</b> update any associated simulation state or drivetrain model.
     *
     * <p>
     * Most code should prefer {@link Swerve#overridePose} when resetting the robot pose, as that
     * method ensures both the estimator and any simulated drivetrain pose remain consistent.
     *
     * <p>
     * Future odometry and vision updates will be applied relative to this new pose.
     *
     * @param pose the desired robot pose in field coordinates
     */
    public void resetPose(Pose2d pose) {
        this.visionAdjustedOdometry.resetPose(pose);
    }

    public void resetInit() {
        this.initted = false;
    }

    /**
     * Updates odometry and pose estimates using swerve module encoders and an optional gyro
     * measurement.
     *
     * @param wheelPositions current swerve module positions
     * @param gyroYaw current robot yaw, if available
     * @param timestamp measurement timestamp in seconds
     */
    public void addOdometryObservation(SwerveModulePosition[] wheelPositions, Rotation2d gyroYaw,
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

    /**
     * Forcibly initializes the pose estimator using a known robot pose.
     *
     * @param pose the known robot pose in field coordinates to initialize the estimator with
     */
    public void overrideInit(Pose2d pose) {
        visionAdjustedOdometry.resetPose(pose);
        initted = true;
    }

    /** Get robot to camera for camera mounted on the turret */
    public Optional<Transform3d> getTurretRobotToCamera(Transform3d turretToCamera,
        double timestamp) {
        var maybeRotation = currentTurretAngle.getSample(timestamp);
        if (maybeRotation.isEmpty()) {
            return Optional.empty();
        }
        var turretRotation = maybeRotation.get();
        Rotation3d rotate = new Rotation3d(0.0, 0.0, turretRotation.getRadians());

        Transform3d robotToTurret =
            new Transform3d(Constants.Vision.turretCenter.getTranslation(), rotate);

        Transform3d robotToCamera = robotToTurret.plus(turretToCamera);

        return Optional.of(robotToCamera);
    }

    private double prevAngle;

    /** Set the current turret angle */
    public void setTurretRawAngle(double timestamp, Angle angle) {
        var angleDeg = angle.in(Degrees);
        if (Math.abs(angleDeg - prevAngle) > 2) {
            Logger.recordOutput("State/stationary/turret", true);
            this.lastTimeMoved = MathSharedStore.getTimestamp();
        } else {
            Logger.recordOutput("State/stationary/turret", false);
        }
        prevAngle = angleDeg;
        currentTurretAngle.addSample(timestamp, new Rotation2d(angle));
        Translation2d[] turretDirection = new Translation2d[2];
        turretDirection[0] = getTurretCenterFieldFrame().getTranslation();
        turretDirection[1] =
            getTurretCenterFieldFrame().getTranslation().plus(new Translation2d(2.0,
                getGlobalPoseEstimate().getRotation().plus(new Rotation2d(angle))));
        Logger.recordOutput("State/TurretDirection", turretDirection);
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


    private static double stdDevMultiplier(List<PhotonTrackedTarget> targets, Pose3d cameraPose) {
        double totalDistance = 0.0;
        int count = 0;
        for (var tag : targets) {
            var maybeTagPose = Constants.Vision.fieldLayout.getTagPose(tag.getFiducialId());
            if (maybeTagPose.isPresent()) {
                var tagPose = maybeTagPose.get();
                totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                count++;
            }
        }
        double avgDistance = totalDistance / count;
        double stddev = Math.pow(avgDistance, 2.0) / count;
        return stddev;
    }

    /**
     * Returns the current best estimate of the robot's global field pose.
     *
     * @return estimated robot pose in field coordinates
     */
    public Pose2d getGlobalPoseEstimate() {
        return visionAdjustedOdometry.getEstimatedPosition();
    }

    public Pose2d getTurretCenterFieldFrame() {
        return getGlobalPoseEstimate().plus(new Transform2d(
            Constants.Vision.turretCenter.toPose2d().getTranslation(), Rotation2d.kZero));
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return currentSpeeds;
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
}
