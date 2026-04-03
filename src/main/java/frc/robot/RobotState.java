package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
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
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.SwerveArcOdometry;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.util.AllianceFlipUtil;


public class RobotState {

    /** Whether the pose estimator has been initialized from vision */
    private boolean initted = false;

    private final PoseEstimator<SwerveModulePosition[]> visionAdjustedOdometry;

    private final TimeInterpolatableBuffer<Rotation2d> currentTurretAngle =
        TimeInterpolatableBuffer.createBuffer(1.5);

    private Rotation2d gyroOffset = Rotation2d.kZero;
    private Rotation2d prevGyroReading = Rotation2d.kZero;

    private ChassisSpeeds currentSpeeds;

    /**
     * Creates a new swerve state estimator.
     *
     * @param wheelPositions the initial swerve module positions used to seed odometry
     * @param gyroYaw the initial reported gyro yaw
     */
    public RobotState(SwerveModulePosition[] wheelPositions, Rotation2d gyroYaw) {
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
        visionAdjustedOdometry.update(gyroYaw.minus(gyroOffset), wheelPositions);
        Logger.recordOutput("State/nextRot", getGlobalPoseEstimate().getRotation());
    }

    /**
     * Updates the robot's current chassis speeds.
     *
     * @param speeds the current robot-relative chassis speeds
     */
    public void updateMeasuredSpeeds(ChassisSpeeds speeds) {
        this.currentSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGlobalPoseEstimate().getRotation());
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

    private static Transform3d getTurretRobotToCamera(Transform3d turretToCamera,
        Rotation2d turretRotation) {
        Rotation3d rotate = new Rotation3d(0.0, 0.0, turretRotation.getRadians());

        Transform3d robotToTurret =
            new Transform3d(Constants.Vision.turretCenter.getTranslation(), rotate);

        Transform3d robotToCamera = robotToTurret.plus(turretToCamera);

        return robotToCamera;
    }

    public void setTurretRawAngle(double timestamp, Angle angle) {
        currentTurretAngle.addSample(timestamp, new Rotation2d(angle));
    }

    public void addVisionObservation(Pose3d cameraPose, Transform3d robotToCamera,
        double translationStdDev, double rotationStdDev, double timestamp) {
        Pose2d robotPose = cameraPose.plus(robotToCamera.inverse()).toPose2d();
        Pose2d before = visionAdjustedOdometry.getEstimatedPosition();
        visionAdjustedOdometry.addVisionMeasurement(robotPose, timestamp,
            VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev));
        Pose2d after = visionAdjustedOdometry.getEstimatedPosition();
        double correction = after.getTranslation().getDistance(before.getTranslation());
        Logger.recordOutput("State/Correction", correction);
        Logger.recordOutput("State/VisionRobotPose", robotPose);
    }

    public boolean addVisionObservation(CameraConstants camera,
        PhotonPipelineResult pipelineResult) {
        var multiTag = pipelineResult.getMultiTagResult();
        Transform3d robotToCamera_ = camera.robotToCamera;
        double translationSpeed =
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double rotationSpeed = Math.abs(currentSpeeds.omegaRadiansPerSecond);
        if (camera.isTurret) {
            var maybeTurretRotation =
                currentTurretAngle.getSample(pipelineResult.getTimestampSeconds());
            var maybeTurretRotationM1 =
                currentTurretAngle.getSample(pipelineResult.getTimestampSeconds() - 0.1);
            var maybeTurretRotationP1 =
                currentTurretAngle.getSample(pipelineResult.getTimestampSeconds() + 0.1);
            if (maybeTurretRotation.isEmpty() || maybeTurretRotationM1.isEmpty()
                || maybeTurretRotationP1.isEmpty()) {
                return false;
            }
            if (Math.abs(angleDiff(maybeTurretRotationM1.get(), maybeTurretRotationP1.get())
                .in(Degrees)) > 5) {
                return false;
            }
            robotToCamera_ = getTurretRobotToCamera(robotToCamera_, maybeTurretRotation.get());
        }
        if (!initted) {
            final Transform3d robotToCamera = robotToCamera_;
            multiTag.ifPresent(multiTag_ -> {
                Transform3d best = multiTag_.estimatedPose.best;
                Pose3d cameraPose =
                    new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
                Pose3d robotPose = cameraPose.plus(robotToCamera.inverse());
                // reading - gyroOffset = actual
                // gyroOffset = reading - actual
                gyroOffset = prevGyroReading.minus(robotPose.toPose2d().getRotation());
                visionAdjustedOdometry.resetPose(robotPose.toPose2d());
                Logger.recordOutput("State/initPose", getGlobalPoseEstimate());
                currentTurretAngle.clear();
                initted = true;
            });
            return initted;
        } else {
            double velocityStdDev = camera.simLatencyStdDev.in(Seconds);
            double velocityTranslationError = translationSpeed * velocityStdDev;
            double velocityRotationError = rotationSpeed * velocityStdDev;
            Logger.recordOutput("State/velocityTranslationError", velocityTranslationError);
            Logger.recordOutput("State/velocityRotationError", velocityRotationError);

            var bestTarget = pipelineResult.hasTargets() ? pipelineResult.getBestTarget() : null;
            if (bestTarget == null) {
                return false;
            }
            var bestTagPose = Constants.Vision.fieldLayout.getTagPose(bestTarget.getFiducialId());
            if (bestTagPose.isEmpty()) {
                return false;
            }

            if (multiTag.isPresent()) {
                // Multi Tag
                Transform3d best = multiTag.get().estimatedPose.best;
                Pose3d cameraPose =
                    new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
                Logger.recordOutput("State/Camera/" + camera.name + "/cameraPose", cameraPose);
                Logger.recordOutput("State/Camera/" + camera.name + "/correctedCameraPose",
                    cameraPose);
                Pose3d estRobotPose = cameraPose.plus(robotToCamera_.inverse());
                Logger.recordOutput("State/Camera/" + camera.name + "/estRobotPose", estRobotPose);
                double stdDevMultiplier = stdDevMultiplier(pipelineResult.targets, cameraPose);
                double translationStdDev =
                    stdDevMultiplier * velocityTranslationError + camera.translationError;
                double rotationStdDev =
                    stdDevMultiplier * velocityRotationError + camera.rotationError;
                Logger.recordOutput("State/Camera/" + camera.name + "/stdDevMultipler",
                    stdDevMultiplier);
                Logger.recordOutput("State/Camera/" + camera.name + "/stdDevTranslation",
                    translationStdDev);
                Logger.recordOutput("State/Camera/" + camera.name + "/stdDevRotation",
                    rotationStdDev);
                addVisionObservation(cameraPose, robotToCamera_, translationStdDev, rotationStdDev,
                    pipelineResult.getTimestampSeconds());
                return true;
            }
        }
        return false;
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

    private static Angle angleDiff(Rotation2d a, Rotation2d b) {
        double diff = (b.getRotations() - a.getRotations() + 0.5);
        diff = diff - Math.floor(diff) - 0.5;
        return Rotations.of(diff < -0.5 ? diff + 1.0 : diff);
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

    private Translation2d shootingTarget = FieldConstants.Hub.centerHub;
    private boolean targetIsGround = false;

    private void updateShootingTarget() {
        Pose2d bluePose = AllianceFlipUtil.apply(getGlobalPoseEstimate());
        if (bluePose.getX() > FieldConstants.Hub.centerHub.getX()) {
            targetIsGround = true;
            if (bluePose.getY() > FieldConstants.fieldWidth / 2) {
                shootingTarget = AllianceFlipUtil.apply(new Translation2d(
                    FieldConstants.Hub.centerHub.getX() / 2, (3 * FieldConstants.fieldWidth / 4)));
            } else {
                shootingTarget = AllianceFlipUtil.apply(new Translation2d(
                    FieldConstants.Hub.centerHub.getX() / 2, (FieldConstants.fieldWidth / 4)));
            }
            shootingTarget = AllianceFlipUtil.apply(FieldConstants.Hub.centerHub);
        } else {
            targetIsGround = false;
            shootingTarget = AllianceFlipUtil.apply(FieldConstants.Hub.centerHub);
        }

        Logger.recordOutput("State/ShootingTarget", shootingTarget);
        Logger.recordOutput("State/TargetIsGround", targetIsGround);
    }

    public void updateTargeting() {
        updateShootingTarget();
    }
}
