package frc.robot.subsystems.swerve.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.CameraConstants;

/**
 * Maintains and updates the robot's estimated global pose for a swerve drive by fusing wheel
 * odometry, gyro measurements, and delayed vision updates.
 *
 * <p>
 * This class tracks two related poses:
 * <ul>
 * <li><b>Odometry pose</b> - derived purely from wheel encoder deltas and gyro yaw</li>
 * <li><b>Estimated pose</b> - a filtered pose that incorporates vision corrections</li>
 * </ul>
 *
 * <p>
 * A time-interpolated pose buffer is maintained to allow vision measurements with latency to be
 * applied retroactively at the correct timestamp.
 *
 * <p>
 * This implementation resembles a lightweight pose estimator rather than a full Kalman filter,
 * using configurable standard deviations to weight vision updates against odometry uncertainty.
 */
@NullMarked
public class SwerveState {

    /** Whether the pose estimator has been initialized from vision */
    private boolean initted = false;

    private final PoseEstimator<SwerveModulePosition[]> visionAdjustedOdometry;

    private final TimeInterpolatableBuffer<Rotation2d> rotationBuffer =
        TimeInterpolatableBuffer.createBuffer(1.5);

    private TimeInterpolatableBuffer<Rotation2d> currentTurretAngle =
        TimeInterpolatableBuffer.createBuffer(1.5);

    /**
     * Creates a new swerve state estimator.
     *
     * @param wheelPositions the initial swerve module positions used to seed odometry
     */
    public SwerveState(SwerveModulePosition[] wheelPositions, Rotation2d gyroYaw) {
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
     * This method updates only the pose estimator / odometry state owned by {@code SwerveState}. It
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
        rotationBuffer.clear();
        rotationBuffer.getInternalBuffer().clear();
    }

    private Optional<Rotation2d> sampleRotationAt(double timestampSeconds) {
        if (rotationBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        double oldestOdometryTimestamp = rotationBuffer.getInternalBuffer().firstKey();
        double newestOdometryTimestamp = rotationBuffer.getInternalBuffer().lastKey();
        if (oldestOdometryTimestamp > timestampSeconds) {
            return Optional.empty();
        }
        timestampSeconds =
            MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        return rotationBuffer.getSample(timestampSeconds);
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
        rotationBuffer.addSample(timestamp, gyroYaw);
        visionAdjustedOdometry.update(gyroYaw, wheelPositions);
    }

    private ChassisSpeeds currentSpeeds;

    /**
     * Updates the robot's current chassis speeds.
     *
     * <p>
     * This method records the most recent robot-relative {@link ChassisSpeeds} command or measured
     * velocity. The stored speeds may be used by future pose estimation or vision fusion logic to
     * account for motion during sensor latency or to improve prediction accuracy.
     *
     * <p>
     * This method does not directly affect the current pose estimate.
     *
     * @param speeds the current robot-relative chassis speeds
     */
    public void updateSpeeds(ChassisSpeeds speeds) {
        currentSpeeds = speeds;
    }

    /**
     * Forcibly initializes the pose estimator using a known robot pose.
     *
     * <p>
     * This method sets the internal odometry/estimator state directly and marks the estimator as
     * initialized, bypassing the normal vision-based initialization path. It is intended for
     * situations where a trusted global pose is already known (for example, at the start of
     * autonomous) or when a vision needs to be tested on a non-compliant field (for example, the
     * practice field at district events).
     *
     * <p>
     * After this method is called, subsequent vision updates will be treated as corrections rather
     * than initialization measurements.
     *
     * @param pose the known robot pose in field coordinates to initialize the estimator with
     */
    public void overrideInit(Pose2d pose) {
        visionAdjustedOdometry.resetPose(pose);
        initted = true;
    }



    private Optional<Transform3d> getRobotToCamera(CameraConstants constants, double time) {
        if (constants.isTurret) {
            Optional<Rotation2d> turretAngleOpt = currentTurretAngle.getSample(time);
            if (turretAngleOpt.isEmpty()) {
                return Optional.empty();
            }
            Rotation3d rotate = new Rotation3d(0.0, 0.0, turretAngleOpt.get().getRadians());

            Transform3d robotToTurret =
                new Transform3d(Constants.Vision.turretCenter.getTranslation(), rotate);

            Transform3d turretToCamera = constants.robotToCamera;
            return Optional.of(robotToTurret.plus(turretToCamera));
        }
        return Optional.of(constants.robotToCamera);
    }

    public void setTurretAngle(double timestamp, Angle angle) {
        currentTurretAngle.addSample(timestamp, new Rotation2d(angle));
    }

    /**
     * Adds a vision measurement using an externally computed camera pose.
     *
     * @param cameraPose estimated camera pose in field coordinates
     * @param robotToCamera transform from robot to camera frame
     * @param translationStdDev translation measurement standard deviation (meters)
     * @param rotationStdDev rotation measurement standard deviation (radians)
     * @param timestamp measurement timestamp in seconds
     */
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

    /**
     * Adds a vision measurement from PhotonVision.
     *
     * <p>
     * If the estimator has not yet been initialized, a valid multi-tag estimate will be used to
     * seed the global pose.
     *
     * @param camera camera configuration constants
     * @param pipelineResult latest PhotonVision pipeline result
     */
    public boolean addVisionObservation(CameraConstants camera,
        PhotonPipelineResult pipelineResult) {
        var multiTag = pipelineResult.getMultiTagResult();
        Optional<Transform3d> robotToCamera =
            getRobotToCamera(camera, pipelineResult.getTimestampSeconds());
        if (robotToCamera.isEmpty()) {
            return false;
        }
        if (!initted) {
            multiTag.ifPresent(multiTag_ -> {
                Transform3d best = multiTag_.estimatedPose.best;
                Pose3d cameraPose =
                    new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
                Pose3d robotPose = cameraPose.plus(robotToCamera.get().inverse());
                visionAdjustedOdometry.resetPose(robotPose.toPose2d());
                initted = true;
            });
            return initted;
        } else {
            double translationSpeed =
                Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
            double rotationSpeed = Math.abs(currentSpeeds.omegaRadiansPerSecond);
            double velocityStdDev = camera.simLatencyStdDev.in(Seconds);
            double velocityTranslationError = translationSpeed * velocityStdDev;
            double velocityRotationError = rotationSpeed * velocityStdDev;
            Logger.recordOutput("State/velocityTranslationError", velocityTranslationError);
            Logger.recordOutput("State/velocityRotationError", velocityRotationError);
            if (multiTag.isPresent()) {
                // Multi Tag
                Transform3d best = multiTag.get().estimatedPose.best;
                Pose3d cameraPose =
                    new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());
                double stdDevMultiplier = stdDevMultiplier(pipelineResult.targets, cameraPose);
                double translationStdDev =
                    stdDevMultiplier * velocityTranslationError + camera.translationError;
                double rotationStdDev =
                    stdDevMultiplier * velocityRotationError + camera.rotationError;
                Logger.recordOutput("State/stdDevMultipler", stdDevMultiplier);
                Logger.recordOutput("State/stdDevTranslation", translationStdDev);
                Logger.recordOutput("State/stdDevRotation", rotationStdDev);
                addVisionObservation(cameraPose, camera.robotToCamera, translationStdDev,
                    rotationStdDev, pipelineResult.getTimestampSeconds());
                return true;
            } else if (rotationSpeed < Units.degreesToRadians(3)) {
                // Single Tag
                PhotonTrackedTarget target = pipelineResult.getBestTarget();
                if (target == null) {
                    return false;
                }
                Optional<Rotation2d> yawSample =
                    sampleRotationAt(pipelineResult.getTimestampSeconds());
                if (!yawSample.isPresent()) {
                    return false;
                }
                Optional<Pose3d> maybePose =
                    Constants.Vision.fieldLayout.getTagPose(target.getFiducialId());
                if (!maybePose.isPresent()) {
                    return false;
                }
                double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                Rotation3d targetInCameraFrame = new Rotation3d(Radians.of(0.0),
                    Degrees.of(-target.getPitch()), Degrees.of(-target.getYaw()));
                Rotation3d cameraRotationInWorldFrame =
                    robotToCamera.get().getRotation().rotateBy(new Rotation3d(yawSample.get()));
                Translation3d debugTranslation =
                    new Pose3d(getGlobalPoseEstimate()).plus(robotToCamera.get()).getTranslation();
                Logger.recordOutput("State/singleTagCameraRotationInWorldFrame",
                    new Pose3d(debugTranslation, cameraRotationInWorldFrame));
                Rotation3d targetRotationInWorldFrame =
                    targetInCameraFrame.plus(cameraRotationInWorldFrame);
                Logger.recordOutput("State/singleTagTargetRotationInWorldFrame",
                    new Pose3d(debugTranslation, targetRotationInWorldFrame));
                Translation3d cameraToTargetInWorldFrame =
                    new Translation3d(distance, targetRotationInWorldFrame);
                Logger.recordOutput("State/singleTagTargetVector", new Translation3d[] {
                    debugTranslation, debugTranslation.plus(cameraToTargetInWorldFrame)});
                Translation2d cameraPosition = maybePose.get().getTranslation()
                    .minus(cameraToTargetInWorldFrame).toTranslation2d();
                Pose3d cameraPose = new Pose3d(cameraPosition.getX(), cameraPosition.getY(),
                    robotToCamera.get().getZ(), cameraRotationInWorldFrame);
                Logger.recordOutput("State/singleTagCameraPose", cameraPose);
                double stdDevMultiplier = stdDevMultiplier(pipelineResult.targets, cameraPose);
                double translationStdDev =
                    stdDevMultiplier * camera.singleTagError + velocityTranslationError;
                Logger.recordOutput("State/stdDevMultipler", stdDevMultiplier);
                Logger.recordOutput("State/stdDevTranslation", translationStdDev);
                addVisionObservation(cameraPose, camera.robotToCamera, translationStdDev,
                    Double.POSITIVE_INFINITY, pipelineResult.getTimestampSeconds());
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

    /**
     * Returns the current best estimate of the robot's global field pose.
     *
     * @return estimated robot pose in field coordinates
     */
    public Pose2d getGlobalPoseEstimate() {
        return visionAdjustedOdometry.getEstimatedPosition();
    }

}
