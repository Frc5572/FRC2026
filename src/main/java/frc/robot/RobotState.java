package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
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
import frc.robot.math.geometry.Rectangle;
import frc.robot.shotdata.ShotData;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.SwerveArcOdometry;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.util.AllianceFlipUtil;

/** Total state of the robot */
public class RobotState {

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
        var before = getGlobalPoseEstimate();
        visionAdjustedOdometry.update(gyroYaw.minus(gyroOffset), wheelPositions);
        var after = getGlobalPoseEstimate();
        if (FieldConstants.isOnBump(before)) {
            var diff = after.minus(before).times(0.6);
            visionAdjustedOdometry.resetPose(before.plus(diff));
        }
        Logger.recordOutput("State/nextRot", getGlobalPoseEstimate().getRotation());
        limitPosition(getGlobalPoseEstimate(), visionAdjustedOdometry::resetPose);
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
        if (Math.abs(this.currentSpeeds.vxMetersPerSecond) > 1e-2
            || Math.abs(this.currentSpeeds.vyMetersPerSecond) > 1e-2
            || Math.abs(this.currentSpeeds.omegaRadiansPerSecond) > 1e-2) {
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
    public static Transform3d getTurretRobotToCamera(Transform3d turretToCamera,
        Rotation2d turretRotation) {
        Rotation3d rotate = new Rotation3d(0.0, 0.0, turretRotation.getRadians());

        Transform3d robotToTurret =
            new Transform3d(Constants.Vision.turretCenter.getTranslation(), rotate);

        Transform3d robotToCamera = robotToTurret.plus(turretToCamera);

        return robotToCamera;
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
        if (Math.abs(angleDeg - prevAngle) > 1e-1) {
            Logger.recordOutput("State/stationary/turret", true);
            this.lastTimeMoved = MathSharedStore.getTimestamp();
        } else {
            Logger.recordOutput("State/stationary/turret", false);
        }
        prevAngle = angleDeg;
        currentTurretAngle.addSample(timestamp, new Rotation2d(angle));
    }

    /** Add potentially asequent observation from camera */
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

    /** Add potentially asequent observation from camera */
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
            // var maybeTurretRotationM1 =
            // currentTurretAngle.getSample(pipelineResult.getTimestampSeconds() - 0.1);
            // var maybeTurretRotationP1 =
            // currentTurretAngle.getSample(pipelineResult.getTimestampSeconds() + 0.1);
            if (maybeTurretRotation.isEmpty()) {
                return false;
            }
            // if (Math.abs(angleDiff(maybeTurretRotationM1.get(), maybeTurretRotationP1.get())
            // .in(Degrees)) > 5) {
            // return false;
            // }
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
                if (camera.isTurret) {
                    boolean isStationary =
                        this.lastTimeMoved + 0.5 < pipelineResult.getTimestampSeconds();
                    Logger.recordOutput("State/Camera/" + camera.name + "/isStationary",
                        isStationary);
                    Logger.recordOutput("State/Camera/" + camera.name + "/stationaryValue",
                        this.lastTimeMoved - pipelineResult.getTimestampSeconds());
                    Logger.recordOutput("State/Camera/" + camera.name + "/lastMoved",
                        this.lastTimeMoved);
                    Logger.recordOutput("State/Camera/" + camera.name + "/timestamp",
                        pipelineResult.getTimestampSeconds());
                    if (!isStationary) {
                        rotationStdDev = 10000.0;
                    }
                }
                Logger.recordOutput("State/Camera/" + camera.name + "/stdDevMultipler",
                    stdDevMultiplier);
                Logger.recordOutput("State/Camera/" + camera.name + "/stdDevTranslation",
                    translationStdDev);
                Logger.recordOutput("State/Camera/" + camera.name + "/stdDevRotation",
                    rotationStdDev);
                if (camera.findConstants) {
                    var transform =
                        new Transform3d(new Pose3d(getGlobalPoseEstimate()), cameraPose);
                    Logger.recordOutput(
                        "State/Camera/" + camera.name + "/foundConstant/translation",
                        transform.getTranslation());
                    Logger.recordOutput("State/Camera/" + camera.name + "/foundConstant/rotation/x",
                        Units.radiansToDegrees(transform.getRotation().getX()));
                    Logger.recordOutput("State/Camera/" + camera.name + "/foundConstant/rotation/y",
                        Units.radiansToDegrees(transform.getRotation().getY()));
                    Logger.recordOutput("State/Camera/" + camera.name + "/foundConstant/rotation/z",
                        Units.radiansToDegrees(transform.getRotation().getZ()));
                    return false;
                }
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
    private double desiredFlywheelSpeed = 0.0;
    private double desiredHoodAngleDeg = 0.0;
    private boolean okayToShoot = false;
    private Rotation2d desiredTurretHeadingFieldRelative = Rotation2d.kZero;
    private double currentFlywheelSpeed;
    private double trimUp = 0.0;
    private double trimLeft = 0.0;

    /** Set trim values for autoshooting */
    public void setTrims(double trimUp, double trimLeft) {
        this.trimUp = trimUp;
        this.trimLeft = trimLeft;
    }

    /** Increment trim values for autoshooting */
    public void incTrims(double incUp, double incLeft) {
        this.trimUp += incUp;
        this.trimLeft += incLeft;
    }

    public double getTrimUp() {
        return this.trimUp;
    }

    public double getTrimLeft() {
        return this.trimLeft;
    }

    private void updateShootingTarget() {
        Pose2d bluePose = AllianceFlipUtil.apply(getGlobalPoseEstimate());
        if (bluePose.getX() > FieldConstants.Hub.centerHub.getX()) {
            targetIsGround = true;
            if (bluePose.getY() > FieldConstants.fieldWidth / 2) {
                shootingTarget = AllianceFlipUtil
                    .apply(new Translation2d(0.0, (3 * FieldConstants.fieldWidth / 4)));
            } else {
                shootingTarget =
                    AllianceFlipUtil.apply(new Translation2d(0.0, (FieldConstants.fieldWidth / 4)));
            }
        } else {
            targetIsGround = false;
            shootingTarget = AllianceFlipUtil.apply(FieldConstants.Hub.centerHub);
        }

        Translation2d[] points = new Translation2d[20];
        for (int i = 0; i < 20; i++) {
            double rot = ((double) i) / 19.0;
            points[i] =
                new Translation2d(FieldConstants.Hub.width / 2.0, Rotation2d.fromRotations(rot))
                    .plus(shootingTarget);
        }

        Logger.recordOutput("State/ShootingTarget", points);
        Logger.recordOutput("State/TargetIsGround", targetIsGround);
    }

    /** Update autoshoot target */
    public void updateTargeting() {
        updateShootingTarget();

        Translation2d[] points = new Translation2d[20];
        for (int i = 0; i < 20; i++) {
            double rot = ((double) i) / 19.0;
            points[i] = new Translation2d(0.15, Rotation2d.fromRotations(rot))
                .plus(getTurretCenterFieldFrame().getTranslation());
        }
        Logger.recordOutput("State/turretEstPos", points);

        Translation2d adjustedTarget = shootingTarget;
        if (currentFlywheelSpeed > 10.0) {
            for (int i = 0; i < 5; i++) {
                double distance =
                    adjustedTarget.getDistance(getTurretCenterFieldFrame().getTranslation())
                        + Units.feetToMeters(trimUp);
                var parameters = targetIsGround
                    ? ShotData.getPassParameters(distance, currentFlywheelSpeed, false)
                    : ShotData.getShotParameters(distance, currentFlywheelSpeed, false);
                double tof = parameters.timeOfFlight();
                var forward = getFieldRelativeSpeeds().times(tof);
                adjustedTarget = shootingTarget
                    .minus(new Translation2d(forward.vxMetersPerSecond, forward.vyMetersPerSecond));
            }
        } else {
            adjustedTarget = AllianceFlipUtil.apply(FieldConstants.Hub.centerHub);
        }
        Logger.recordOutput("State/AdjustedShootingTarget", adjustedTarget);
        double distance = adjustedTarget.getDistance(getTurretCenterFieldFrame().getTranslation())
            + Units.feetToMeters(trimUp);
        Logger.recordOutput("State/distance", distance);
        var parameters =
            targetIsGround ? ShotData.getPassParameters(distance, currentFlywheelSpeed, false)
                : ShotData.getShotParameters(distance, currentFlywheelSpeed, true);
        this.desiredFlywheelSpeed = parameters.desiredSpeed();
        this.desiredHoodAngleDeg = targetIsGround ? 30.0 : parameters.hoodAngleDeg();
        this.okayToShoot = parameters.isOkayToShoot();
        this.desiredTurretHeadingFieldRelative =
            adjustedTarget.minus(getTurretCenterFieldFrame().getTranslation()).getAngle()
                .plus(Rotation2d.fromDegrees(trimLeft));
        Logger.recordOutput("State/Trim/TrimUp", trimUp);
        Logger.recordOutput("State/Trim/TrimLeft", trimLeft);
    }

    public boolean isInitted() {
        return initted;
    }

    public double getDesiredFlywheelSpeed() {
        return desiredFlywheelSpeed;
    }

    public double getDesiredHoodAngleDeg() {
        return desiredHoodAngleDeg;
    }

    public boolean isOkayToShoot() {
        return okayToShoot;
    }

    public Rotation2d getDesiredTurretHeadingFieldRelative() {
        return desiredTurretHeadingFieldRelative;
    }

    public void setFlywheelSpeed(double flywheelSpeed) {
        this.currentFlywheelSpeed = flywheelSpeed;
    }

    private final Rectangle robotRect = new Rectangle("pose", Pose2d.kZero,
        Constants.Swerve.bumperFront.in(Meters) * 2, Constants.Swerve.bumperRight.in(Meters) * 2);;

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
