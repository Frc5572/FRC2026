package frc.robot.localization;

import static edu.wpi.first.units.Units.Degrees;
import java.util.List;
import java.util.Optional;
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
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
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
    public boolean addVisionObservation(CameraConstants camera,
        PhotonPipelineResult pipelineResult) {

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
            // for (int i = 0; i < 5; i++) {
            // double distance =
            // adjustedTarget.getDistance(getTurretCenterFieldFrame().getTranslation())
            // + Units.feetToMeters(trimUp);
            // var parameters = targetIsGround
            // ? ShotData.getPassParameters(distance, currentFlywheelSpeed, false)
            // : ShotData.getShotParameters(distance, currentFlywheelSpeed, false);
            // double tof = parameters.timeOfFlight();
            // var forward = getFieldRelativeSpeeds().times(tof);
            // adjustedTarget = shootingTarget
            // .minus(new Translation2d(forward.vxMetersPerSecond, forward.vyMetersPerSecond));
            // }
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
        Logger.recordOutput("State/desiredTurretHeading", this.desiredTurretHeadingFieldRelative);
        Logger.recordOutput("State/Trim/TrimUp", trimUp);
        Logger.recordOutput("State/Trim/TrimLeft", trimLeft);

        Translation2d[] turretDirection = new Translation2d[2];
        turretDirection[0] = getTurretCenterFieldFrame().getTranslation();
        turretDirection[1] = getTurretCenterFieldFrame().getTranslation()
            .plus(new Translation2d(2.0, this.desiredTurretHeadingFieldRelative));
        Logger.recordOutput("State/DesiredTurretDirection", turretDirection);
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


}
