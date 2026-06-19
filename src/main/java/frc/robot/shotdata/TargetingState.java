package frc.robot.shotdata;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.localization.RobotState;
import frc.robot.util.AllianceFlipUtil;

/** Computes and stores the current targeting state for the robot's shooter. */
public class TargetingState {

    private final RobotState drivetrainState;
    private final ShotData shotData;

    /**
     * Creates a new TargetingState.
     *
     * @param drivetrainState reference to the robot's state estimator, used for pose and turret
     *        position in the field frame
     * @param shotData shot lookup table used to determine flywheel speed and hood angle
     */
    public TargetingState(RobotState drivetrainState, ShotData shotData) {
        this.drivetrainState = drivetrainState;
        this.shotData = shotData;
    }

    private Translation2d shootingTarget = FieldConstants.Hub.centerHub;
    private boolean targetIsGround = false;

    private void updateShootingTarget() {
        Pose2d bluePose = AllianceFlipUtil.apply(drivetrainState.getGlobalPoseEstimate());
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

    private double desiredFlywheelSpeed = 0.0;
    private double desiredHoodAngleDeg = 0.0;
    private boolean okayToShoot = false;
    private Rotation2d aimAtHubRotation = Rotation2d.kZero;
    private Rotation2d aimForShootingRotation = Rotation2d.kZero;
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

    /**
     * Gets the current vertical/distance trim.
     *
     * @return trimUp in feet; positive values increase effective shooting distance
     */
    public double getTrimUp() {
        return this.trimUp;
    }

    /**
     * Gets the current horizontal/heading trim.
     *
     * @return trimLeft in degrees; positive values rotate the turret CCW relative to the computed
     *         heading
     */
    public double getTrimLeft() {
        return this.trimLeft;
    }

    /**
     * Recomputes the current targeting solution based on the latest robot state, trims, and current
     * flywheel speed.
     */
    public void updateTargeting() {
        updateShootingTarget();

        Pose2d robotPose = drivetrainState.getGlobalPoseEstimate();
        Translation2d shooterOffset =
            Constants.Vision.turretCenter.getTranslation().toTranslation2d();

        okayToShoot = true;
        var motion = RadialVelocityUtil.getDistanceRadialTangentialVector(
            drivetrainState.getFieldRelativeSpeeds(), robotPose, shooterOffset, shootingTarget);

        Logger.recordOutput("TargetingState/motion/distanceMeters", motion.distanceMeters());
        Logger.recordOutput("TargetingState/motion/radialVelocityMetersPerSecond",
            motion.radialVelocityMetersPerSecond());
        Logger.recordOutput("TargetingState/motion/tangentialVelocityField",
            motion.tangentialVelocityField());

        var parameters_ = shotData.getShotEntry(Meters.of(motion.distanceMeters()),
            MetersPerSecond.of(motion.radialVelocityMetersPerSecond()));

        Logger.recordOutput("TargetingState/hasParameters", parameters_.isPresent());
        Logger.recordOutput("TargetingState/currentFlywheelSpeed", currentFlywheelSpeed);

        Translation2d shooterPosField =
            robotPose.getTranslation().plus(shooterOffset.rotateBy(robotPose.getRotation()));
        aimAtHubRotation = shootingTarget.minus(shooterPosField).getAngle();

        if (parameters_.isPresent()) {
            var parameters = parameters_.get();

            Logger.recordOutput("TargetingState/parameters/flywheelSpeedRps",
                parameters.flywheelSpeedRps());
            Logger.recordOutput("TargetingState/parameters/minFlywheelSpeed",
                parameters.minFlywheelSpeed());
            Logger.recordOutput("TargetingState/parameters/maxFlywheelSpeed",
                parameters.maxFlywheelSpeed());

            Logger.recordOutput("TargetingState/parameters/hoodAngle", parameters.hoodAngle());
            Logger.recordOutput("TargetingState/parameters/minHoodAngle",
                parameters.minHoodAngle());
            Logger.recordOutput("TargetingState/parameters/maxHoodAngle",
                parameters.maxHoodAngle());

            Logger.recordOutput("TargetingState/parameters/tof", parameters.tof());

            double tof = parameters.tof();
            Translation2d aimOffset = motion.tangentialVelocityField().times(tof);
            Translation2d futureAimPoint = shootingTarget.minus(aimOffset);

            Logger.recordOutput("TargetingState/parameters/futureAimPoint", futureAimPoint);

            desiredFlywheelSpeed = parameters.flywheelSpeedRps();
            desiredHoodAngleDeg = parameters.hoodAngle();

            aimForShootingRotation = futureAimPoint.minus(shooterPosField).getAngle();
            if (currentFlywheelSpeed < parameters.minFlywheelSpeed()) {
                okayToShoot = false;
            }
        } else {
            okayToShoot = false;
        }

        Logger.recordOutput("TargetingState/okayToShoot", okayToShoot);
    }

    /**
     * Gets the desired flywheel speed
     *
     * @return desired flywheel speed in rotations per second (RPS)
     */
    public double getDesiredFlywheelSpeed() {
        return desiredFlywheelSpeed;
    }

    /**
     * Gets the desired hood angle
     *
     * @return desired hood angle in degrees
     */
    public double getDesiredHoodAngleDeg() {
        return desiredHoodAngleDeg;
    }

    /**
     * Indicates whether it is currently okay to shoot.
     *
     * @return {@code true} if a valid shot solution is available, {@code false} otherwise
     */
    public boolean isOkayToShoot() {
        return okayToShoot;
    }

    /**
     * Updates the currently measured flywheel speed used for targeting calculations.
     *
     * <p>
     * This value is used, for example, to decide whether to perform motion compensation and whether
     * to trust the current shot solution.
     *
     * @param flywheelSpeed current flywheel speed in rotations per second (RPS)
     */
    public void setFlywheelSpeed(double flywheelSpeed) {
        this.currentFlywheelSpeed = flywheelSpeed;
    }

    public Rotation2d getAimAtHubRotation() {
        return aimAtHubRotation;
    }

    public Rotation2d getAimForShootingRotation() {
        return aimForShootingRotation;
    }

}
