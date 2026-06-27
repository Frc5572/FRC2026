package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.shotdata.RadialVelocityUtil;
import frc.robot.shotdata.ShotData;
import frc.robot.util.AllianceFlipUtil;

/** Computes and stores the current targeting state for the robot's shooter. */
public class TargetingState {

    private final Supplier<Pose2d> poseSource;
    private final Supplier<ChassisSpeeds> speedSource;
    private final DoubleSupplier flywheelSource;
    private final ShotData shotData;

    /**
     * Creates a new TargetingState.
     *
     * @param poseSource supplies the robot's global field pose
     * @param speedSource supplies the robot's field-relative chassis speeds
     * @param flywheelSource supplies the measured flywheel speed in rotations per second (RPS)
     * @param shotData shot lookup table used to determine flywheel speed and hood angle
     */
    public TargetingState(Supplier<Pose2d> poseSource, Supplier<ChassisSpeeds> speedSource,
        DoubleSupplier flywheelSource, ShotData shotData) {
        this.poseSource = poseSource;
        this.speedSource = speedSource;
        this.flywheelSource = flywheelSource;
        this.shotData = shotData;
    }

    private Translation2d shootingTarget = FieldConstants.Hub.centerHub;
    private boolean targetIsGround = false;

    private void updateShootingTarget(Pose2d globalEst) {
        Pose2d bluePose = AllianceFlipUtil.apply(globalEst);
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
    /**
     * Hysteresis state for the burst gate. While {@code true} the indexer keeps running as the
     * flywheel sags toward the bottom of the shot window; it only re-arms after the flywheel
     * recovers near the top. Prevents the per-shot flip-flopping of {@link #okayToShoot}.
     */
    private boolean bursting = false;
    private Rotation2d aimAtHubRotation = Rotation2d.kZero;
    private Rotation2d aimForShootingRotation = Rotation2d.kZero;
    private double currentFlywheelSpeed;
    private double trimUp = 0.0;
    private double trimLeft = 0.0;

    public void setTrims(double trimUp, double trimLeft) {
        this.trimUp = trimUp;
        this.trimLeft = trimLeft;
    }

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
        Pose2d robotPose = poseSource.get();
        ChassisSpeeds speeds = speedSource.get();
        currentFlywheelSpeed = flywheelSource.getAsDouble();

        updateShootingTarget(robotPose);

        Translation2d shooterOffset =
            Constants.Vision.turretCenter.getTranslation().toTranslation2d();

        var motion = RadialVelocityUtil.getDistanceRadialTangentialVector(speeds, robotPose,
            shooterOffset, shootingTarget);

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

            var burstGate = Constants.Shooter.burstGate.get();

            // Run hot: target somewhere between the nominal and the top of the shot window so
            // there is sag headroom for a continuous burst. Always between nominal and max.
            double nominal = parameters.flywheelSpeedRps();
            double max = parameters.maxFlywheelSpeed();
            double min = parameters.minFlywheelSpeed();
            desiredFlywheelSpeed = nominal + burstGate.setpointFraction * (max - nominal);
            desiredHoodAngleDeg = parameters.hoodAngle();

            aimForShootingRotation = futureAimPoint.minus(shooterPosField).getAngle();

            // Hysteresis (Schmitt-trigger) burst gate. Once a burst starts, keep indexing as the
            // flywheel sags all the way down to the bottom of the window, then stop and wait until
            // it recovers near the setpoint before re-arming. This replaces the single-threshold
            // gate that flip-flopped the indexer on every shot.
            double armThreshold = desiredFlywheelSpeed - burstGate.armFraction * (max - min);


            Logger.recordOutput("TargetingState/schmitt/target", desiredFlywheelSpeed);
            Logger.recordOutput("TargetingState/schmitt/armThreshold", armThreshold);
            Logger.recordOutput("TargetingState/schmitt/minThreshold", min);

            if (bursting) {
                if (currentFlywheelSpeed < min) {
                    bursting = false;
                }
            } else if (currentFlywheelSpeed >= armThreshold) {
                bursting = true;
            }
            okayToShoot = bursting;
        } else {
            bursting = false;
            okayToShoot = false;
        }

        Logger.recordOutput("TargetingState/bursting", bursting);

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
    public double getDesiredHoodAngle() {
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

    public Rotation2d getAimAtHubRotation() {
        return aimAtHubRotation;
    }

    public Rotation2d getAimForShootingRotation() {
        return aimForShootingRotation;
    }

}
