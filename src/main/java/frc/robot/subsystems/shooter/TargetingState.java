package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.shotdata.ShotData;
import frc.robot.util.AllianceFlipUtil;

/** Computes desired shooting parameters from robot state. */
public class TargetingState {

    private final Supplier<Pose2d> poseSource;
    private final Supplier<ChassisSpeeds> speedSource;
    private final DoubleSupplier flywheelSource;

    /**
     * constructor for targeting state
     * 
     * @param poseSource pose supplier
     * @param speedSource bot chassis speeds supplier
     * @param flywheelSource flywheel velocity supplier
     */
    public TargetingState(Supplier<Pose2d> poseSource, Supplier<ChassisSpeeds> speedSource,
        DoubleSupplier flywheelSource) {

        this.poseSource = poseSource;
        this.speedSource = speedSource;
        this.flywheelSource = flywheelSource;
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

    public void setTrims(double trimUp, double trimLeft) {
        this.trimUp = trimUp;
        this.trimLeft = trimLeft;
    }

    public void incTrims(double incUp, double incLeft) {
        this.trimUp += incUp;
        this.trimLeft += incLeft;
    }

    public double getTrimUp() {
        return trimUp;
    }

    public double getTrimLeft() {
        return trimLeft;
    }

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

    /** Recompute targeting solution. Call once per loop. */
    public void updateTargeting() {

        Pose2d pose = poseSource.get();
        ChassisSpeeds speeds = speedSource.get();
        currentFlywheelSpeed = flywheelSource.getAsDouble();

        updateShootingTarget(pose);

        Translation2d turretCenter = poseSource.get().getTranslation();

        Translation2d[] points = new Translation2d[20];
        for (int i = 0; i < 20; i++) {
            double rot = ((double) i) / 19.0;

            points[i] = new Translation2d(0.15, Rotation2d.fromRotations(rot)).plus(turretCenter);
        }

        Logger.recordOutput("State/turretEstPos", points);

        Translation2d adjustedTarget = shootingTarget;

        if (currentFlywheelSpeed > 10.0) {

            /*
             * for (int i = 0; i < 5; i++) { double distance =
             * adjustedTarget.getDistance(turretCenter) + Units.feetToMeters(trimUp);
             * 
             * var parameters = targetIsGround ? ShotData.getPassParameters( distance,
             * currentFlywheelSpeed, false) : ShotData.getShotParameters( distance,
             * currentFlywheelSpeed, false);
             * 
             * double tof = parameters.timeOfFlight();
             * 
             * var forward = speeds.times(tof);
             * 
             * adjustedTarget = shootingTarget.minus( new Translation2d( forward.vxMetersPerSecond,
             * forward.vyMetersPerSecond)); }
             */
        } else {
            adjustedTarget = AllianceFlipUtil.apply(FieldConstants.Hub.centerHub);
        }

        Logger.recordOutput("State/AdjustedShootingTarget", adjustedTarget);

        double distance = adjustedTarget.getDistance(turretCenter) + Units.feetToMeters(trimUp);

        Logger.recordOutput("State/distance", distance);

        var parameters =
            targetIsGround ? ShotData.getPassParameters(distance, currentFlywheelSpeed, false)
                : ShotData.getShotParameters(distance, currentFlywheelSpeed, true);

        desiredFlywheelSpeed = parameters.desiredSpeed();
        desiredHoodAngleDeg = targetIsGround ? 30.0 : parameters.hoodAngleDeg();
        okayToShoot = parameters.isOkayToShoot();

        desiredTurretHeadingFieldRelative =
            adjustedTarget.minus(turretCenter).getAngle().plus(Rotation2d.fromDegrees(trimLeft));

        Logger.recordOutput("State/desiredTurretHeading", desiredTurretHeadingFieldRelative);

        Logger.recordOutput("State/Trim/TrimUp", trimUp);
        Logger.recordOutput("State/Trim/TrimLeft", trimLeft);

        Translation2d[] turretDirection = new Translation2d[2];

        turretDirection[0] = turretCenter;
        turretDirection[1] =
            turretCenter.plus(new Translation2d(2.0, desiredTurretHeadingFieldRelative));

        Logger.recordOutput("State/DesiredTurretDirection", turretDirection);
    }

    public double getDesiredFlywheelSpeed() {
        return desiredFlywheelSpeed;
    }

    public double getDesiredHoodAngle() {
        return desiredHoodAngleDeg;
    }

    public boolean isOkayToShoot() {
        return okayToShoot;
    }

    public Rotation2d getDesiredTurretHeadingFieldRelative() {
        return desiredTurretHeadingFieldRelative;
    }
}
