package frc.robot.shotdata;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;

public class TargetingState {

    private final RobotState drivetrainState;
    private final ShotData shotData;

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

    public void updateTargeting() {
        updateShootingTarget();

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
        double distance =
            adjustedTarget.getDistance(drivetrainState.getTurretCenterFieldFrame().getTranslation())
                + Units.feetToMeters(trimUp);
        Logger.recordOutput("State/distance", distance);
        var parameters = shotData.getShotEntry(Meters.of(distance), MetersPerSecond.of(0));

        if (parameters.isPresent()) {
            var parameters_ = parameters.get();
            this.desiredFlywheelSpeed = parameters_.flywheelSpeedRps();
            this.desiredHoodAngleDeg = parameters_.hoodAngle();
        }
        this.okayToShoot = parameters.isPresent();

        this.desiredTurretHeadingFieldRelative =
            adjustedTarget.minus(drivetrainState.getTurretCenterFieldFrame().getTranslation())
                .getAngle().plus(Rotation2d.fromDegrees(trimLeft));
        Logger.recordOutput("State/desiredTurretHeading", this.desiredTurretHeadingFieldRelative);
        Logger.recordOutput("State/Trim/TrimUp", trimUp);
        Logger.recordOutput("State/Trim/TrimLeft", trimLeft);

        Translation2d[] turretDirection = new Translation2d[2];
        turretDirection[0] = drivetrainState.getTurretCenterFieldFrame().getTranslation();
        turretDirection[1] = drivetrainState.getTurretCenterFieldFrame().getTranslation()
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
