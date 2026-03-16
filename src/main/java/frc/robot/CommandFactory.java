package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;

/** Command Factory */
public class CommandFactory {

    /** Prepare flywheel and turret for shooting from a given robot pose. */
    public static Command preShoot(Supplier<Pose2d> robotPoseSupplier,
        Supplier<Translation2d> targetSupplier, Turret turret, Shooter shooter,
        DoubleSupplier adjustUp, DoubleSupplier adjustRight) {
        return Commands.run(() -> {
            final Translation2d target = targetSupplier.get();
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d turretPosition = robotPose
                .plus(new Transform2d(Constants.Vision.turretCenter.toPose2d().getTranslation(),
                    Rotation2d.kZero))
                .getTranslation();
            double adjustUpValue = Units.metersToFeet(adjustUp.getAsDouble());
            Rotation2d adjustRightValue = Rotation2d.fromDegrees(adjustRight.getAsDouble());
            double distance = target.getDistance(turretPosition) + adjustUpValue;
            var parameters = ShotData.getShotParameters(Units.metersToFeet(distance),
                shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond), true);
            shooter.setVelocity(parameters.desiredSpeed());
            boolean turretFacing = turret.setGoalFieldRelative(
                target.minus(turretPosition).getAngle().plus(adjustRightValue));
            boolean isOkay = parameters.isOkayToShoot();
            Logger.recordOutput("AutoShoot/turretFacing", turretFacing);
            Logger.recordOutput("AutoShoot/isOkay", isOkay);
            Logger.recordOutput("AutoShoot/desiredSpeed", parameters.desiredSpeed());
            Logger.recordOutput("AutoShoot/hoodAngleDeg",
                MathUtil.clamp(parameters.hoodAngleDeg(), 0.0, 30.0));
            Logger.recordOutput("AutoShoot/distanceFeet", Units.metersToFeet(distance));
        }, shooter, turret);
    }

    /** Shoot at a given target. */
    public static Command shoot(RobotState state, Supplier<Translation2d> targetSupplier,
        Turret turret, Shooter shooter, Indexer indexer, AdjustableHood hood,
        DoubleSupplier adjustUp, DoubleSupplier adjustLeft, BooleanSupplier disableTurret) {
        return Commands.runEnd(() -> {
            var lookahead = state.getFieldRelativeSpeeds().times(0.05);
            final Translation2d target = targetSupplier.get()
                .plus(new Translation2d(lookahead.vxMetersPerSecond, lookahead.vyMetersPerSecond));
            Translation2d adjustedTarget = target;
            double adjustUpValue = Units.feetToMeters(adjustUp.getAsDouble());
            Rotation2d adjustLeftValue = Rotation2d.fromDegrees(adjustLeft.getAsDouble());
            Logger.recordOutput("AutoShoot/AdjustUp", adjustUpValue);
            Logger.recordOutput("AutoShoot/AdjustLeft", adjustLeftValue);
            for (int i = 0; i < 20; i++) {
                double distance =
                    adjustedTarget.getDistance(state.getTurretCenterFieldFrame().getTranslation())
                        + adjustUpValue;
                var parameters = ShotData.getShotParameters(Units.metersToFeet(distance),
                    shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond), false);
                double tof = parameters.timeOfFlight();
                var forward = state.getFieldRelativeSpeeds().times(tof);
                adjustedTarget = target
                    .minus(new Translation2d(forward.vxMetersPerSecond, forward.vyMetersPerSecond));
            }
            Logger.recordOutput("AutoShoot/Target", target);
            Logger.recordOutput("AutoShoot/AdjustedTarget", adjustedTarget);
            Logger.recordOutput("AutoShoot/TargetDiff", adjustedTarget.minus(target));
            double distance =
                adjustedTarget.getDistance(state.getTurretCenterFieldFrame().getTranslation())
                    + adjustUpValue;
            var parameters = ShotData.getShotParameters(Units.metersToFeet(distance),
                shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond), true);
            shooter.setVelocity(parameters.desiredSpeed());
            hood.setTargetAngle(Degrees.of(MathUtil.clamp(parameters.hoodAngleDeg(), 0.0, 30.0)));
            if (disableTurret.getAsBoolean()) {
                turret.setGoalRobotRelative(Rotation2d.kZero, RotationsPerSecond.of(0));
            } else {
                boolean turretFacing = turret.setGoalFieldRelative(
                    adjustedTarget.minus(state.getTurretCenterFieldFrame().getTranslation())
                        .getAngle().plus(adjustLeftValue)
                        .plus(Rotation2d.fromRadians(lookahead.omegaRadiansPerSecond)));
                Logger.recordOutput("AutoShoot/turretFacing", turretFacing);
            }
            boolean isOkay = parameters.isOkayToShoot();
            Logger.recordOutput("AutoShoot/isOkay", isOkay);
            Logger.recordOutput("AutoShoot/desiredSpeed", parameters.desiredSpeed());
            Logger.recordOutput("AutoShoot/hoodAngleDeg",
                MathUtil.clamp(parameters.hoodAngleDeg(), 0.0, 30.0));
            Logger.recordOutput("AutoShoot/distanceFeet", Units.metersToFeet(distance));
            if (isOkay) {
                indexer.setMagazineDutyCycle(1.0);
                indexer.setSpindexerDutyCycle(6.0);
            } else {
                indexer.setMagazineDutyCycle(0.0);
                indexer.setSpindexerDutyCycle(0.0);
            }
        }, () -> {
            shooter.setVelocity(0.0);
            indexer.setMagazineDutyCycle(0.0);
            indexer.setSpindexerDutyCycle(0.0);
        }, shooter, turret, indexer, hood);
    }

    /** Point turret at hub. */
    public static Command followHub(Turret turret, Swerve swerve, DoubleSupplier trimRight) {
        return turret.goToAngleFieldRelative(() -> {
            return AllianceFlipUtil.apply(FieldConstants.Hub.centerHub)
                .minus(swerve.state.getTurretCenterFieldFrame().getTranslation()).getAngle()
                .plus(Rotation2d.fromDegrees(trimRight.getAsDouble()));
        });
    }
}
