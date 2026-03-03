package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Passing;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

/** Command Factory */
public class CommandFactory {
    /**
     * Sets the turret's target to the left or right based off of its closest distence, then sets
     * the angle of the hood, then the velocity of the shooter, then it shoots.
     */
    public static Command autoPass(Supplier<Pose2d> supplierSwervePose, Turret turret,
        AdjustableHood hood, Shooter shooter) {
        return Commands.run(() -> {
            Pose2d swervePose = supplierSwervePose.get();
            Distance leftDistance =
                Meters.of(Passing.blueAllianceLeft.getDistance(swervePose.getTranslation()));
            Distance rightDistance =
                Meters.of(Passing.blueAllianceRight.getDistance(swervePose.getTranslation()));
            if (leftDistance.in(Meters) < rightDistance.in(Meters)) {
                Rotation2d leftDistanceGoal =
                    Passing.blueAllianceLeft.minus(swervePose.getTranslation()).getAngle();
                turret.setGoalRobotRelative(leftDistanceGoal, RotationsPerSecond.of(0));
            } else {
                Rotation2d rightDistanceGoal =
                    Passing.blueAllianceRight.minus(swervePose.getTranslation()).getAngle();
                turret.setGoalRobotRelative(rightDistanceGoal, RotationsPerSecond.of(0));
            }
            hood.setGoal(Rotations.of(Constants.AdjustableHood.passingAngle));
            shooter.shoot(Constants.Shooter.shooterVelocity);
        }, turret, hood, shooter);
    }

    /** Shoot at a given target. */
    public static Command shoot(RobotState state, Supplier<Translation2d> targetSupplier,
        Turret turret, Shooter shooter, Indexer indexer, AdjustableHood hood,
        DoubleSupplier adjustUp, DoubleSupplier adjustRight) {
        return Commands.runEnd(() -> {
            final Translation2d target = targetSupplier.get();
            Translation2d adjustedTarget = target;
            double adjustUpValue = Units.metersToFeet(adjustUp.getAsDouble());
            Rotation2d adjustRightValue = Rotation2d.fromDegrees(adjustRight.getAsDouble());
            Translation2d[] adjustedTargets = new Translation2d[21];
            for (int i = 0; i < 20; i++) {
                adjustedTargets[i] = adjustedTarget;
                double distance =
                    adjustedTarget.getDistance(state.getTurretCenterFieldFrame().getTranslation())
                        + adjustUpValue;
                var parameters = ShotData.getShotParameters(Units.metersToFeet(distance),
                    shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond));
                double tof = parameters.timeOfFlight();
                var forward = state.getFieldRelativeSpeeds().times(tof);
                adjustedTarget = target
                    .minus(new Translation2d(forward.vxMetersPerSecond, forward.vyMetersPerSecond));
            }
            adjustedTargets[20] = adjustedTarget;
            Logger.recordOutput("AutoShoot/AdjustedTargetIterations", adjustedTargets);
            double distance =
                adjustedTarget.getDistance(state.getTurretCenterFieldFrame().getTranslation())
                    + adjustUpValue;
            var parameters = ShotData.getShotParameters(Units.metersToFeet(distance),
                shooter.inputs.shooterAngularVelocity1.in(RotationsPerSecond));
            shooter.setVelocity(parameters.desiredSpeed());
            hood.setTargetAngle(Degrees.of(MathUtil.clamp(parameters.hoodAngleDeg(), 0.0, 30.0)));
            boolean turretFacing = turret.setGoalFieldRelative(
                adjustedTarget.minus(state.getTurretCenterFieldFrame().getTranslation()).getAngle()
                    .plus(adjustRightValue));
            boolean isOkay = parameters.isOkayToShoot();
            Logger.recordOutput("AutoShoot/turretFacing", turretFacing);
            Logger.recordOutput("AutoShoot/isOkay", isOkay);
            Logger.recordOutput("AutoShoot/desiredSpeed", parameters.desiredSpeed());
            Logger.recordOutput("AutoShoot/hoodAngleDeg",
                MathUtil.clamp(parameters.hoodAngleDeg(), 0.0, 30.0));
            Logger.recordOutput("AutoShoot/distanceFeet", Units.metersToFeet(distance));
            if (isOkay && turretFacing) {
                indexer.setMagazineDutyCycle(0.7);
                indexer.setSpindexerDutyCycle(0.7);
            } else {
                indexer.setMagazineDutyCycle(0.0);
                indexer.setSpindexerDutyCycle(-0.2);
            }
        }, () -> {
            shooter.setVelocity(0.0);
            indexer.setMagazineDutyCycle(0.0);
            indexer.setSpindexerDutyCycle(0.0);
        }, shooter, indexer, hood);
    }
}
