package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShooterParams;

/** Static factory for creating shooting-related commands. */
public final class CommandFactory {

    private CommandFactory() {}

    /**
     * Creates a command that calculates shot parameters and fires at the hub.
     *
     * @param swerve drivetrain subsystem
     * @param shooter shooter subsystem
     * @param hood hood subsystem
     * @param intake intake subsystem
     * @param indexer indexer subsystem
     * @param isVeloComp enables velocity compensation
     * @return shooting command
     */
    public static Command shootAtTarget(Swerve swerve, Shooter shooter, AdjustableHood hood,
        Intake intake, Indexer indexer, boolean isVeloComp) {
        DoubleSupplier distance = () -> swerve.state.getGlobalPoseEstimate().getTranslation()
            .getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

        return Commands.runOnce(() -> {
            ShooterParams params = isVeloComp
                ? ShotCalculator.velocityCompParams(
                    swerve.state.getGlobalPoseEstimate().getTranslation(),
                    swerve.state.getCurrentSpeeds(),
                    FieldConstants.Hub.innerCenterPoint.toTranslation2d())
                : ShotCalculator.staticShotparams(distance);

            shooter.shoot(params.rps()).schedule();
            hood.goToAngle(Degrees.of(params.hoodAngle())).schedule();
            indexer.setSpeedCommand(0.8, 0.8).schedule();
        }).alongWith(intake.slowReturn());
    }

    /**
     * Creates a command that aims at the hub and performs a static shot.
     *
     * @return aiming and shooting command
     */
    public static Command staticShoot(Swerve swerve, Shooter shooter, AdjustableHood hood,
        Intake intake, Indexer indexer) {
        return swerve.pointAtHubAndCross().asProxy()
            .alongWith(shootAtTarget(swerve, shooter, hood, intake, indexer, false));
    }

    /**
     * Creates a command that drives, auto-aims, and shoots while moving.
     *
     * @return drive-and-shoot command
     */
    public static Command shootWhileMoving(Swerve swerve, Shooter shooter, AdjustableHood hood,
        Intake intake, Indexer indexer, CommandPS5Controller controller) {
        return swerve.run(() -> {
            double vx = -controller.getLeftY() * Constants.Swerve.maxSpeedShooting;
            double vy = -controller.getLeftX() * Constants.Swerve.maxSpeedShooting;

            Rotation2d angleToHub = new Rotation2d(Radians.of(FieldConstants.Hub.innerCenterPoint
                .toTranslation2d().minus(swerve.state.getGlobalPoseEstimate().getTranslation())
                .getAngle().getRadians()));

            Rotation2d currentRotation = swerve.state.getGlobalPoseEstimate().getRotation();

            double rotationError = angleToHub.minus(currentRotation).getRadians();
            double omega = rotationError * 5.0;

            omega = Math.max(-Constants.Swerve.maxAngularVelocity,
                Math.min(Constants.Swerve.maxAngularVelocity, omega));

            ChassisSpeeds fieldRelative = new ChassisSpeeds(vx, vy, omega);

            ChassisSpeeds robotRelative =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, currentRotation);

            swerve.setModuleStates(robotRelative);
        }).alongWith(shootAtTarget(swerve, shooter, hood, intake, indexer, true));
        }

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
                Angle leftDistanceGoal = Rotations.of(Passing.blueAllianceLeft
                    .minus(swervePose.getTranslation()).getAngle().getRotations());
                turret.setGoal(leftDistanceGoal);
            } else {
                Angle rightDistanceGoal = Rotations.of(Passing.blueAllianceRight
                    .minus(swervePose.getTranslation()).getAngle().getRotations());
                turret.setGoal(rightDistanceGoal);
            }
            hood.setGoal(Rotations.of(Constants.AdjustableHood.passingAngle));
            shooter.shoot(Constants.Shooter.shooterVelocity);
        }, turret, hood, shooter);
    }
}
