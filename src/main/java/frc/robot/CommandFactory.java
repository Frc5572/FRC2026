package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

/** Static factory for commands. */
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
            .plus(Constants.Vision.turretCenter.toPose2d().getTranslation())
            .getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

        return Commands.runOnce(() -> {
            ShooterParams params = isVeloComp
                ? ShotCalculator.velocityCompParams(
                    swerve.state.getGlobalPoseEstimate().getTranslation(),
                    swerve.state.getCurrentSpeeds(),
                    FieldConstants.Hub.innerCenterPoint.toTranslation2d())
                : ShotCalculator.staticShotparams(distance);

            shooter.setVelocity(params.rps());
            hood.setGoal(Degrees.of(params.hoodAngle()));

        }).alongWith(intake.slowReturn()).alongWith(indexer.setSpeedCommand(0.8, 0.8));
    }

    /**
     * Creates a command that aims at the hub and performs a static shot.
     *
     * @return aiming and shooting command
     */
    public static Command staticShootFixedTurret(Swerve swerve, Shooter shooter,
        AdjustableHood hood, Intake intake, Indexer indexer) {
        return swerve.pointAtHubAndCross()
            .alongWith(shootAtTarget(swerve, shooter, hood, intake, indexer, false));
    }

    /**
     * Creates a command that drives, auto-aims, and shoots while moving.
     *
     * @return drive-and-shoot command
     */
    public static Command shootWhileMovingFixedTurret(Swerve swerve, Shooter shooter,
        AdjustableHood hood, Intake intake, Indexer indexer, CommandPS5Controller controller) {
        return swerve.driveUserRelative(() -> {
            double vx = -controller.getLeftY() * Constants.Swerve.maxSpeedShooting;
            double vy = -controller.getLeftX() * Constants.Swerve.maxSpeedShooting;

            Translation2d targetPosition = FieldConstants.Hub.topCenterPoint.toTranslation2d();
            if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) {
                targetPosition = new Translation2d(
                    FieldConstants.fieldLength - targetPosition.getX(), targetPosition.getY());
            }

            Rotation2d angleToHub = new Rotation2d(Radians
                .of(targetPosition.minus(swerve.state.getGlobalPoseEstimate().getTranslation())
                    .getAngle().getRadians()));
            Rotation2d currentRotation = swerve.state.getGlobalPoseEstimate().getRotation();
            double rotationError = angleToHub.minus(currentRotation).getRadians();
            double omega = rotationError * 5.0;
            omega = Math.max(-Constants.Swerve.maxAngularVelocity,
                Math.min(Constants.Swerve.maxAngularVelocity, omega));
            ChassisSpeeds fieldRelative = new ChassisSpeeds(vx, vy, omega);
            ChassisSpeeds robotRelative =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, currentRotation);
            return robotRelative;
        }).alongWith(shootAtTarget(swerve, shooter, hood, intake, indexer, true));
    }

    /**
     * Creates a command that drives, auto-aims, and shoots while moving.
     *
     * @return drive-and-shoot command
     */
    public static Command passWhileMoving(Swerve swerve, Shooter shooter, AdjustableHood hood,
        Intake intake, Indexer indexer, CommandPS5Controller controller) {
        return swerve.driveUserRelative(() -> {
            Translation2d target;
            double vx = -controller.getLeftY() * Constants.Swerve.maxSpeedShooting;
            double vy = -controller.getLeftX() * Constants.Swerve.maxSpeedShooting;

            Translation2d rightTarget = ((DriverStation.getAlliance().get() == Alliance.Blue)
                ? FieldConstants.Passing.blueAllianceLeft
                : FieldConstants.Passing.redAllianceLeft);

            Translation2d leftTarget = ((DriverStation.getAlliance().get() == Alliance.Blue)
                ? FieldConstants.Passing.blueAllianceRight
                : FieldConstants.Passing.redAllianceRight);

            double distanceToLeft =
                leftTarget.getDistance(swerve.state.getGlobalPoseEstimate().getTranslation());
            double distanceToRight =
                rightTarget.getDistance(swerve.state.getGlobalPoseEstimate().getTranslation());

            if (Math.abs(distanceToLeft) < Math.abs(distanceToRight)) {
                target = leftTarget;
            } else {
                target = rightTarget;
            }

            Rotation2d angleToPass =
                target.minus(swerve.state.getGlobalPoseEstimate().getTranslation()).getAngle();

            Rotation2d currentRotation = swerve.state.getGlobalPoseEstimate().getRotation();

            double rotationError = angleToPass.minus(currentRotation).getRadians();
            double omega = rotationError * 5.0;

            omega = Math.max(-Constants.Swerve.maxAngularVelocity,
                Math.min(Constants.Swerve.maxAngularVelocity, omega));

            ChassisSpeeds fieldRelative = new ChassisSpeeds(vx, vy, omega);

            ChassisSpeeds robotRelative =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, currentRotation);
            return robotRelative;
        }).alongWith(shootAtTarget(swerve, shooter, hood, intake, indexer, true));
    }
}
