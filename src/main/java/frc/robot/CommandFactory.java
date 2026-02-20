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

/** Command Factory */
public class CommandFactory extends Command {
    public static Command shootAtTarget(Swerve swerve, Shooter shooter, AdjustableHood hood,
        Intake intake, Indexer indexer, boolean isVeloComp) {
        DoubleSupplier distance = () -> swerve.state.getGlobalPoseEstimate().getTranslation()
            .getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

        return Commands.runOnce(() -> {
            ShooterParams params;
            if (isVeloComp) {
                params = ShotCalculator.velocityCompParams(
                    swerve.state.getGlobalPoseEstimate().getTranslation(),
                    swerve.state.getCurrentSpeeds(),
                    FieldConstants.Hub.innerCenterPoint.toTranslation2d());
            } else {
                params = ShotCalculator.staticShotparams(distance);
            }

            shooter.shoot(params.rps()).schedule();
            hood.goToAngle(Degrees.of(params.hoodAngle())).schedule();
            indexer.setSpeedCommand(0.8, 0.8).schedule();
        }).alongWith(intake.slowReturn());
    }

    public static Command staticShoot(Swerve swerve, Shooter shooter, AdjustableHood hood,
        Intake intake, Indexer indexer) {
        return swerve.pointAtHubAndCross().asProxy()
            .alongWith(shootAtTarget(swerve, shooter, hood, intake, indexer, false));
    }

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

            ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(vx, vy, omega);
            ChassisSpeeds robotRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentRotation);

            swerve.setModuleStates(robotRelativeSpeeds);
        }).alongWith(shootAtTarget(swerve, shooter, hood, intake, indexer, true));
    }
}
