package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

/** Command Factory */

/**
 * Sets the turret's target to the left or right based off of its closest distence, then sets the
 * angle of the hood, then the velocity of the shooter, then it shoots.
 */
public class CommandFactory {
    public static Command autoShoot(Pose2d swervePose, Turret turret, AdjustableHood hood,
        Shooter shooter) {
        return Commands.run(() -> {
            double leftDistance = Hub.nearLeftCorner.getDistance(swervePose.getTranslation());
            double rightDistance = Hub.nearRightCorner.getDistance(swervePose.getTranslation());
            if (leftDistance < rightDistance) {
                double leftDistanceGoal =
                    Hub.nearLeftCorner.minus(swervePose.getTranslation()).getAngle().getRotations();
                turret.setGoal(Rotations.of(leftDistanceGoal));
            } else {
                double rightDistanceGoal = Hub.nearRightCorner.minus(swervePose.getTranslation())
                    .getAngle().getRotations();
                turret.setGoal(Rotations.of(rightDistanceGoal));
            }
            hood.setGoal(Rotations.of(Constants.AdjustableHood.passingAngle));
            shooter.runShooterVelocityCommand(Constants.Shooter.shooterVelocity);
        });
    }
}
