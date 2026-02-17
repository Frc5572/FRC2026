package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
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
                Meters.of(Hub.nearLeftCorner.getDistance((swervePose).getTranslation()));
            Distance rightDistance =
                Meters.of(Hub.nearRightCorner.getDistance((swervePose).getTranslation()));
            if (leftDistance.in(Meters) < rightDistance.in(Meters)) {
                Angle leftDistanceGoal = Rotations.of(Hub.nearLeftCorner
                    .minus(swervePose.getTranslation()).getAngle().getRotations());
                turret.setGoal(leftDistanceGoal);
            } else {
                Angle rightDistanceGoal = Rotations.of(Hub.nearRightCorner
                    .minus(swervePose.getTranslation()).getAngle().getRotations());
                turret.setGoal(rightDistanceGoal);
            }
            hood.setGoal(Rotations.of(Constants.AdjustableHood.passingAngle));
            shooter.runShooterVelocityCommand(Constants.Shooter.shooterVelocity);
        });
    }
}
