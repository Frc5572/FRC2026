package frc.robot;

import java.util.Set;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.MoveToPose;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommandFactory {

    AutoFactory autoFactory;
    Swerve swerve;
    AdjustableHood adjustableHood;
    Climber climber;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    Turret turret;

    public AutoCommandFactory(AutoFactory autoFactory, Swerve swerve, AdjustableHood adjustableHood,
        Climber climber, Intake intake, Indexer indexer, Shooter shooter, Turret turret) {
        this.autoFactory = autoFactory;
        this.swerve = swerve;
        this.adjustableHood = adjustableHood;
        this.climber = climber;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.turret = turret;
    }

    public AutoRoutine shootOnlyAuto() {
        AutoRoutine routine = autoFactory.newRoutine("Shoot Only Auto");

        Pose2d leftTrench = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.LeftTrench.redTrenchCenterLeft, Rotation2d.kZero));
        Pose2d rightTrench = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.RightTrench.redTrenchCenterRight, Rotation2d.kZero));
        Pose2d hubPose = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.Hub.centerHub, Rotation2d.fromDegrees(0)));
        Pose2d leftFuel = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.LinesVertical.closeFuelXPosition + (35.95 / 2),
                FieldConstants.LinesHorizontal.leftFuelYPosition, Rotation2d.kZero));
        Pose2d rightFuel = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.LinesVertical.closeFuelXPosition + (35.95 / 2),
                FieldConstants.LinesHorizontal.rightFuelYPosition, Rotation2d.kZero));

        // Defer the command so it calculates "closest" when Auto begins
        Command run = Commands.defer(() -> {
            // 1. Get current pose (which was updated by AprilTags in your Swerve periodic)
            Pose2d currentPose = swerve.getPose();

            double distToLeft =
                currentPose.getTranslation().getDistance(leftTrench.getTranslation());
            double distToRight =
                currentPose.getTranslation().getDistance(rightTrench.getTranslation());

            Pose2d chosenTrench;
            Rotation2d intakeRotation;
            Pose2d intakeLocation;

            if (distToLeft < distToRight) {
                chosenTrench = leftTrench;
                intakeRotation = Rotation2d.fromDegrees(90);
                intakeLocation = leftFuel;
            } else {
                chosenTrench = rightTrench;
                intakeRotation = Rotation2d.fromDegrees(270);
                intakeLocation = rightFuel;
            }

            // 3. Return the sequence using your MoveToPose constructor (8 arguments)
            return Commands.sequence(
                // Move to the chosen trench
                new MoveToPose(
                    swerve, swerve::driveRobotRelativeDirect,
                    () -> new Pose2d(
                        AllianceFlipUtil.apply(new Translation2d(
                            FieldConstants.LinesVertical.starting, chosenTrench.getY())),
                        chosenTrench.getRotation()),
                    routine, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5),

                // Move through trench
                new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                    () -> new Pose2d(
                        AllianceFlipUtil.apply(new Translation2d(
                            FieldConstants.LinesVertical.neutralZoneNear, chosenTrench.getY())),
                        chosenTrench.getRotation()),
                    routine, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5)
                        .deadlineWith(intake.extendHopper(), intake.intakeBalls(6)),

                // IntakeBalls

                new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                    () -> new Pose2d(
                        AllianceFlipUtil
                            .apply(new Translation2d(intakeLocation.getX(), intakeLocation.getY())),
                        intakeRotation),
                    routine, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5),

                // Return to Hub and Shoot
                new MoveToPose(swerve, swerve::driveRobotRelativeDirect, () -> hubPose, routine,
                    () -> Constants.Swerve.autoMaxSpeed, true, 0.05, 5.0)
                        .andThen(() -> shooter.shoot(5)).repeatedly());
        }, Set.of(swerve));

        routine.active().onTrue(run);
        return routine;
    }
}


