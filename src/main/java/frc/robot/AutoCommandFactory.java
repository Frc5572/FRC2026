package frc.robot;

import java.util.Set;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.MoveToPose;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommandFactory {

    private final AutoFactory autoFactory;
    private final Swerve swerve;
    private final AdjustableHood adjustableHood;
    private final Indexer indexer;
    private final Intake intake;
    private final Turret turret;
    private final Climber climber;
    private final Shooter shooter;
    private final Vision vision;

    public AutoCommandFactory(AutoFactory autoFactory, Swerve swerve, AdjustableHood adjustableHood,
        Climber climber, Intake intake, Indexer indexer, Shooter shooter, Turret turret,
        Vision vision) {
        this.swerve = swerve;
        this.adjustableHood = adjustableHood;
        this.climber = climber;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.turret = turret;
        this.vision = vision;
        this.autoFactory = autoFactory;
    }

    public AutoRoutine shootOnlyAuto() {
        AutoRoutine routine = autoFactory.newRoutine("shootOnlyAuto");
        Commands.runOnce(() -> swerve.resetOdometry(swerve.getPose()));
        Command score;
        Pose2d leftTrench = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.LeftTrench.redTrenchCenterLeft, Rotation2d.kZero));
        Pose2d rightTrench = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.RightTrench.redTrenchCenterRight, Rotation2d.kZero));
        Pose2d leftFuel = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.LinesVertical.closeFuelXPosition + (35.95 / 2),
                FieldConstants.LinesHorizontal.leftFuelYPosition, Rotation2d.kZero));
        Pose2d rightFuel = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.LinesVertical.closeFuelXPosition + (35.95 / 2),
                FieldConstants.LinesHorizontal.rightFuelYPosition, Rotation2d.kZero));
        Pose2d leftFinalFuel = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.LinesVertical.closeFuelXPosition + (35.95 / 2),
                FieldConstants.LinesHorizontal.leftFinalFuelYPosition, Rotation2d.kZero));
        Pose2d rightFinalFuel = AllianceFlipUtil
            .apply(new Pose2d(FieldConstants.LinesVertical.closeFuelXPosition + (35.95 / 2),
                FieldConstants.LinesHorizontal.rightFinalFuelYPosition, Rotation2d.kZero));
        Pose2d leftShootLocation =
            AllianceFlipUtil.apply(new Pose2d(FieldConstants.LinesVertical.starting - 1,
                FieldConstants.LeftTrench.openingTopLeft.getY(), Rotation2d.kZero));
        Pose2d rightShootLocation =
            AllianceFlipUtil.apply(new Pose2d(FieldConstants.LinesVertical.starting - 1,
                FieldConstants.RightTrench.openingTopLeft.getY(), Rotation2d.kZero));

        score =
            Commands.sequence(Commands.waitUntil(() -> vision.hasHighConfidence()).withTimeout(3.0),
                Commands.defer(() -> {
                    Pose2d currentPose = swerve.getPose();
                    double distToLeft =
                        currentPose.getTranslation().getDistance(leftTrench.getTranslation());
                    double distToRight =
                        currentPose.getTranslation().getDistance(rightTrench.getTranslation());

                    Pose2d chosenTrench;
                    Rotation2d intakeRotation;
                    Pose2d intakeLocation;
                    Pose2d intakeFinalLocation;
                    Pose2d shootLocation;

                    if (distToLeft < distToRight) {
                        chosenTrench = leftTrench;
                        intakeRotation = Rotation2d.fromDegrees(90);
                        intakeLocation = leftFuel;
                        intakeFinalLocation = leftFinalFuel;
                        shootLocation = leftShootLocation;
                    } else {
                        chosenTrench = rightTrench;
                        intakeRotation = Rotation2d.fromDegrees(270);
                        intakeLocation = rightFuel;
                        intakeFinalLocation = rightFinalFuel;
                        shootLocation = rightShootLocation;
                    }

                    return Commands.sequence(
                        // Move to Trench
                        new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                            () -> new Pose2d(
                                AllianceFlipUtil.apply(new Translation2d(
                                    FieldConstants.LinesVertical.starting, chosenTrench.getY())),
                                chosenTrench.getRotation()),
                            null, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5),

                        // Move through trench + Intake
                        new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                            () -> new Pose2d(AllianceFlipUtil.apply(new Translation2d(
                                FieldConstants.LinesVertical.neutralZoneNear, chosenTrench.getY())),
                                chosenTrench.getRotation()),
                            null, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5)
                                .andThen(intake.waitUntilExtended(), intake.intakeBalls(6)),

                        // Drive to Fuel
                        new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                            () -> new Pose2d(AllianceFlipUtil.apply(
                                new Translation2d(intakeLocation.getX(), intakeLocation.getY())),
                                intakeRotation),
                            null, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5),

                        // Intake
                        new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                            () -> new Pose2d(
                                AllianceFlipUtil.apply(new Translation2d(intakeFinalLocation.getX(),
                                    intakeFinalLocation.getY())),
                                intakeRotation),
                            null, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5),


                        // Back to Hub
                        new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                            () -> shootLocation, null, () -> Constants.Swerve.autoMaxSpeed, true,
                            0.05, 5.0),

                        // Shoot
                        Commands.parallel(shooter.shoot(65),
                            Commands.sequence(Commands.waitSeconds(0.6),
                                indexer.setSpeedCommand(0.8, 0.8)),
                            Commands.repeatingSequence(intake.extendHopper().withTimeout(0.4),
                                intake.retractHopper().withTimeout(0.4)))
                            .withTimeout(7.0));

                }, Set.of(swerve, intake, indexer, shooter)));

        score = score.andThen(swerve.stop());
        routine.active().onTrue(score.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        return routine;
    }
}
