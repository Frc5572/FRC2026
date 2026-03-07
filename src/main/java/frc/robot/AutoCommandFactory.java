package frc.robot;

import java.util.Set;
import java.util.function.Supplier;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.swerve.util.TurnToRotation;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;

/**
 * Auto Command Factory
 */
public class AutoCommandFactory {

    AutoFactory autoFactory;
    Swerve swerve;
    AdjustableHood adjustableHood;
    Climber climber;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    Turret turret;

    /**
     * Auto Command Factory
     */
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

    /**
     * 
     * @return an auto routine where the bot drive through the trench to the other side, intakes
     *         balls then comes back and shoots.
     */
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

        score = Commands.defer(() -> {
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
                    routine, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5),

                // Move through trench + Intake
                new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                    () -> new Pose2d(
                        AllianceFlipUtil.apply(new Translation2d(
                            FieldConstants.LinesVertical.neutralZoneNear, chosenTrench.getY())),
                        chosenTrench.getRotation()),
                    routine, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5)
                        .andThen(intake.extendHopper(0)).andThen(intake.intakeBalls(6)),

                // Drive to Fuel
                new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                    () -> new Pose2d(AllianceFlipUtil
                        .apply(new Translation2d(intakeLocation.getX(), intakeLocation.getY())),
                        intakeRotation),
                    routine, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5),

                // Intake
                new MoveToPose(swerve, swerve::driveRobotRelativeDirect,
                    () -> new Pose2d(AllianceFlipUtil.apply(
                        new Translation2d(intakeFinalLocation.getX(), intakeFinalLocation.getY())),
                        intakeRotation),
                    routine, () -> Constants.Swerve.autoMaxSpeed, true, 0.1, 5),

                // Back to Hub
                new MoveToPose(swerve, swerve::driveRobotRelativeDirect, () -> shootLocation,
                    routine, () -> Constants.Swerve.autoMaxSpeed, true, 0.05, 5.0),

                // Shoot
                Commands.parallel(shooter.shoot(65),
                    Commands.sequence(Commands.waitSeconds(0.6), indexer.setSpeedCommand(0.8, 0.8)),
                    Commands.repeatingSequence(intake.jerkIntake())).withTimeout(7.0));

        }, Set.of(swerve, intake, indexer, shooter));

        score = score.andThen(swerve.stop());
        routine.active().onTrue(score.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        return routine;
    }

    /**
     * Gather Fuel from the left side and then return and shoot
     *
     * @return AutoRoutine
     */
    public AutoRoutine gatherThenShootLeft() {

        AutoRoutine routine = autoFactory.newRoutine("Gather Then Shoot (Left)");
        MoveToPose moveToStart = swerve.moveToPose().target(new Pose2d(3.6, 7.5, new Rotation2d()))
            .autoRoutine(routine).finish();

        AutoTrajectory path = routine.trajectory("LeftSideGatherShoot");
        routine.active().onTrue(moveToStart);
        // moveToStart.active().whileTrue(Commands.print("Running Move To Start").repeatedly());
        // moveToStart.done().onTrue(Commands.print("Move to Start Complete!!!!!!!!!!!"));
        moveToStart.done().onTrue(path.cmd());
        // path.active().whileTrue(Commands.print("Running Gather Path from Choreo").repeatedly());
        // path.done().onTrue(Commands.print("Gather Path Complete!!!!!!!!!!!"));

        path.active().onTrue(intake.extendHopper(0).andThen(intake.intakeBalls()));

        Supplier<Rotation2d> rotSup = () -> {
            Pose2d target =
                AllianceFlipUtil.apply(new Pose2d(FieldConstants.Hub.centerHub, new Rotation2d()));
            Pose2d currPose2d = swerve.state.getGlobalPoseEstimate();
            return target.minus(currPose2d).getRotation();
        };
        path.done().onTrue(new TurnToRotation(swerve, rotSup, true)
            .andThen(intake.jerkIntake().alongWith(shooter.shoot(1))));

        return routine;
    }

    /**
     * Move to a specified X,Y and shoot
     *
     * @return AutoRoutine
     */
    public AutoRoutine justShoot() {
        Supplier<Pose2d> poseSup = () -> {
            double x = SmartDashboard.getNumber(Constants.DashboardValues.shootX, 0);
            double y = SmartDashboard.getNumber(Constants.DashboardValues.shootY, 0);
            Pose2d hub =
                AllianceFlipUtil.apply(new Pose2d(FieldConstants.Hub.centerHub, new Rotation2d()));
            Pose2d target = new Pose2d(x, y, new Rotation2d());
            Rotation2d angle = hub.getTranslation().minus(target.getTranslation()).getAngle()
                .plus(Rotation2d.fromDegrees(180));
            return new Pose2d(target.getX(), target.getY(), angle);
        };

        AutoRoutine routine = autoFactory.newRoutine("Just Shoot");
        MoveToPose moveToStart = swerve.moveToPose().target(poseSup).autoRoutine(routine).finish();
        routine.active().onTrue(moveToStart);
        moveToStart.done().onTrue(shooter.shoot(0));
        return routine;
    }
}
