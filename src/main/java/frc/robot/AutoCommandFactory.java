package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
     * Gather Fuel from the left side and then return and shoot
     *
     * @return AutoRoutine
     */
    public AutoRoutine sweepThenShoot() {

        AutoRoutine routine = autoFactory.newRoutine("Sweep to pickup and shoot");
        double maxX = 7.723;
        Supplier<Pose2d> startpoint = () -> {
            Pose2d pose = new Pose2d(FieldConstants.LinesVertical.starting,
                FieldConstants.LinesHorizontal.rightBlueTrenchCenter.getY(), new Rotation2d());

            if (swerve.getPose().getY() > FieldConstants.LinesVertical.center) {
                pose = new Pose2d(pose.getX(),
                    FieldConstants.LinesHorizontal.leftBlueTrenchCenter.getY(), new Rotation2d());
            }
            return pose;
        };


        Supplier<Pose2d> sweepStart = () -> {
            Pose2d pose =
                new Pose2d(maxX, FieldConstants.LinesHorizontal.rightBlueTrenchCenter.getY(),
                    Rotation2d.fromDegrees(90));

            if (swerve.getPose().getY() > FieldConstants.LinesVertical.center) {
                pose = new Pose2d(pose.getX(),
                    FieldConstants.LinesHorizontal.leftBlueTrenchCenter.getY(),
                    Rotation2d.fromDegrees(-90));
            }
            return pose;
        };

        Supplier<Pose2d> sweepEnd = () -> {
            boolean fullWidthSweep =
                SmartDashboard.getBoolean(Constants.DashboardValues.fullWidthSweep, false);
            double endY = fullWidthSweep
                ? FieldConstants.LinesHorizontal.center - Constants.Swerve.bumperFront.in(Meters)
                : FieldConstants.LinesHorizontal.leftBlueTrenchCenter.getY();
            Pose2d pose = new Pose2d(maxX, endY, Rotation2d.fromDegrees(90));

            if (swerve.getPose().getY() > FieldConstants.LinesVertical.center) {
                endY = fullWidthSweep
                    ? FieldConstants.LinesHorizontal.center
                        + Constants.Swerve.bumperFront.in(Meters)
                    : FieldConstants.LinesHorizontal.rightBlueTrenchCenter.getY();
                pose = new Pose2d(pose.getX(), endY, Rotation2d.fromDegrees(-90));
            }
            return pose;
        };

        Supplier<Pose2d> sweepStart2 = () -> {
            Pose2d pose = sweepStart.get();
            return new Pose2d(pose.getX(), pose.getY(),
                pose.getRotation().plus(Rotation2d.k180deg));
        };

        Supplier<Pose2d> end = () -> {
            Pose2d pose = startpoint.get();
            pose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(-90));
            if (swerve.getPose().getY() > FieldConstants.LinesVertical.center) {
                pose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(90));
            }
            return pose;
        };

        MoveToPose moveToStart =
            swerve.moveToPose().target(startpoint).autoRoutine(routine).finish();
        MoveToPose moveToSweepStart =
            swerve.moveToPose().target(sweepStart).autoRoutine(routine).finish();
        MoveToPose moveSweep = swerve.moveToPose().target(sweepEnd).autoRoutine(routine).finish();
        TurnToRotation turnAround = new TurnToRotation(swerve, 180, true).setAutoRoutine(routine);
        MoveToPose moveSweep2 =
            swerve.moveToPose().target(sweepStart2).autoRoutine(routine).finish();
        MoveToPose moveToEnd = swerve.moveToPose().target(end).autoRoutine(routine).finish();


        routine.active().onTrue(moveToStart).onTrue(Commands.print("Running Move To Start"));
        moveToStart.done().onTrue(moveToSweepStart)
            .onTrue(Commands.print("Running Move To Sweep Start"));
        moveToSweepStart.active().debounce(2).onTrue(intake.extendAndIntake());
        moveToSweepStart.done().onTrue(moveSweep);
        moveSweep.done().onTrue(turnAround).onTrue(Commands.print("Running Turning Around"));
        turnAround.done().onTrue(moveSweep2).onTrue(Commands.print("Running Move To Sweep Start"));
        moveSweep2.done().onTrue(moveToEnd).onTrue(Commands.print("Running Move To Start"));
        moveToEnd.done().onTrue(intake.jerkIntake().alongWith(CommandFactory
            .shootAtHub(swerve.state, turret, shooter, indexer, adjustableHood, null, null)));

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
