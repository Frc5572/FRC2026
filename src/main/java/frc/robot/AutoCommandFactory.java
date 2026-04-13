package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import java.util.function.Supplier;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
     * Move to a specified X,Y and shoot
     *
     * @return AutoRoutine
     */
    public AutoRoutine justShoot() {
        Supplier<Pose2d> poseSup = () -> {
            double x = SmartDashboard.getNumber(Constants.DashboardValues.shootX,
                Constants.DashboardValues.shootXDefault);
            double y = SmartDashboard.getNumber(Constants.DashboardValues.shootY,
                Constants.DashboardValues.shootYDefault);
            Pose2d hub =
                AllianceFlipUtil.apply(new Pose2d(FieldConstants.Hub.centerHub, new Rotation2d()));
            Pose2d target = new Pose2d(x, y, new Rotation2d());
            Rotation2d angle = hub.getTranslation().minus(target.getTranslation()).getAngle();
            return new Pose2d(target.getX(), target.getY(), angle);
        };

        AutoRoutine routine = autoFactory.newRoutine("Just Shoot");
        MoveToPose moveToStart = swerve.moveToPose().target(poseSup).autoRoutine(routine).finish();
        routine.active().onTrue(moveToStart);
        moveToStart.done()
            .onTrue(CommandFactory.shoot(swerve.state, shooter, indexer, adjustableHood));
        return routine;
    }

    /** Test to make sure autos work. */
    public AutoRoutine wilsonTest2() {
        AutoRoutine routine = autoFactory.newRoutine("WilsonTest2");
        routine.active().onTrue(new ConditionalCommand(wilsonTest2Side(routine, true),
            wilsonTest2Side(routine, false), () -> {
                return AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
                    .getY() > FieldConstants.fieldWidth / 2.0;
            }));
        return routine;
    }

    private Command wilsonTest2Side(AutoRoutine routine, boolean left) {
        double turretFudge = 2.5;
        String[] paths = new String[] {"LeftSweep", "LeftSidewinder", "LeftJab"};
        Command c = new InstantCommand();
        for (var path : paths) {
            c = c.andThen(neutralZonePath(routine, left, path),
                CommandFactory.shoot(swerve.state, shooter, indexer, adjustableHood)
                    .withTimeout(6.0).deadlineFor(
                        Commands.waitSeconds(2.5).andThen(intake.retractHopper(0).withTimeout(0.5),
                            Commands.waitSeconds(2.5), intake.retractHopper(0).withTimeout(0.5))));
        }
        return c.repeatedly().deadlineFor(
            CommandFactory.followHub(turret, swerve, () -> left ? turretFudge : -turretFudge));
    }

    private Command neutralZonePath(AutoRoutine routine, boolean left, String name) {
        AutoTrajectory test = routine.trajectory(name, 1);
        var start = test.getRawTrajectory().getInitialSample(false).get();
        Pose2d beforeEnter = test.getFinalPose().get();
        Pose2d end = routine.trajectory(name, 2).getFinalPose().get();

        return Commands.parallel(Commands.runOnce(() -> swerve.flipTrajectories(!left)),
            Commands.sequence(
                swerve.moveToPose().autoRoutine(routine).target(start.getPose()).maxSpeed(4.5)
                    .flipY(!left).translationTolerance(0.5).ignoreRotation(true)
                    .feedforward(Math.hypot(start.getChassisSpeeds().vxMetersPerSecond,
                        start.getChassisSpeeds().vyMetersPerSecond))
                    .flipForRed(true).finish(),
                test.cmd(),
                swerve.moveToPose().autoRoutine(routine).target(beforeEnter).maxSpeed(4.5)
                    .flipY(!left).translationTolerance(0.2).rotationTolerance(5).finish(),
                swerve.moveToPose().autoRoutine(routine).target(end).maxSpeed(4.5).flipY(!left)
                    .translationTolerance(0.2).rotationTolerance(15).finish(),
                swerve.emergencyStop()));
    }

    private Command jab(AutoRoutine routine, boolean left) {
        return neutralZonePath(routine, left, "JabLeft");
    }

    private Command sweep(AutoRoutine routine, boolean left) {
        return neutralZonePath(routine, left, "LeftSweep");
    }

    private Command sidewinder(AutoRoutine routine, boolean left) {
        return neutralZonePath(routine, left, "LeftSidewinder");
    }

    /** Test to make sure autos work. */
    public AutoRoutine wilsonTest() {
        AutoRoutine routine = autoFactory.newRoutine("WilsonTest");
        routine.active()
            .onTrue(new ConditionalCommand(wilsonTestSide(true), wilsonTestSide(false), () -> {
                return AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
                    .getY() > FieldConstants.fieldWidth / 2.0;
            }));
        return routine;
    }

    private Command wilsonTestSide(boolean left) {
        double shootingTime = 5.5;
        double driveSpeed = 2.5;
        double turretFudge = 2.5;
        return Commands.sequence(sweep(left, true, Constants.Auto.wilsonTestX, driveSpeed)
            .alongWith(Commands.runOnce(() -> {
                swerve.state.setTrims(1.0, left ? turretFudge : -turretFudge);
            })),
            CommandFactory.shoot(swerve.state, shooter, indexer, adjustableHood)
                .alongWith(intake.jerkIntake()).withTimeout(shootingTime),
            Commands
                .sequence(adjustableHood.setGoal(Rotations.of(0)),
                    sweep(left, false, 6.5, driveSpeed),
                    CommandFactory.shoot(swerve.state, shooter, indexer, adjustableHood)
                        .alongWith(intake.jerkIntake()).withTimeout(shootingTime),
                    adjustableHood.setGoal(Rotations.of(0)), sweep(left, false, 8.076, driveSpeed),
                    CommandFactory.shoot(swerve.state, shooter, indexer, adjustableHood)
                        .alongWith(intake.jerkIntake()).withTimeout(shootingTime * 2))
                .repeatedly());
    }

    private Command sweep(boolean left, boolean isFirst, double xMeters, double driveSpeed) {
        return Commands
            .sequence(
                Commands.sequence(
                    swerve.moveToPose()
                        .target(new Pose2d(5.7, 0.622,
                            isFirst ? Rotation2d.kCCW_90deg : Rotation2d.kZero))
                        .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15)
                        .flipY(left).finish(),
                    swerve
                        .moveToPose().target(new Pose2d(xMeters, 1.267, Rotation2d.kCCW_90deg))
                        .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15)
                        .flipY(left).finish().alongWith(intake.extendHopper(0.0)),
                    swerve.moveToPose()
                        .target(new Pose2d(xMeters,
                            (FieldConstants.fieldWidth / 2.0) + Units.feetToMeters(
                                SmartDashboard.getNumber(Constants.DashboardValues.feetPastCenter,
                                    Constants.DashboardValues.feetPastCenterDefault)),
                            Rotation2d.kCCW_90deg))
                        .maxSpeed(1.0).translationTolerance(0.5).rotationTolerance(15).flipY(left)
                        .finish()
                        .deadlineFor(intake.extendHopper(1.0).andThen(
                            intake.intakeBalls().alongWith(indexer.spinWhileIntake()))),
                    swerve.moveToPose().target(new Pose2d(xMeters, 1.267, Rotation2d.kCCW_90deg))
                        .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(
                            15)
                        .flipY(left).finish(),
                    swerve
                        .moveToPose().target(
                            new Pose2d(6.0, 0.622, Rotation2d.kZero))
                        .maxSpeed(driveSpeed).translationTolerance(0.2).rotationTolerance(8)
                        .flipY(left).finish().withTimeout(3.5)),
                Commands
                    .sequence(swerve.moveToPose().target(new Pose2d(4.04, 0.622, Rotation2d.kZero))
                        .maxSpeed(1.5).translationTolerance(0.1).rotationTolerance(5).flipY(left)
                        .finish().withTimeout(3.5), swerve.emergencyStop())
                    .deadlineFor(shooter.shoot(60.0)))
            .deadlineFor(CommandFactory.followHub(turret, swerve, () -> 0.0));
    }
}
