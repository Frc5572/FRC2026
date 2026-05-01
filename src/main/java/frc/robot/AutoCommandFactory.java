package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitSupplierCommand;
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
    BooleanSupplier fieldSide = () -> {
        return AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
            .getY() > FieldConstants.fieldWidth / 2.0;
    };
    BooleanSupplier shootFirst =
        () -> SmartDashboard.getBoolean(Constants.DashboardValues.shootFirst, false);
    BooleanSupplier fullWidth =
        () -> SmartDashboard.getBoolean(Constants.DashboardValues.fullWidth, false);
    BooleanSupplier rampOrTranchEnd =
        () -> SmartDashboard.getBoolean(Constants.DashboardValues.rampOrTrenchEnd, false);
    DoubleSupplier x1 =
        () -> SmartDashboard.getNumber(Constants.DashboardValues.x1, Constants.Auto.wilsonTestX);
    DoubleSupplier x2 =
        () -> SmartDashboard.getNumber(Constants.DashboardValues.x2, Constants.Auto.wilsonTestX2);
    DoubleSupplier feetPastCenter =
        () -> SmartDashboard.getNumber(Constants.DashboardValues.feetPastCenter,
            Constants.DashboardValues.feetPastCenterDefault);
    BooleanSupplier secondSweep =
        () -> SmartDashboard.getBoolean(Constants.DashboardValues.secondSweep, false);

    Supplier<Pose2d> shootPoseSupplier = () -> {
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
    final double driveSpeed = 6.0;
    final double intakeSpeed = 1.2;

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

    public Command shootFirst() {
        return autoShooting(2).andThen(adjustableHood.setGoal(Degree.of(0)),
            Commands.waitSeconds(0.25));
    }

    /**
     * CMP Special
     *
     * @return AutoRoutine
     */
    public AutoRoutine cmpSpecial() {

        AutoRoutine routine = autoFactory.newRoutine("CMP Special");

        Command shootOrNot = Commands.either(shootFirst(), Commands.none(), shootFirst);
        Command halfSweep = Commands.either(wilsonTestSide(true), wilsonTestSide(false), fieldSide);

        Command fullsweep =
            Commands.either(fullSweep(routine, true), fullSweep(routine, false), fieldSide);
        Command sweep = Commands.either(fullsweep, halfSweep, fullWidth);

        routine.active().onTrue(new SequentialCommandGroup(shootOrNot, sweep));

        return routine;
    }


    /**
     * Move to a specified X,Y and shoot
     *
     * @return AutoRoutine
     */
    public AutoRoutine justShoot() {

        AutoRoutine routine = autoFactory.newRoutine("Just Shoot");
        MoveToPose moveToStart = swerve.moveToPose().target(shootPoseSupplier).autoRoutine(routine)
            .flipY(false).finish();
        routine.active().onTrue(moveToStart);
        moveToStart.done().onTrue(autoShooting(15));
        return routine;
    }

    /** Cross ramp once, pick up balls, come back over same ramp, shoot. */
    public AutoRoutine rampAuto() {
        AutoRoutine routine = autoFactory.newRoutine("rampAuto");
        routine.active()
            .onTrue(
                crossRampIntoCenter(routine)
                    .andThen(
                        swerve.moveToPose().target(new Pose2d(5.7, 0.622, Rotation2d.kCCW_90deg))
                            .maxSpeed(1.2).translationTolerance(1.5).rotationTolerance(15).flipY(
                                true)
                            .finish(),
                        swerve.moveToPose().target(new Pose2d(8.076, 1.267, Rotation2d.kCCW_90deg))
                            .maxSpeed(1.2).translationTolerance(
                                0.5)
                            .rotationTolerance(15).flipY(true).finish()
                            .alongWith(intake.extendHopper(0.0)),
                        swerve.moveToPose().target(new Pose2d(8.076,
                            (FieldConstants.fieldWidth / 2.0) + Units.feetToMeters(
                                SmartDashboard.getNumber(Constants.DashboardValues.feetPastCenter,
                                    Constants.DashboardValues.feetPastCenterDefault)),
                            Rotation2d.kCCW_90deg)).maxSpeed(intakeSpeed).translationTolerance(0.5)
                            .rotationTolerance(15).flipY(true).finish().deadlineFor(
                                intake.extendHopper(1.0).andThen(
                                    intake.intakeBalls().alongWith(indexer.spinWhileIntake()))),
                        swerve.moveToPose()
                            .target(new Pose2d(5.8537397384643555, 2.05923399925231934,
                                Rotation2d.fromDegrees(45)))
                            .maxSpeed(intakeSpeed).translationTolerance(0.5).rotationTolerance(15)
                            .flipY(true).finish()
                            .deadlineFor(intake.extendHopper(1.0).andThen(
                                intake.intakeBalls().alongWith(indexer.spinWhileIntake()))),
                        crossRampIntoZone(routine), swerve.emergencyStop(),
                        CommandFactory.shoot(swerve.state, shooter, indexer, adjustableHood)
                            .alongWith(
                                Commands.waitSeconds(2.0).andThen(intake.retractHopper(1.0)))));
        return routine;
    }

    /** Cross ramp into the neutral zone. */
    public Command crossRampIntoCenter(AutoRoutine routine) {
        return swerve.moveToPose().autoRoutine(routine).target(() -> {
            return new Pose2d(FieldConstants.fieldLength / 2.0,
                swerve.state.getGlobalPoseEstimate().getY(),
                swerve.state.getGlobalPoseEstimate().getRotation());
        }).flipForRed(false).flipY(false).maxSpeed(2.5).finish()
            .until(() -> AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
                .getX() > FieldConstants.LeftBump.farLeftCorner.getX() + Units.inchesToMeters(10));
    }

    /** Cross ramp into our alliance zone. */
    public Command crossRampIntoZone(AutoRoutine routine) {
        return swerve.moveToPose().autoRoutine(routine).target(() -> {
            return new Pose2d(AllianceFlipUtil.applyX(0.0),
                swerve.state.getGlobalPoseEstimate().getY(),
                swerve.state.getGlobalPoseEstimate().getRotation());
        }).flipForRed(false).flipY(false).maxSpeed(2.5).finish()
            .until(() -> AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
                .getX() < FieldConstants.LeftBump.nearLeftCorner.getX() - Units.inchesToMeters(10));
    }

    /** Test to make sure autos work. */
    public AutoRoutine wilsonTest() {
        AutoRoutine routine = autoFactory.newRoutine("WilsonTest");
        return wilsonTestBase(routine);
    }

    // public AutoRoutine wilsonTestShort() {
    // AutoRoutine routine = autoFactory.newRoutine("WilsonTestShort");
    // return wilsonTestBase(routine, Constants.Auto.wilsonTestX2,Constants.Auto.wilsonTestX2);
    // }

    /** Base for auto routines. */
    public AutoRoutine wilsonTestBase(AutoRoutine routine) {
        routine.active()
            .onTrue(new ConditionalCommand(wilsonTestSide(true), wilsonTestSide(false), () -> {
                return AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
                    .getY() > FieldConstants.fieldWidth / 2.0;
            }));
        return routine;
    }

    private Command fullSweepCrossField(AutoRoutine routine, double endY, double driveSpeed,
        boolean left) {
        return Commands
            .sequence(
                swerve.moveToPose()
                    .target(() -> new Pose2d(x1.getAsDouble(), endY - Units.feetToMeters(1),
                        Rotation2d.kCCW_90deg))
                    .maxSpeed(intakeSpeed).translationTolerance(0.5).rotationTolerance(15)
                    .flipY(left).finish(),
                swerve.moveToPose()
                    .target(() -> new Pose2d(x1.getAsDouble(), endY, Rotation2d.kZero))
                    .maxSpeed(intakeSpeed).translationTolerance(0.5).rotationTolerance(15)
                    .flipY(left).finish())
            .deadlineFor(intake.extendHopper(1.0)
                .andThen(intake.intakeBalls().alongWith(indexer.spinWhileIntake())));
    }

    private Command fullSweep(AutoRoutine routine, boolean left) {
        double shootingTime = 15;
        double driveSpeed = 6.0;
        // Positive turret trim towards net, negative towards DS

        Command endTrench = Commands.sequence(fullSweepCrossField(routine, 7.420, driveSpeed, left),
            swerve.moveToPose().target(() -> new Pose2d(4.04, 7.420, Rotation2d.kZero))
                .maxSpeed(1.5).translationTolerance(0.1).rotationTolerance(5).flipY(left).finish());
        Command endRamp = Commands.sequence(fullSweepCrossField(routine, 5.655, driveSpeed, left),
            crossRampIntoZone(routine));
        Command ending = Commands.either(
            endRamp.andThen(swerve.emergencyStop(), Commands.waitSeconds(1),
                swerve.moveToPose().target(shootPoseSupplier).maxSpeed(driveSpeed)
                    .translationTolerance(0.5).rotationTolerance(15).flipY(false).finish()),
            endTrench,
            () -> SmartDashboard.getBoolean(Constants.DashboardValues.rampOrTrenchEnd, false));

        Command sequence = Commands
            .sequence(
                swerve.moveToPose().target(new Pose2d(5.7, 0.622, Rotation2d.kCCW_90deg))
                    .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15)
                    .flipY(left).finish(),
                swerve.moveToPose()
                    .target(() -> new Pose2d(x1.getAsDouble(), 1.267, Rotation2d.kCCW_90deg))
                    .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15)
                    .flipY(left).finish().alongWith(intake.extendHopper(0.0)),
                ending, swerve.emergencyStop())
            .deadlineFor(CommandFactory.followHub(turret, swerve, () -> 0.0))
            .andThen(autoShooting(shootingTime));
        return sequence;

    }

    private Command wilsonTestSide(boolean left) {
        double shootingTime = 5.5;
        double driveSpeed = 6.0;
        // Positive turret trim towards net, negative towards DS
        double turretFudge1 = 0;
        double turretFudge2 = 0;
        return Commands
            .sequence(wilsonTestSweep(left, true, x1, driveSpeed).alongWith(Commands.runOnce(() -> {
                swerve.state.setTrims(-0.5, left ? turretFudge1 : -turretFudge1);
            })), autoShooting(shootingTime),
                Commands.sequence(adjustableHood.setGoal(Rotations.of(0)),
                    wilsonTestSweep(left, false, x2, driveSpeed), Commands.runOnce(() -> {
                        swerve.state.setTrims(-0.5, left ? turretFudge2 : -turretFudge2);
                    }), autoShooting(shootingTime), adjustableHood.setGoal(Rotations.of(0)),
                    wilsonTestSweep(left, false, x1, driveSpeed), autoShooting(shootingTime * 2))
                    .repeatedly());
    }

    private Command wilsonTestSweep(boolean left, boolean isFirst, DoubleSupplier xMeters,
        double driveSpeed) {
        return Commands
            .sequence(Commands.sequence(
                swerve
                    .moveToPose()
                    .target(new Pose2d(5.7, 0.622,
                        isFirst ? Rotation2d.kCCW_90deg : Rotation2d.k180deg))
                    .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15)
                    .flipY(left).finish(),
                swerve.moveToPose()
                    .target(() -> new Pose2d(xMeters.getAsDouble(), 1.267, Rotation2d.kCCW_90deg))
                    .maxSpeed(
                        driveSpeed)
                    .translationTolerance(0.5).rotationTolerance(15).flipY(left).finish()
                    .alongWith(intake.extendHopper(0.0)),
                swerve.moveToPose()
                    .target(() -> new Pose2d(xMeters.getAsDouble(),
                        (FieldConstants.fieldWidth / 2.0)
                            + Units.feetToMeters(feetPastCenter.getAsDouble()),
                        Rotation2d.kCCW_90deg))
                    .maxSpeed(intakeSpeed).translationTolerance(0.5).rotationTolerance(15)
                    .flipY(left).finish()
                    .deadlineFor(intake.extendHopper(1.0)
                        .andThen(intake.intakeBalls().alongWith(indexer.spinWhileIntake()))),
                swerve.moveToPose()
                    .target(() -> new Pose2d(xMeters.getAsDouble(), 1.267, Rotation2d.kCCW_90deg))
                    .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(
                        15)
                    .flipY(left).finish(),
                swerve
                    .moveToPose().target(new Pose2d(6.0, 0.622, Rotation2d.k180deg))
                    .maxSpeed(driveSpeed).translationTolerance(0.2).rotationTolerance(8).flipY(left)
                    .finish().withTimeout(3.5)),
                Commands
                    .sequence(
                        swerve.moveToPose().target(new Pose2d(4.04, 0.622, Rotation2d.k180deg))
                            .maxSpeed(1.5).translationTolerance(0.1).rotationTolerance(5)
                            .flipY(left).finish().withTimeout(3.5),
                        swerve.emergencyStop())
                    .deadlineFor(shooter.shoot(60.0)))
            .deadlineFor(CommandFactory.followHub(turret, swerve, () -> 0.0));
    }

    /** Base for auto routines. */
    public AutoRoutine halfSweepTrenchRamp() {
        AutoRoutine routine = autoFactory.newRoutine("halfSweepTrenchRamp");
        routine.active().onTrue(new ConditionalCommand(halfSweepTrenchRamp(routine, true),
            halfSweepTrenchRamp(routine, false), () -> {
                return AllianceFlipUtil.apply(swerve.state.getGlobalPoseEstimate())
                    .getY() > FieldConstants.fieldWidth / 2.0;
            }));
        return routine;
    }

    public Command halfSweepTrenchRampPath(AutoRoutine routine, boolean left) {
        return Commands.sequence(
            swerve.moveToPose().target(new Pose2d(5.7, 0.622, Rotation2d.kCCW_90deg))
                .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15).flipY(left)
                .finish(),
            swerve.moveToPose()
                .target(() -> new Pose2d(x1.getAsDouble(), 1.267, Rotation2d.kCCW_90deg))
                .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15).flipY(left)
                .finish().alongWith(intake.extendHopper(0.0)),
            swerve.moveToPose()
                .target(() -> new Pose2d(x1.getAsDouble(),
                    (FieldConstants.fieldWidth / 2.0)
                        + Units.feetToMeters(feetPastCenter.getAsDouble()),
                    Rotation2d.kCCW_90deg))
                .maxSpeed(intakeSpeed).translationTolerance(0.5).rotationTolerance(15).flipY(left)
                .finish()
                .deadlineFor(intake.extendHopper(1.0)
                    .andThen(intake.intakeBalls().alongWith(indexer.spinWhileIntake()))),
            swerve.moveToPose().target(() -> new Pose2d(x1.getAsDouble(), 2.4, Rotation2d.kZero))
                .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15).flipY(left)
                .finish(),
            swerve.stop(),
            new WaitSupplierCommand(() -> SmartDashboard.getNumber(Constants.DashboardValues.delay2,
                Constants.DashboardValues.delayDefault)),
            crossRampIntoZone(routine),
            swerve.moveToPose().target(shootPoseSupplier).maxSpeed(driveSpeed)
                .translationTolerance(0.5).rotationTolerance(15).flipY(false).finish(),
            swerve.emergencyStop(), autoShooting(5));
    }

    /** Cross trench and then back over ramp and shoot */
    public Command halfSweepTrenchRamp(AutoRoutine routine, boolean left) {
        double driveSpeed = 6.0;
        Command shootOrNot = Commands.either(shootFirst(), Commands.none(), shootFirst);
        Command runPath =
            Commands
                .either(
                    halfSweepTrenchRampPath(routine, left)
                        .andThen(
                            swerve.moveToPose().target(new Pose2d(2.8, 0.622, Rotation2d.kZero))
                                .maxSpeed(driveSpeed).translationTolerance(0.2)
                                .rotationTolerance(15).flipY(left).finish(),
                            adjustableHood.setGoal(Degree.of(0)), Commands.waitSeconds(0.25))
                        .repeatedly(),
                    halfSweepTrenchRampPath(routine, left).andThen(autoShooting(5)), secondSweep);
        return shootOrNot.andThen(runPath);
    }

    /**
     * Shooting Function
     *
     * @param shootingTime Time to shoot
     * @return Command
     */
    Command autoShooting(double shootingTime, double delayTime) {
        return Commands.sequence(Commands.waitSeconds(delayTime),
            CommandFactory.shoot(swerve.state, shooter, indexer, adjustableHood)
                .alongWith(intake.jerkIntake(),
                    turret.goToAngleFieldRelative(
                        () -> swerve.state.getDesiredTurretHeadingFieldRelative()))
                .withTimeout(shootingTime))
            .andThen(intake.retractHopper(1));
    }

    /**
     * Shooting Function
     *
     * @param shootingTime Time to shoot
     * @return Command
     */
    Command autoShooting(double shootingTime) {
        return autoShooting(shootingTime, 0);
    }
}
