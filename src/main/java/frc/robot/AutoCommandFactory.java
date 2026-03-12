package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import java.util.Set;
import java.util.function.Supplier;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.util.Tuples.Tuple2;

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
     * The method that makes the Shoot Then Climb AutoRoutine.
     * 
     * @return The AutoRoutine
     */
    public AutoRoutine shootThenClimb() {
        AutoRoutine routine = autoFactory.newRoutine("Shoot Then Climb");

        MoveToPose moveToClimb = moveToClimb(routine);

        Command fullCommand = Commands.defer(() -> {
            return Commands.sequence(
                Commands.parallel(Commands.sequence(Commands.waitSeconds(2.75), moveToClimb)),
                climber.moveTo(() -> new Tuple2<Angle, Distance>(Degrees.of(0), Meters.of(0.526))));
        }, Set.of(climber));

        routine.active().onTrue(fullCommand);
        routine.active().and(moveToClimb.active().negate().and(moveToClimb.done().negate()))
            .whileTrue(CommandFactory.shoot(swerve.state, () -> {
                return AllianceFlipUtil.apply(FieldConstants.Hub.centerHub);
            }, turret, shooter, indexer, adjustableHood, () -> 1.5, () -> 0.0, () -> false)
                .alongWith(intake.jerkIntake()).withTimeout(2.75));

        moveToClimb.done().onTrue(swerve.stop().andThen(Commands.waitSeconds(3),
            climber.moveTo(() -> new Tuple2<Angle, Distance>(Degrees.of(0), Meters.of(0.4)))));
        return routine;
    }

    private static final Pose2d climbPose = AllianceFlipUtil.apply(new Pose2d(
        FieldConstants.Tower.centerPoint.getX() + Constants.Swerve.bumperRight.in(Meter)
            - Units.inchesToMeters(0.3),
        FieldConstants.Tower.centerPoint.getY() + Units.inchesToMeters(0),
        Rotation2d.fromDegrees(0)));

    private MoveToPose moveToClimb(AutoRoutine routine) {
        return swerve.moveToPose().target(climbPose).autoRoutine(routine).maxSpeed(2.5)
            .translationTolerance(0.05).finish();
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
        double shootingTime = 6.0;
        return Commands
            .sequence(sweep(left, true, 8.076),
                CommandFactory
                    .shoot(
                        swerve.state, () -> AllianceFlipUtil
                            .apply(FieldConstants.Hub.centerHub),
                        turret, shooter, indexer, adjustableHood, () -> 0.0, () -> 0.0, () -> false)
                    .alongWith(intake.jerkIntake()).withTimeout(shootingTime),
                Commands
                    .sequence(adjustableHood.setGoal(Rotations.of(0)), sweep(left, false, 6.0),
                        CommandFactory
                            .shoot(swerve.state,
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.centerHub), turret,
                                shooter, indexer, adjustableHood, () -> 0.0, () -> 0.0, () -> false)
                            .alongWith(intake.jerkIntake()).withTimeout(shootingTime),
                        adjustableHood.setGoal(Rotations.of(0)), sweep(left, false, 8.076),
                        CommandFactory
                            .shoot(swerve.state,
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.centerHub), turret,
                                shooter, indexer, adjustableHood, () -> 0.0, () -> 0.0, () -> false)
                            .alongWith(intake.jerkIntake()).withTimeout(shootingTime))
                    .repeatedly());
    }

    private Command sweep(boolean left, boolean isFirst, double xMeters) {
        return Commands
            .sequence(Commands
                .sequence(
                    swerve.moveToPose()
                        .target(new Pose2d(5.7, 0.622,
                            isFirst ? Rotation2d.kCCW_90deg : Rotation2d.kZero))
                        .maxSpeed(1.5).translationTolerance(0.5).rotationTolerance(15).flipY(left)
                        .finish(),
                    swerve.moveToPose().target(new Pose2d(xMeters, 1.267, Rotation2d.kCCW_90deg))
                        .maxSpeed(1.5).translationTolerance(0.5).rotationTolerance(15).flipY(left)
                        .finish().deadlineFor(intake.extendHopper(0.0)),
                    swerve.moveToPose().target(new Pose2d(xMeters, 4.5, Rotation2d.kCCW_90deg))
                        .maxSpeed(1.5).translationTolerance(0.5).rotationTolerance(15).flipY(
                            left)
                        .finish().deadlineFor(intake.intakeBalls()),
                    swerve.moveToPose().target(new Pose2d(xMeters, 1.267, Rotation2d.kCCW_90deg))
                        .maxSpeed(1.5).translationTolerance(0.5).rotationTolerance(
                            15)
                        .flipY(left).finish(),
                    swerve
                        .moveToPose().target(
                            new Pose2d(6.0, 0.622, Rotation2d.kZero))
                        .maxSpeed(
                            1.5)
                        .translationTolerance(0.1).rotationTolerance(5).flipY(left).finish()),
                Commands
                    .sequence(swerve.moveToPose().target(new Pose2d(4.04, 0.622, Rotation2d.kZero))
                        .maxSpeed(1.5).translationTolerance(0.1).rotationTolerance(5).flipY(left)
                        .finish(), swerve.stop())
                    .deadlineFor(shooter.shoot(60.0)))
            .deadlineFor(CommandFactory.followHub(turret, swerve, () -> 0.0));
    }
}
