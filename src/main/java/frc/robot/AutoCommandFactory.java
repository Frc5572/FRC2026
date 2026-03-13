package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import java.util.function.Supplier;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        moveToStart.done().onTrue(CommandFactory.shoot(swerve.state, () -> {
            return AllianceFlipUtil.apply(FieldConstants.Hub.centerHub);
        }, turret, shooter, indexer, adjustableHood, () -> 0, () -> 0, () -> true));
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
        double shootingTime = 3.0;
        double driveSpeed = 2.5;
        return Commands
            .sequence(sweep(left, true, 8.076, driveSpeed),
                CommandFactory
                    .shoot(swerve.state, () -> AllianceFlipUtil.apply(FieldConstants.Hub.centerHub),
                        turret, shooter, indexer, adjustableHood, () -> 0.0, () -> left ? 5
                            : -5,
                        () -> false)
                    .alongWith(intake.jerkIntake()).withTimeout(shootingTime),
                Commands.sequence(adjustableHood.setGoal(Rotations.of(0)),
                    sweep(left, false, 6.0, driveSpeed),
                    CommandFactory
                        .shoot(swerve.state,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.centerHub), turret,
                            shooter, indexer, adjustableHood, () -> 0.0, () -> 0.0, () -> false)
                        .alongWith(intake.jerkIntake()).withTimeout(shootingTime),
                    adjustableHood.setGoal(Rotations.of(0)), sweep(left, false, 8.076, driveSpeed),
                    CommandFactory
                        .shoot(swerve.state,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.centerHub), turret,
                            shooter, indexer, adjustableHood, () -> 0.0, () -> 0.0, () -> false)
                        .alongWith(intake.jerkIntake()).withTimeout(shootingTime))
                    .repeatedly());
    }

    private Command sweep(boolean left, boolean isFirst, double xMeters, double driveSpeed) {
        return Commands
            .sequence(Commands
                .sequence(
                    swerve
                        .moveToPose()
                        .target(new Pose2d(5.7, 0.622,
                            isFirst ? Rotation2d.kCCW_90deg : Rotation2d.kZero))
                        .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15)
                        .flipY(left).finish(),
                    swerve.moveToPose().target(new Pose2d(xMeters, 1.267, Rotation2d.kCCW_90deg))
                        .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15)
                        .flipY(left).finish().deadlineFor(intake.extendHopper(0.0)),
                    swerve.moveToPose().target(new Pose2d(xMeters, 4.5, Rotation2d.kCCW_90deg))
                        .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15).flipY(
                            left)
                        .finish().deadlineFor(intake.intakeBalls()),
                    swerve.moveToPose().target(new Pose2d(xMeters, 1.267, Rotation2d.kCCW_90deg))
                        .maxSpeed(driveSpeed).translationTolerance(0.5).rotationTolerance(15).flipY(
                            left)
                        .finish(),
                    swerve
                        .moveToPose().target(
                            new Pose2d(6.0, 0.622, Rotation2d.kZero))
                        .maxSpeed(
                            driveSpeed)
                        .translationTolerance(0.2).rotationTolerance(8).flipY(left).finish()),
                Commands
                    .sequence(swerve.moveToPose().target(new Pose2d(4.04, 0.622, Rotation2d.kZero))
                        .maxSpeed(1.5).translationTolerance(0.1).rotationTolerance(5).flipY(left)
                        .finish(), swerve.stop())
                    .deadlineFor(shooter.shoot(60.0)))
            .deadlineFor(CommandFactory.followHub(turret, swerve, () -> 0.0));
    }
}
