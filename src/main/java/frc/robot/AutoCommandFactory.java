package frc.robot;

import java.util.function.Supplier;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        path.active().onTrue(intake.extendHopper().andThen(intake.intakeBalls()));

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
