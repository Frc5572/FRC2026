package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.function.Supplier;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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



    public AutoRoutine lawnmower() {
        AutoRoutine routine = autoFactory.newRoutine("lawnmower ");
        MoveToPose travelToNeutralZone = swerve
            .moveToPose().target(new Pose2d(Meters.of(6.637223720550537),
                Meters.of(7.330338001251221), new Rotation2d(Radians.of(-0.7102708052557201))))
            .autoRoutine(routine).finish();
        MoveToPose slam1 = swerve.moveToPose().target(new Pose2d(Meters.of(7.759509086608887),
            Meters.of(5.793294906616211), Rotation2d.kZero)).autoRoutine(routine).finish();
        MoveToPose slam2 = swerve.moveToPose().target(new Pose2d(Meters.of(8.247459411621094),
            Meters.of(4.13426399230957), Rotation2d.k180deg)).autoRoutine(routine).finish();
        MoveToPose slam3 = swerve.moveToPose().target(new Pose2d(Meters.of(7.710713863372803),
            Meters.of(2.6216182708740234), Rotation2d.kZero)).autoRoutine(routine).finish();
        MoveToPose frontTrench = swerve.moveToPose().target(new Pose2d(Meters.of(6.027285575866699),
            Meters.of(0.6454197764396667), Rotation2d.kZero)).autoRoutine(routine).finish();
        MoveToPose pointAtHub = swerve
            .moveToPose().target(new Pose2d(Meters.of(3.3679568767547607),
                Meters.of(0.6942147612571716), new Rotation2d(Radians.of(-1.1966918933365922))))
            .autoRoutine(routine).finish();

        routine.active().onTrue(
            travelToNeutralZone.alongWith(intake.extendHopper().andThen(intake.intakeBalls(0.7))));

        travelToNeutralZone.done().onTrue(slam1);
        slam1.done().onTrue(slam2);
        slam1.done().onTrue(slam3);
        slam3.done().onTrue(frontTrench);
        frontTrench.done().onTrue(pointAtHub
            .alongWith(intake.idle().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

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

    public AutoRoutine peashooter() {
        AutoRoutine routine = autoFactory.newRoutine("peashooter");
        MoveToPose travelToNeutralZone = swerve.moveToPose()
            .target(new Pose2d(Meters.of(7.759509086608887), Meters.of(7.354735374450684),
                Rotation2d.kCW_90deg))
            .autoRoutine(routine).maxSpeed(Constants.Swerve.autoMaxSpeed).flipForRed(true).finish();
        MoveToPose lawnmower = swerve.moveToPose()
            .target(new Pose2d(Meters.of(7.832701206207275), Meters.of(1.2797551155090332),
                Rotation2d.kCW_90deg))
            .autoRoutine(routine).maxSpeed(Constants.Swerve.autoMaxSpeed).flipForRed(true).finish();
        MoveToPose frontTrench = swerve.moveToPose()
            .target(new Pose2d(Meters.of(6.027285575866699), Meters.of(0.6454197764396667),
                Rotation2d.kZero))
            .autoRoutine(routine).maxSpeed(Constants.Swerve.autoMaxSpeed).flipForRed(true).finish();
        MoveToPose pointAtHub = swerve.moveToPose()
            .target(new Pose2d(Meters.of(3.3679568767547607), Meters.of(0.6942147612571716),
                new Rotation2d(Radians.of(-1.1966918933365922))))
            .autoRoutine(routine).maxSpeed(Constants.Swerve.autoMaxSpeed).flipForRed(true).finish();

        routine.active()
            .onTrue(swerve.overridePose(() -> swerve.state.getGlobalPoseEstimate())
                .andThen(travelToNeutralZone
                    .alongWith(intake.extendHopper().andThen(intake.intakeBalls(0.7)))));
        travelToNeutralZone.done().onTrue(lawnmower);
        lawnmower.done().onTrue(frontTrench.alongWith(intake.retractHopper()
            .andThen(intake.idle().withInterruptBehavior(InterruptionBehavior.kCancelIncoming))));
        frontTrench.done().onTrue(pointAtHub);

        return routine;
    }

    /**
     * Move to a specified X,Y and shoot
     *
     * @return AutoRoutine
     */
    public AutoRoutine justShoot() {
        Supplier<Pose2d> poseSup = () -> {
            double x = SmartDashboard.getNumber("Auto Shoot X", 0);
            double y = SmartDashboard.getNumber("Auto Shoot Y", 0);
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
