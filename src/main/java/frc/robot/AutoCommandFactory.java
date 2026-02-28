package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.MoveToPose;
import frc.robot.subsystems.turret.Turret;

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

        routine.active().onTrue(travelToNeutralZone
            .alongWith(intake.extendHopper().withTimeout(0.5).andThen(intake.intakeBalls(0.7))));

        travelToNeutralZone.done().onTrue(slam1);
        slam1.done().onTrue(slam2);
        slam1.done().onTrue(slam3);
        slam3.done().onTrue(frontTrench);
        frontTrench.done().onTrue(pointAtHub
            .alongWith(intake.idle().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

        return routine;
    }

    public AutoRoutine peashooter() {
        AutoRoutine routine = autoFactory.newRoutine("peashooter");
        MoveToPose travelToNeutralZone =
            swerve
                .moveToPose().target(new Pose2d(Meters.of(7.759509086608887),
                    Meters.of(7.354735374450684), Rotation2d.kCW_90deg))
                .autoRoutine(routine).finish();
        MoveToPose lawnmower =
            swerve
                .moveToPose().target(new Pose2d(Meters.of(7.832701206207275),
                    Meters.of(1.2797551155090332), Rotation2d.kCW_90deg))
                .autoRoutine(routine).finish();
        MoveToPose frontTrench = swerve.moveToPose().target(new Pose2d(Meters.of(6.027285575866699),
            Meters.of(0.6454197764396667), Rotation2d.kZero)).autoRoutine(routine).finish();
        MoveToPose pointAtHub = swerve
            .moveToPose().target(new Pose2d(Meters.of(3.3679568767547607),
                Meters.of(0.6942147612571716), new Rotation2d(Radians.of(-1.1966918933365922))))
            .autoRoutine(routine).finish();

        routine.active().onTrue(travelToNeutralZone
            .alongWith(intake.extendHopper().withTimeout(0.5), intake.intakeBalls(0.7)));
        travelToNeutralZone.done().onTrue(lawnmower);
        lawnmower.done().onTrue(frontTrench.alongWith(intake.retractHopper().withTimeout(0.5),
            intake.intakeBalls(0.0)));
        frontTrench.done().onTrue(pointAtHub);

        return routine;
    }
}
