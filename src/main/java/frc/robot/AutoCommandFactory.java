package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Hub;
import frc.robot.FieldConstants.Tower;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.MoveToPose;
import frc.robot.subsystems.turret.Turret;

public class AutoCommandFactory {
    public AdjustableHood hood;
    public Climber climber;
    public Indexer indexer;
    public Intake intake;
    public Shooter shooter;
    public Swerve swerve;
    public Turret turrett;
    public AutoFactory autoFactory;

    public AutoCommandFactory(AutoFactory autoFactory, AdjustableHood hood, Climber climber,
        Indexer indexer, Intake intake, Shooter shooter, Swerve swerve, Turret turret) {
        this.hood = hood;
        this.climber = climber;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.swerve = swerve;
        this.turrett = turret;
        this.autoFactory = autoFactory;
    }

    public AutoRoutine example() {
        AutoRoutine routine = autoFactory.newRoutine("Example");
        MoveToPose testMTP = new MoveToPose(swerve, null, null, routine, null, false, 0, 0);

        routine.active()
            .onTrue(Commands.sequence(
                Commands.runOnce(() -> swerve.overridePose(() -> new Pose2d(
                    new Translation2d(Hub.centerHub.getX(), Tower.centerPoint.getY()), null))),
                testMTP));
        return routine;
    }
}
