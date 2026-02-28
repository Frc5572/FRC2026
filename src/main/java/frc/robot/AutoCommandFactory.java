package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
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
    private AutoFactory autoFactory;
);

    public AutoCommandFactory(AdjustableHood hood, Climber climber, Indexer indexer, Intake intake,
        Shooter shooter, Swerve swerve, Turret turrett,AutoFactory autoFactory) {
        this.hood = hood;
        this.climber = climber;
        this.indexer = indexer;
        this.shooter = shooter;
        this.swerve = swerve;
        this.turrett = turrett;
        this.autoFactory = autoFactory;
    }

    public AutoRoutine passThenClimbAuto(){
        AutoRoutine routine = autoFactory.newRoutine("Pass then Climb");
        MoveToPose mtp = new MoveToPose(swerve, null, null, routine, null, false, 0, 0);
    }
    
}
