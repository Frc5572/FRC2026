package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommandFactory {

    AutoFactory autoFactory;
    Swerve swerve;
    AdjustableHood adjustableHood;
    Climber climber;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    Turret turret;

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

    public AutoRoutine shootThenClimbAuto() {
        AutoRoutine routine = autoFactory.newRoutine("Shoot Then Climb");

        Pose2d climbPose = AllianceFlipUtil.apply(new Pose2d(
            FieldConstants.Tower.centerPoint.getX() + Constants.Swerve.bumperRight.in(Meter)
                - Units.inchesToMeters(3),
            FieldConstants.Tower.centerPoint.getY(), Rotation2d.fromDegrees(180)));

        routine.trajectory("shootThenClimbRight");

        return routine;
    }
}
