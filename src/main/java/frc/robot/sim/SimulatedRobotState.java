package frc.robot.sim;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.adjustable_hood.AdjustableHoodSim;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.indexer.IndexerSim;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.turret.TurretSim;
import frc.robot.subsystems.vision.VisionSim;

/** Simulated state of the robot */
public class SimulatedRobotState {

    /** Swerve state */
    public final SwerveSim swerveDrive;
    public final TurretSim turret;
    public final AdjustableHoodSim adjustableHood;
    public final ShooterSim shooter;
    public final IntakeSim intake;
    public final IndexerSim indexer;
    public final ClimberSim climber;
    public final VisionSim visionSim;
    // Turret, intake, and hood state would go here too.

    /** Create new robot simulation */
    public SimulatedRobotState(Pose2d initialPose) {
        this.swerveDrive = new SwerveSim(initialPose);
        this.turret = new TurretSim();
        this.adjustableHood = new AdjustableHoodSim();
        this.shooter = new ShooterSim();
        this.intake = new IntakeSim();
        this.indexer = new IndexerSim();
        this.climber = new ClimberSim();
        this.visionSim = new VisionSim();
    }

    public Pose3d getGroundTruthPose() {
        return new Pose3d(this.swerveDrive.mapleSim.getSimulatedDriveTrainPose());
    }

    public void update() {
        visionSim.updateState(getGroundTruthPose(), Radians.of(turret.turrentAngle.position));
    }

}
