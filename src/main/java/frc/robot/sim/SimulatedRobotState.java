package frc.robot.sim;

import static edu.wpi.first.units.Units.Radians;
import java.util.Random;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.ShotData;
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

    private static final double avgBallsPerSecond = 4.0;

    private final Random random;

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
        this.random = new Random(5572);
        this.swerveDrive = new SwerveSim(initialPose);
        this.turret = new TurretSim(random);
        this.adjustableHood = new AdjustableHoodSim();
        this.shooter = new ShooterSim();
        this.intake = new IntakeSim();
        this.indexer = new IndexerSim();
        this.climber = new ClimberSim();
        this.visionSim = new VisionSim();
    }


    /** Get the drivetrain pose. */
    public Pose3d getGroundTruthPose() {
        return new Pose3d(this.swerveDrive.mapleSim.getSimulatedDriveTrainPose());
    }

    /** Update the simulation. Must be called once per iteration. */
    public void update() {
        visionSim.updateState(getGroundTruthPose(), Radians.of(turret.turrentAngle.position));

        double avgBallsPerTick = avgBallsPerSecond * TimedRobot.kDefaultPeriod;

        Logger.recordOutput("FuelSim/indexerIsFeeding", this.indexer.isFeeding);
        if (this.indexer.isFeeding && this.indexer.numFuel > 0) {
            double p = random.nextDouble();
            if (p < avgBallsPerTick) {
                double speedRotationsPerSecond =
                    (shooter.flywheel.position + 0.02 * random.nextFloat() - 0.01) * 0.4;
                Logger.recordOutput("FuelSim/speedRotationsPerSecond", speedRotationsPerSecond);
                shooter.shootOne();
                double effectiveHoodAngle =
                    adjustableHood.hood.position + 0.02 * random.nextFloat() - 0.01;
                double effectiveTurretAngle = this.swerveDrive.mapleSim.getSimulatedDriveTrainPose()
                    .getRotation().getRadians() + turret.turrentAngle.position
                    + 0.02 * random.nextFloat() - 0.01;
                ShotData.ShotEntry entry =
                    ShotData.flywheelSpeedHoodAngle.query(new Translation2d(speedRotationsPerSecond,
                        Units.radiansToDegrees(effectiveHoodAngle))).value();
                double vert = entry.verticalVelocity();
                double horiz = entry.horizontalVelocity();
                double x = Math.cos(effectiveTurretAngle) * horiz;
                double y = Math.sin(effectiveTurretAngle) * horiz;
                Translation3d initial =
                    new Pose3d(swerveDrive.mapleSim.getSimulatedDriveTrainPose())
                        .plus(new Transform3d(-0.1651, 0.0, 0.367722, Rotation3d.kZero))
                        .getTranslation();
                Translation3d velocity = new Translation3d(x, y, vert);
                FuelSim.getInstance().spawnFuel(initial, velocity);
                this.indexer.numFuel--;
            }
        }
    }

}
