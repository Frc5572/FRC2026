package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import org.ironmaple.simulation.SimulatedArena;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.sim.FuelSim;
import frc.robot.sim.SimulatedRobotState;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.adjustable_hood.AdjustableHoodIOEmpty;
import frc.robot.subsystems.adjustable_hood.AdjustableHoodReal;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOEmpty;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOEmpty;
import frc.robot.subsystems.indexer.IndexerReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOEmpty;
import frc.robot.subsystems.intake.IntakeReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOEmpty;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOEmpty;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.gyro.GyroIOEmpty;
import frc.robot.subsystems.swerve.gyro.GyroNavX2;
import frc.robot.subsystems.swerve.mod.SwerveModuleIOEmpty;
import frc.robot.subsystems.swerve.mod.SwerveModuleReal;
import frc.robot.subsystems.swerve.util.TeleopControls;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOEmpty;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOEmpty;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.color.ColorDetection;
import frc.robot.subsystems.vision.color.ColorDetectionIO;
import frc.robot.util.DeviceDebug;
import frc.robot.viz.RobotViz;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@NullMarked
public final class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver =
        new CommandXboxController(Constants.DriverControls.controllerId);

    /* Subsystems */
    private final Swerve swerve;
    private final Vision vision;
    private final AdjustableHood adjustableHood;
    private final Turret turret;
    private final Shooter shooter;
    private final Intake intake;
    private final ColorDetection colorDetection;
    private final Climber climber;
    private final Indexer indexer;
    private final RobotViz viz;
    private final SimulatedRobotState sim;

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        switch (runtimeType) {
            case kReal:
                sim = null;
                swerve = new Swerve(SwerveReal::new, GyroNavX2::new, SwerveModuleReal::new);
                vision = new Vision(swerve.state, new VisionReal());
                adjustableHood = new AdjustableHood(new AdjustableHoodReal());
                turret = new Turret(new TurretIOEmpty(), swerve.state);
                shooter = new Shooter(new ShooterReal());
                intake = new Intake(new IntakeReal());
                climber = new Climber(new ClimberIOEmpty());
                indexer = new Indexer(new IndexerReal());

                colorDetection = new ColorDetection(new ColorDetectionIO.Empty());
                break;
            case kSimulation:
                FuelSim.getInstance().spawnStartingFuel();
                sim = new SimulatedRobotState(new Pose2d(2.0, 2.0, Rotation2d.kZero));
                FuelSim.getInstance().registerRobot(Constants.Swerve.bumperFront.in(Meters) * 2,
                    Constants.Swerve.bumperRight.in(Meters), Units.inchesToMeters(5.0),
                    () -> sim.swerveDrive.mapleSim.getSimulatedDriveTrainPose(),
                    () -> sim.swerveDrive.mapleSim
                        .getDriveTrainSimulatedChassisSpeedsFieldRelative());
                FuelSim.getInstance().registerIntake(Constants.Swerve.bumperFront.in(Meters),
                    Constants.Swerve.bumperFront.in(Meters) + Units.inchesToMeters(7),
                    -Constants.Swerve.bumperRight.in(Meters),
                    Constants.Swerve.bumperRight.in(Meters), () -> sim.intake.isIntaking, () -> {
                        sim.indexer.addFuel();
                    });
                FuelSim.getInstance().start();
                swerve = new Swerve(sim.swerveDrive::simProvider, sim.swerveDrive::gyroProvider,
                    sim.swerveDrive::moduleProvider);
                vision = new Vision(swerve.state, sim.visionSim);
                adjustableHood = new AdjustableHood(sim.adjustableHood);
                turret = new Turret(sim.turret, swerve.state);
                shooter = new Shooter(sim.shooter);
                intake = new Intake(sim.intake);
                climber = new Climber(sim.climber);
                indexer = new Indexer(sim.indexer);

                colorDetection = new ColorDetection(new ColorDetectionIO.Empty());
                break;
            default:
                sim = null;
                swerve = new Swerve(SwerveIOEmpty::new, GyroIOEmpty::new, SwerveModuleIOEmpty::new);
                vision = new Vision(swerve.state, new VisionIOEmpty());
                adjustableHood = new AdjustableHood(new AdjustableHoodIOEmpty());
                turret = new Turret(new TurretIOEmpty(), swerve.state);
                shooter = new Shooter(new ShooterIOEmpty());
                intake = new Intake(new IntakeIOEmpty());
                climber = new Climber(new ClimberSim());
                indexer = new Indexer(new IndexerIOEmpty());

                colorDetection = new ColorDetection(new ColorDetectionIO.Empty());
                break;
        }
        viz = new RobotViz(sim, swerve, turret, adjustableHood, intake, climber);

        DeviceDebug.initialize();


        swerve.setDefaultCommand(swerve.driveUserRelative(TeleopControls.teleopControls(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())));

        driver.y().onTrue(swerve.setFieldRelativeOffset());

        driver.rightTrigger().whileTrue(shooter.shoot(65)).onFalse(shooter.shoot(0));

        driver.leftTrigger().whileTrue(indexer.setSpeedCommand(0.8, 0.8))
            .onFalse(indexer.setSpeedCommand(0.0, 0.0));

        driver.a().whileTrue(intake.extendHopper()).onFalse(intake.stop());
        driver.b().onTrue(intake.retractHopper()).onFalse(intake.stop());
        driver.x().whileTrue(intake.intakeBalls(0.7));
    }

    /** Runs once per 0.02 seconds after subsystems and commands. */
    public void periodic() {
        if (sim != null) {
            SimulatedArena.getInstance().simulationPeriodic();
            FuelSim.getInstance().updateSim();
            sim.update();
        }
        viz.periodic();

    }
}


