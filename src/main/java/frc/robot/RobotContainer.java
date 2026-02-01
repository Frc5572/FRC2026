package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import org.ironmaple.simulation.SimulatedArena;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.sim.SimulatedRobotState;
import frc.robot.subsystems.adjustable_hood.AdjustableHood;
import frc.robot.subsystems.adjustable_hood.AdjustableHoodIOEmpty;
import frc.robot.subsystems.adjustable_hood.AdjustableHoodReal;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberReal;
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
import frc.robot.subsystems.turret.TurretReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOEmpty;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.color.ColorDetection;
import frc.robot.subsystems.vision.color.ColorDetectionIO;
import frc.robot.subsystems.vision.color.ColorDetectionReal;
import frc.robot.util.DeviceDebug;
import frc.robot.util.Tuples;
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
    public final CommandXboxController tester = new CommandXboxController(1);

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
                turret = new Turret(new TurretReal());
                shooter = new Shooter(new ShooterReal());
                intake = new Intake(new IntakeReal());
                climber = new Climber(new ClimberReal());
                indexer = new Indexer(new IndexerReal());

                colorDetection = new ColorDetection(new ColorDetectionReal());
                break;
            case kSimulation:
                SimulatedArena.getInstance().resetFieldForAuto();
                sim = new SimulatedRobotState(new Pose2d(2.0, 2.0, Rotation2d.kZero));
                swerve = new Swerve(sim.swerveDrive::simProvider, sim.swerveDrive::gyroProvider,
                    sim.swerveDrive::moduleProvider);
                vision = new Vision(swerve.state, sim.visionSim);
                adjustableHood = new AdjustableHood(sim.adjustableHood);
                turret = new Turret(sim.turret);
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
                turret = new Turret(new TurretIOEmpty());
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

        driver.povLeft().onTrue(turret.goToAngle(Degrees.of(-90)))
            .onFalse(turret.goToAngle(Degrees.of(0)));
        driver.povRight().onTrue(turret.goToAngle(Degrees.of(90)))
            .onFalse(turret.goToAngle(Degrees.of(0)));
        driver.a().onTrue(adjustableHood.goToAngle(Degrees.of(45)))
            .onFalse(adjustableHood.goToAngle(Degrees.of(0)));
        driver.b().onTrue(intake.useHopperCommand(Constants.IntakeConstants.hopperOutDistance))
            .onFalse(intake.useHopperCommand(Meters.of(0)));
        driver.leftBumper()
            .onTrue(climber.moveTo(() -> new Tuples.Tuple2<>(Degrees.of(0), Meters.of(0.5))));
        driver.rightBumper()
            .onTrue(climber.moveTo(() -> new Tuples.Tuple2<>(Degrees.of(0), Meters.of(0))));
    }

    /** Runs once per 0.02 seconds after subsystems and commands. */
    public void periodic() {
        if (sim != null) {
            SimulatedArena.getInstance().simulationPeriodic();
            Logger.recordOutput("FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
            sim.update();
        }
        viz.periodic();

    }
}


