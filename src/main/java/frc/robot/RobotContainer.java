package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOEmpty;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.swerve.gyro.GyroIOEmpty;
import frc.robot.subsystems.swerve.gyro.GyroNavX2;
import frc.robot.subsystems.swerve.mod.SwerveModuleIOEmpty;
import frc.robot.subsystems.swerve.mod.SwerveModuleReal;
import frc.robot.subsystems.swerve.util.TeleopControls;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOEmpty;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.subsystems.vision.VisionSim;
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

    private final SwerveSim sim;
    private final RobotViz viz;

    private String gamedata;
    private Boolean activeHub;

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        switch (runtimeType) {
            case kReal:
                sim = null;
                swerve = new Swerve(SwerveReal::new, GyroNavX2::new, SwerveModuleReal::new);
                vision = new Vision(swerve.state, new VisionReal());
                break;
            case kSimulation:
                sim = new SwerveSim(new Pose2d(2.0, 2.0, Rotation2d.kZero));
                swerve = new Swerve(sim::simProvider, sim::gyroProvider, sim::moduleProvider);
                vision = new Vision(swerve.state, new VisionSim(sim));
                break;
            default:
                sim = null;
                swerve = new Swerve(SwerveIOEmpty::new, GyroIOEmpty::new, SwerveModuleIOEmpty::new);
                vision = new Vision(swerve.state, new VisionIOEmpty());
        }
        viz = new RobotViz(sim, swerve);

        DeviceDebug.initialize();

        swerve.setDefaultCommand(swerve.driveUserRelative(TeleopControls.teleopControls(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())));

        driver.y().onTrue(swerve.setFieldRelativeOffset());

        driver.a().whileTrue(swerve.wheelRadiusCharacterization()).onFalse(swerve.emergencyStop());
        driver.b().whileTrue(swerve.feedforwardCharacterization()).onFalse(swerve.emergencyStop());
    }

    /** Runs once per 0.02 seconds after subsystems and commands. */
    public void periodic() {
        if (sim != null) {
            SimulatedArena.getInstance().simulationPeriodic();
            Logger.recordOutput("FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
            Logger.recordOutput("FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        }
        viz.periodic();
        gamedata = DriverStation.getGameSpecificMessage();
        if (gamedata.length() > 0) {
            switch (gamedata.charAt(0)) {
                case 'B':
                    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
                        activeHub = true;
                    } else {
                        activeHub = false;
                    }
                    break;
                case 'R':
                    if (DriverStation.getAlliance().equals(Alliance.Red)) {
                        activeHub = true;
                    } else {
                        activeHub = false;
                    }
                default:
                    activeHub = false;
                    break;
            }
        }
    }
}
