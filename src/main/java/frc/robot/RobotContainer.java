package frc.robot;

import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.viz.BallSim;
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
    private final BallSim ballSim;
    private final RobotViz viz;

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        switch (runtimeType) {
            case kReal:
                sim = null;
                ballSim = null;
                swerve = new Swerve(SwerveReal::new, GyroNavX2::new, SwerveModuleReal::new);
                vision = new Vision(swerve.state, new VisionReal());
                break;
            case kSimulation:
                sim = new SwerveSim(new Pose2d(2.0, 2.0, Rotation2d.kZero));
                ballSim = new BallSim();
                swerve = new Swerve(sim::simProvider, sim::gyroProvider, sim::moduleProvider);
                vision = new Vision(swerve.state, new VisionSim(sim));
                break;
            default:
                sim = null;
                ballSim = null;
                swerve = new Swerve(SwerveIOEmpty::new, GyroIOEmpty::new, SwerveModuleIOEmpty::new);
                vision = new Vision(swerve.state, new VisionIOEmpty());
        }
        viz = new RobotViz(sim, swerve);

        DeviceDebug.initialize();

        swerve.setDefaultCommand(swerve.driveUserRelative(TeleopControls.teleopControls(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())));

        driver.y().onTrue(swerve.setFieldRelativeOffset());

        driver.a().onTrue(Commands.runOnce(() -> {
            if (ballSim != null) {
                ballSim.spawnBall(
                    new Translation3d(swerve.state.getGlobalPoseEstimate().getTranslation())
                        .plus(new Translation3d(0, 0, 1)),
                    new Translation3d(5, 0, 5));
            }
        }));
        driver.b().whileTrue(swerve.feedforwardCharacterization()).onFalse(swerve.emergencyStop());
    }

    /** Runs once per 0.02 seconds after subsystems and commands. */
    public void periodic() {
        if (ballSim != null) {
            ballSim.periodic();
        }
        viz.periodic();
    }

}
