package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.sim.SimulatedRobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO.Empty;
import frc.robot.subsystems.intake.IntakeReal;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOEmpty;
import frc.robot.subsystems.swerve.SwerveReal;
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
    public final CommandXboxController testController = new CommandXboxController(1);

    /* Subsystems */
    private final Swerve swerve;
    private final Vision vision;
    private final Intake intake;

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
                intake = new Intake(new IntakeReal());
                break;
            case kSimulation:
                SimulatedArena.getInstance().resetFieldForAuto();
                sim = new SimulatedRobotState(new Pose2d(2.0, 2.0, Rotation2d.kZero));
                swerve = new Swerve(sim.swerveDrive::simProvider, sim.swerveDrive::gyroProvider,
                    sim.swerveDrive::moduleProvider);
                vision = new Vision(swerve.state, new VisionSim(sim));
                intake = new Intake(new IntakeSim());

                break;
            default:
                sim = null;
                swerve = new Swerve(SwerveIOEmpty::new, GyroIOEmpty::new, SwerveModuleIOEmpty::new);
                vision = new Vision(swerve.state, new VisionIOEmpty());
                intake = new Intake(new Empty());
        }
        viz = new RobotViz(sim, swerve);

        DeviceDebug.initialize();

        swerve.setDefaultCommand(swerve.driveUserRelative(TeleopControls.teleopControls(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())));

        driver.y().onTrue(swerve.setFieldRelativeOffset());

        driver.a().whileTrue(swerve.wheelRadiusCharacterization()).onFalse(swerve.emergencyStop());
        driver.b().whileTrue(swerve.feedforwardCharacterization()).onFalse(swerve.emergencyStop());
        testController.leftTrigger().whileTrue(intake.useIntakeCommand(
            Constants.IntakeConstants.intakeSpeed, Constants.IntakeConstants.hopperOutDistance));
        testController.povDown()
            .whileTrue(intake.useIntakeCommand(0, Constants.IntakeConstants.hopperOutDistance));
    }

    /** Runs once per 0.02 seconds after subsystems and commands. */
    public void periodic() {
        if (sim != null) {
            SimulatedArena.getInstance().simulationPeriodic();
            Logger.recordOutput("FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
        viz.periodic();
    }

}
