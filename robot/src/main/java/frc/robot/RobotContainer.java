package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.localization.DrivetrainState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.CameraProcessor;
import frc.robot.subsystems.vision.Vision;
import frc.robot.targeting.TargetingState;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandXboxController tuner = new CommandXboxController(2);
    private final CommandXboxController pit = new CommandXboxController(3);

    /**
     * Robot Container
     *
     * @param runtimeType Run type
     */
    public RobotContainer(RobotRunType runtimeType) {
        DrivetrainState drivetrainState = new DrivetrainState();
        Swerve swerve = new Swerve(drivetrainState);

        List<CameraProcessor> processors =
            List.of(new CameraProcessor(Constants.Vision.TURRET_CAMERA,
                drivetrainState::getFieldRelativeSpeeds));
        Vision vision = new Vision(null, drivetrainState, processors);

        Shooter shooter = new Shooter();

        TargetingState targeting =
            new TargetingState(drivetrainState::getPose, shooter::getCurrentRps);
    }

    /** Runs once per 0.02 seconds after subsystems and commands. */
    public void periodic() {

    }


    /**
     * Runs during disabled
     */
    public void disabledPeriodic() {

    }
}

