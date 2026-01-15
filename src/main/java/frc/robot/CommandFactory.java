package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.TeleopControls;
import frc.robot.subsystems.vision.color.ColorDetection;

/** Command Factory */
public class CommandFactory {

    /**
     * Orents robot to point at fuel
     * 
     * @param color color detection subsystem to get yaw to turn to
     * @param swerve swerve drive
     * @param controller controller inputs
     * @return robot orented at fuel
     */
    public static Command turnToFuel(ColorDetection color, Swerve swerve,
        CommandXboxController controller) {
        final PIDController pidFuel = new PIDController(0, 0, 0);
        pidFuel.enableContinuousInput(-Math.PI, Math.PI);
        return swerve.driveUserRelative(TeleopControls.teleopControls(() -> controller.getRightY(),
            () -> controller.getRightX(), () -> {
                if (Timer.getFPGATimestamp() - color.lastSeenYellow() > 0.1) {
                    return -controller.getRightX();
                } else {
                    double output = pidFuel.calculate(color.getYaw().in(Radians));

                    Logger.recordOutput("objectYaw",
                        MathUtil.angleModulus(color.getYaw().in(Radians)));

                    Logger.recordOutput("objectYawOutput", output);
                    return output;

                }
            }));
    }
}
