package frc.robot.subsystems.adjustable_hood;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

/**
 * Adjustable Hood Subsystem
 */
public class AdjustableHood extends SubsystemBase {

    private final AdjustableHoodIO io;
    public final AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();
    public final RobotState state;
    private Translation2d robotPosition;
    private double hubDistance;

    /**
     * Creates a new Adjustable Hood subsystem.
     *
     * @param io Hardware abstraction
     */
    public AdjustableHood(AdjustableHoodIO io) {
        super("Adjustable Hood");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Adjustable Hood", inputs);

        Logger.processInputs("Robot Position", inputs);
        this.robotPosition = state.getGlobalPoseEstimate().getTranslation();

        Logger.processInputs("Distance to Hub Center", inputs);
        this.hubDistance = this.robotPosition.getDistance(FieldConstants.Hub.centerHub);
    }

    /**
     * @param targetAngle gets the goal angle
     */
    public void setGoal() {
        Angle targetAngle;

        InterpolatingDoubleTreeMap hoodAngles = new InterpolatingDoubleTreeMap();
        hoodAngles.put(0.0, 0.0);
        hoodAngles.put();

        io.setTargetAngle(targetAngle);
    }

    public Command goToAngle(Angle angle) {
        return run(() -> this.setGoal(angle));
    }
}
