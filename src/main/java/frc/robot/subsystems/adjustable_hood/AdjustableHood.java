package frc.robot.subsystems.adjustable_hood;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Adjustable Hood Subsystem
 */
public class AdjustableHood extends SubsystemBase {

    private final AdjustableHoodIO io;
    public final AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();

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
    }

    public Distance hubDistance() {

    }

    public Angle goalAngle(){
        InterpolatingDoubleTreeMap hoodAngles = new InterpolatingDoubleTreeMap();
        hoodAngles.put()
    }

    /**
     * @param targetAngle gets the goal angle
     */
    public void setGoal(Angle targetAngle) {
        io.setTargetAngle(targetAngle);
    }

    public Command goToAngle(Angle angle) {
        return run(() -> this.setGoal(angle));
    }
}
