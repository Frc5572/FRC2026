package frc.robot.subsystems.adjustableHood;

import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AdjustableHood extends SubsystemBase {

    private boolean atAngle = false;
    private double targetHoodRotation = 0.0;
    private final AdjustableHoodIO io;
    private final AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();

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

        atAngle = Math
            .abs(inputs.hoodLocation - targetHoodRotation) < Constants.AdjustableHood.hoodTolerence;
    }

    public void setGoal(Angle targetAngle) {
        targetHoodRotation = Math.max(Constants.AdjustableHood.minAngle.in(Rotations),
            Math.min(targetAngle.in(Rotations), Constants.AdjustableHood.maxAngle.in(Rotations)));
        io.setTargetAngle(Rotations.of(targetHoodRotation));
    }

    public Command goToAngle(Angle angle) {
        return run(() -> this.setGoal(angle)).until(() -> atAngle);
    }


}
