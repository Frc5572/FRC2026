package frc.robot.subsystems.adjustableHood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AdjustableHood extends SubsystemBase {

    private final AdjustableHood io;
    private final AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();

    public AdjustableHood(AdjustableHoodIO io) {
        super("Adjustable Hood");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Adjustable Hood", inputs);
    }

    public void setGoal(Angle targetAngle) {
        double clamped = Math.max(Constants.AdjustableHood.minAngle.in(Rotations),
            Math.min(targetAngle.in(Rotations), Constants.AdjustableHood.maxAngle.in(Rotations)));
        io.setTargetAngle(Degrees.of(clamped));
    }

    public Command goToAngle(Angle angle) {
        return run(() -> this.setGoal(angle));
    }


}
