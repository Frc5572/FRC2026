package frc.robot.subsystems.vision.colorDetection;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Color extends SubsystemBase {
    private ColorIO io;
    private ColorInputsAutoLogged inputs = new ColorInputsAutoLogged();

    public void color(ColorIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Color/", inputs);
    }

    public Angle getYaw() {
        return inputs.yaw;
    }
}
