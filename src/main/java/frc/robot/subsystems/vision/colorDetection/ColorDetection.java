package frc.robot.subsystems.vision.colorDetection;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorDetection extends SubsystemBase {
    private ColorDetectionIO io;
    private ColorInputsAutoLogged inputs = new ColorInputsAutoLogged();
    private double lastSeenYellow;

    public ColorDetection(ColorDetectionIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Color/", inputs);
        if (inputs.seesYellow) {
            lastSeenYellow = Timer.getFPGATimestamp();
        }
    }

    public double lastSeenYellow() {
        return lastSeenYellow;
    }

    public Angle getYaw() {
        return inputs.yaw;
    }
}
