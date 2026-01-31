package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.GenerateEmptyIO;

@GenerateEmptyIO
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        Distance hopperPositionMeters;
        double intakeVelocity;
        boolean limitSwitch;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void runIntakeMotor(double speed);

    public void setEncoderPosition(double position);

    public void runHopperMotor(double setPoint);


}
