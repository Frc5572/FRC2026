package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.GenerateEmptyIO;

@GenerateEmptyIO
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        Distance hopperPosition = Distance.ofBaseUnits(0, Meters);
        double intakeDutyCycle = 0;
        boolean limitSwitch = false;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void runIntakeMotor(double speed);

    public void setEncoderPosition(double position);

    public void runHopperMotor(double setPoint);


}
