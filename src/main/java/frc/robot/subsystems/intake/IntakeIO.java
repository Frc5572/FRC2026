package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.GenerateEmptyIO;

/**
 * intake IO
 */
@GenerateEmptyIO
public interface IntakeIO {
    /**
     * inputs class
     */
    @AutoLog
    public static class IntakeInputs {
        Distance hopperPosition = Meters.of(0);
        double intakeDutyCycle = 0;
        boolean limitSwitch = false;
    }

    public void updateInputs(IntakeInputs inputs);

    public void runIntakeMotor(double speed);

    public void setEncoderPosition(double position);

    public void runHopperMotor(double setPoint);


}
