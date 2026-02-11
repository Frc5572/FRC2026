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
        public Distance hopperPosition = Meters.of(0);
        public double intakeDutyCycle = 0;
        public boolean limitSwitch = false;

        public boolean intakeMotorConnected = false;
    }

    public void updateInputs(IntakeInputs inputs);

    public void runIntakeMotor(double speed);

    public void setEncoderPosition(double position);

    public void runHopperMotor(double setPoint);


}
