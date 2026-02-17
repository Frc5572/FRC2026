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
        public double leftHopperPositionRotations = 0;
        public double rightHopperPositionRotations = 0;

        public Distance leftHopperPosition = Meters.of(leftHopperPositionRotations);

        public double intakeDutyCycle = 0;
        public boolean limitSwitch = false;
        public boolean intakeMotorConnected = false;
    }

    public void updateInputs(IntakeInputs inputs);

    public void runIntakeMotor(double speed);

    public void setEncoderPosition(double position);

    public void setLeftHopperVoltage(double setPoint);

    public void setRightHopperVoltage(double setPoint);
}
