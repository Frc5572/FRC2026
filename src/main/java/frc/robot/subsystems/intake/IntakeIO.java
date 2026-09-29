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
        public Distance rightHopperPosition = Meters.of(rightHopperPositionRotations);

        /** Voltage the hopper motors are actually applying. */
        public double leftHopperAppliedVolts = 0;
        public double rightHopperAppliedVolts = 0;
        public double leftHopperStatorCurrent = 0;
        public double rightHopperStatorCurrent = 0;
        /** Raw fault bitfields; nonzero means the motor may be refusing to drive. */
        public int leftHopperFaults = 0;
        public int rightHopperFaults = 0;
        public boolean leftHopperConnected = false;
        public boolean rightHopperConnected = false;
        /** True on the tick a hopper motor controller is seen to have rebooted. */
        public boolean leftHopperReset = false;
        public boolean rightHopperReset = false;

        public double intakeDutyCycle = 0;
        public boolean limitSwitch = false;
        public boolean intakeMotorConnected = false;
    }

    public void updateInputs(IntakeInputs inputs);

    public void runIntakeMotor(double speed);

    public void setLeftHopperVoltage(double setPoint);

    public void setRightHopperVoltage(double setPoint);
}
