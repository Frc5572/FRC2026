package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import frc.robot.Constants;

/**
 * sim class
 */
public class IntakeSim implements IntakeIO {

    public boolean isIntaking = false;
    private double position = 0;

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.leftHopperPositionRotations = position;
        inputs.rightHopperPositionRotations = position;
        inputs.leftHopperPosition = Meters.of(position);
    }

    @Override
    public void runIntakeMotor(double speed) {
        isIntaking = speed > 0.5;
    }

    @Override
    public void setLeftHopperVoltage(double volts) {
        if (volts > 0.1) {
            this.position = Constants.IntakeConstants.hopperOutDistance.in(Meters);
        } else if (volts < -0.1) {
            this.position = 0;
        }
    }

    @Override
    public void setRightHopperVoltage(double volts) {
        if (volts > 0.1) {
            this.position = Constants.IntakeConstants.hopperOutDistance.in(Meters);
        } else if (volts < -0.1) {
            this.position = 0;
        }
    }
}
