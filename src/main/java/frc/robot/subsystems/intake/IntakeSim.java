package frc.robot.subsystems.intake;

import frc.robot.sim.SimPosition;

/**
 * sim class
 */
public class IntakeSim implements IntakeIO {

    public boolean isIntaking = false;
    private double targetPosition = 0.0;
    private double targetVoltage = 0.0;

    private final SimPosition hopper = new SimPosition(0.7, 2.0, 60.0);

    @Override
    public void updateInputs(IntakeInputs inputs) {
        hopper.update(targetPosition);
        inputs.leftHopperPositionRotations = (hopper.position);
        inputs.rightHopperPositionRotations = hopper.position;
    }

    @Override
    public void runIntakeMotor(double speed) {
        isIntaking = speed > 0.5;
    }

    @Override
    public void setEncoderPosition(double position) {}

    @Override
    public void setLeftHopperVoltage(double volts) {
        targetVoltage = (volts);
    }

    @Override
    public void setRightHopperVoltage(double volts) {
        targetVoltage = volts;
    }

    @Override
    public void setLeftHopperPosition(double rotations) {
        targetPosition = rotations;
    }

    @Override
    public void setRightHopperPosition(double rotations) {
        targetPosition = rotations;
    }
}
