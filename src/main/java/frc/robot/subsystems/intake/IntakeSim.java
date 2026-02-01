package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import frc.robot.sim.SimPosition;

/**
 * sim class
 */
public class IntakeSim implements IntakeIO {

    public boolean isIntaking = false;
    private double targetPosition = 0.0;

    private final SimPosition hopper = new SimPosition(0.7, 2.0, 60.0);

    @Override
    public void updateInputs(IntakeInputs inputs) {
        hopper.update(targetPosition);
        inputs.hopperPosition = Meters.of(hopper.position);
    }

    @Override
    public void runIntakeMotor(double speed) {
        isIntaking = speed > 0.5;
    }

    @Override
    public void setEncoderPosition(double position) {}

    @Override
    public void runHopperMotor(double setPoint) {
        targetPosition = setPoint;
    }

}
