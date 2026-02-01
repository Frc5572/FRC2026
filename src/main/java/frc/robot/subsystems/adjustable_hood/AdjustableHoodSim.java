package frc.robot.subsystems.adjustable_hood;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.SimPosition;

/**
 * Simulation implementation of {@link AdjustableHoodIO}.
 */
public class AdjustableHoodSim implements AdjustableHoodIO {

    private final SimPosition hood = new SimPosition(0.8, 4.0, 60.0);
    private double hoodTarget = 0.0;

    @Override
    public void updateInputs(AdjustableHoodInputs inputs) {
        hood.update(hoodTarget);
        inputs.relativeAngle = Radians.of(hood.position);
    }

    @Override
    public void setAdjustableHoodVoltage(Voltage volts) {

    }

    @Override
    public void setTargetAngle(Angle angle) {
        hoodTarget = angle.in(Radians);
    }

}
