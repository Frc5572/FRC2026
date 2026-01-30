package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;

/**
 * Simulation implementation of the climber subsystem.
 *
 * <p>
 * This class provides a mock implementation of {@link ClimberIO} for testing and simulation
 * purposes. It allows developers to test climber control logic without physical hardware.
 *
 * @see ClimberIO
 */
public class ClimberSim implements ClimberIO {

    /**
     * Updates the climber input values for simulation.
     *
     * <p>
     * In simulation, this method can be used to update simulated sensor readings and other input
     * states.
     *
     * @param inputs the input container to update with current climber state
     */
    @Override
    public void updateInputs(ClimberInputs inputs) {}

    /**
     * Sets the voltage for the telescope motor.
     *
     * @param volts the voltage to apply to the telescope motor, in volts
     */
    @Override
    public void setVoltageTelescope(double volts) {}

    /**
     * Sets the power for the telescope motor.
     *
     * @param power the power output for the telescope motor, typically in the range [-1.0, 1.0]
     */
    @Override
    public void setPowerTelescope(double power) {}

    /**
     * Sets the voltage for the pivot motor.
     *
     * @param volts the voltage to apply to the pivot motor, in volts
     */
    @Override
    public void setVoltagePivot(double volts) {}

    /**
     * Sets the power for the pivot motor.
     *
     * @param power the power output for the pivot motor, typically in the range [-1.0, 1.0]
     */
    @Override
    public void setPowerPivot(double power) {}

    /**
     * Sets the target angle for the pivot mechanism.
     *
     * @param angle the desired pivot angle
     */
    @Override
    public void setAnglePivot(Angle angle) {}
}
