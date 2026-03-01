package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.sim.SimPosition;
import frc.robot.util.tunable.PIDConstants;

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

    private final SimPosition arm = new SimPosition(0.8, 0.4, 2.0);
    private final SimPosition hooks = new SimPosition(1.0, 0.2, 3.0);

    private double armTarget = Constants.Climber.Pivot.startingAngle.in(Radians);
    private double hookTarget = 0.5;

    public ClimberSim() {
        arm.position = armTarget;
        hooks.position = hookTarget;
    }

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
    public void updateInputs(ClimberInputs inputs) {
        arm.update(armTarget);
        hooks.update(hookTarget);

        inputs.positionPivot = Radians.of(arm.position);
        inputs.positionTelescope = Meters.of(hooks.position);
    }

    /**
     * Sets the voltage for the telescope motor.
     *
     * @param volts the voltage to apply to the telescope motor, in volts
     */
    @Override
    public void setVoltageTelescope(double volts) {}

    /**
     * Sets the voltage for the pivot motor.
     *
     * @param volts the voltage to apply to the pivot motor, in volts
     */
    @Override
    public void setVoltagePivot(double volts) {}

    /**
     * Sets the target angle for the pivot mechanism.
     *
     * @param angle the desired pivot angle
     */
    @Override
    public void setAnglePivot(Angle angle) {
        armTarget = angle.in(Radians);
    }


    /**
     * Sets the height for the telescope mechanism
     *
     * @param height the desired height
     */
    @Override
    public void setHeightTelescope(Distance height) {
        hookTarget = height.in(Meters);
    }

    @Override
    public void setPIDPivot(PIDConstants constants) {}

    @Override
    public void setPIDTelescope(PIDConstants constants) {}
}
