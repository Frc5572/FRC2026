package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import java.util.Random;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.tunable.PIDConstants;

/**
 * Simulation implementation of {@link TurretIO}.
 *
 * <p>
 * This class provides a lightweight software model of the turret hardware for use in simulation,
 * unit testing, and log replay. It maintains an internal turret angle state and generates synthetic
 * absolute encoder readings based on the configured gear ratios and offsets.
 * </p>
 *
 * <p>
 * To better approximate real sensor behavior, small amounts of random noise are added to each
 * simulated encoder measurement. This helps exercise the turret angle estimation and filtering
 * logic under non-ideal conditions.
 * </p>
 *
 * <p>
 * This simulation does not model turret dynamics such as inertia, acceleration limits, or
 * closed-loop control behavior. Target angles are applied instantaneously.
 * </p>
 */
public class TurretSim implements TurretIO {

    private final Random random;

    private static final double start = 0.0;
    public double turretTarget = start;

    public TurretSim(Random random) {
        this.random = random;
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.relativeAngle = Units.radiansToRotations(turretTarget - start);
        inputs.velocity = RadiansPerSecond.of(0.0);
    }

    @Override
    public void setTurretVoltage(Voltage volts) {}


    @Override
    public void setTargetAngle(Angle angle, AngularVelocity velocity) {
        turretTarget = angle.in(Radians);
    }

    @Override
    public void resetPosition(Angle angle) {}

    @Override
    public void setPID(PIDConstants constants) {}
}
