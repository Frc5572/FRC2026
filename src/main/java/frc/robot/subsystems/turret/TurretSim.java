package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import java.util.Random;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.sim.SimPosition;

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

    public final SimPosition turrentAngle = new SimPosition(0.8, 4.0, 60.0);
    private double turretTarget = 0.0;

    public TurretSim(Random random) {
        this.random = random;
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        turrentAngle.update(turretTarget);

        inputs.relativeAngle = Radians.of(turrentAngle.position);
        inputs.velocity = RadiansPerSecond.of(turrentAngle.velocity);

        double noise1 = (random.nextDouble() - 0.5) * 2.0 * 0.2;
        inputs.gear1AbsoluteAngle =
            Turret.getGearAnglesFromTurret(inputs.relativeAngle, Constants.Turret.gear1Gearing,
                Constants.Turret.gear1Offset).plus(Rotation2d.fromDegrees(noise1));
        double noise2 = (random.nextDouble() - 0.5) * 2.0 * 0.2;
        inputs.gear2AbsoluteAngle =
            Turret.getGearAnglesFromTurret(inputs.relativeAngle, Constants.Turret.gear2Gearing,
                Constants.Turret.gear2Offset).plus(Rotation2d.fromDegrees(noise2));
    }

    @Override
    public void setTurretVoltage(Voltage volts) {}


    @Override
    public void setTargetAngle(Angle angle) {
        turretTarget = angle.in(Radians);
    }

    @Override
    public void resetPosition(Angle angle) {}

}
