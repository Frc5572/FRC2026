package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.tunable.FlywheelConstants;

/**
 * Shooter Sim Implementation.
 *
 * <p>
 * Models the dual-Kraken flywheel as a real {@link FlywheelSim} plant driven by the same
 * Phoenix-style velocity controller (kS + kV*setpoint + kP*error, clamped to the duty-cycle voltage
 * limit) that runs on the robot. This reproduces both regimes seen in the Worlds logs: tracking an
 * achievable setpoint (e.g. ~49 rev/s), and saturating when commanded an unreachable setpoint
 * (180 rev/s drives the flywheel to its load-limited top speed of ~80 rev/s).
 * </p>
 *
 * <p>
 * The plant is identified by its feedforward gains {@link #PLANT_KV} / {@link #PLANT_KA} (in
 * V*s/rad and V*s^2/rad). These are the two knobs tuned against {@code /Shooter/ShooterAngular
 * Velocity1} via the kReplayCompare loop: PLANT_KV sets the saturated top speed (free speed at 12 V
 * = 12 / PLANT_KV), PLANT_KA sets the spin-up time constant.
 * </p>
 */
public class ShooterSim implements ShooterIO {

    /** Plant velocity feedforward, V*s/rad. 12 / PLANT_KV is the free speed at full voltage. */
    private static final double PLANT_KV = 0.021;
    /** Plant acceleration feedforward, V*s^2/rad. Larger = slower spin-up. */
    private static final double PLANT_KA = 0.005;
    /**
     * Quadratic aero/contact drag, modeled as a speed^2 voltage subtracted from the motor output
     * (V per (rad/s)^2). Negligible at the holding speeds but pulls the saturated top speed down to
     * the real ~477 rad/s envelope instead of the plant free speed.
     */
    private static final double PLANT_DRAG = 8.3e-6;
    /** Voltage limit from the real config (maxDutyCycle = 1.0 -> 12 V). */
    private static final double MAX_VOLTS = 12.0;
    /**
     * Fraction of flywheel speed lost each time a ball passes through. The Worlds logs show a ~6%
     * dip per ball; the exact timing is effectively random, so the goal is matching magnitude and
     * character, not the instant of each dip.
     */
    private static final double BALL_SPEED_DROP_FRAC = 0.06;

    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(PLANT_KV, PLANT_KA), DCMotor.getKrakenX60(2));

    // Velocity controller gains (rev/s based, matching Phoenix VelocityVoltage). Defaulted to the
    // real ShooterConstantsPID and refreshed from setConstants.
    private double kP = 0.9;
    private double kV = 0.13;
    private double kS = 0.04;

    /** Commanded flywheel setpoint, rev/s (when in velocity mode). */
    private double setpointRevPerSec = 0.0;
    /** Directly commanded voltage (when not in velocity mode, e.g. characterization). */
    private double directVolts = 0.0;
    private boolean velocityMode = false;
    private int numBallsShot = 0;

    /** Current flywheel speed in rev/s, for the fuel/shot model. */
    public double getSpeedRevPerSec() {
        return Units.radiansToRotations(flywheelSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        double measRevPerSec = getSpeedRevPerSec();
        double volts;
        if (velocityMode) {
            volts = kS * Math.signum(setpointRevPerSec) + kV * setpointRevPerSec
                + kP * (setpointRevPerSec - measRevPerSec);
        } else {
            volts = directVolts;
        }
        volts = MathUtil.clamp(volts, 0.0, MAX_VOLTS);
        // Quadratic aero/contact drag, always present, applied as a speed^2 voltage opposing motion.
        double w = flywheelSim.getAngularVelocityRadPerSec();
        double netVolts = volts - PLANT_DRAG * w * Math.abs(w);
        flywheelSim.setInputVoltage(netVolts);
        flywheelSim.update(TimedRobot.kDefaultPeriod);

        var speed = RotationsPerSecond.of(getSpeedRevPerSec());
        inputs.shooterAngularVelocity1 = speed;
        inputs.shooterAngularVelocity2 = speed;
        // SimCompare: predicted speed in the same unit as real /Shooter/ShooterAngularVelocity1
        // (rad/s), plus applied voltage and the commanded setpoint, for kReplayCompare overlay.
        Logger.recordOutput("SimCompare/Flywheel/predictedRadPerSec", speed.in(RadiansPerSecond));
        Logger.recordOutput("SimCompare/Flywheel/targetRPS", setpointRevPerSec);
        Logger.recordOutput("SimCompare/Flywheel/appliedVolts", volts);
    }

    @Override
    public void runVolts(double volts) {
        velocityMode = false;
        directVolts = volts;
    }

    @Override
    public void runVelocity(double velocity) {
        velocityMode = true;
        setpointRevPerSec = velocity;
    }

    /** Simulate shooting one ball: the flywheel loses ~6% of its speed feeding it. */
    public void shootOne() {
        double dropped =
            flywheelSim.getAngularVelocityRadPerSec() * (1.0 - BALL_SPEED_DROP_FRAC);
        flywheelSim.setState(VecBuilder.fill(Math.max(0.0, dropped)));
        numBallsShot++;
        Logger.recordOutput("FuelSim/BallsShot", numBallsShot);
    }

    @Override
    public void setConstants(FlywheelConstants constants) {
        this.kP = constants.pid.kP;
        this.kV = constants.pid.kV;
        this.kS = constants.pid.kS;
    }

}
