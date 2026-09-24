package frc.robot.subsystems.swerve.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.controls.ControlsConfig;
import frc.robot.controls.ControlsCurve;

/**
 * Control scheme utilities for teleoperated swerve driving.
 *
 * <p>
 * This class converts raw driver input signals (typically joystick axes) into {@link ChassisSpeeds}
 * suitable for commanding a swerve drivetrain. Every shaping parameter is read from a live
 * {@link ControlsConfig} rather than a compile-time constant, so drivers can tune the feel from
 * the web interface without a redeploy.
 *
 * <h2>Shaping</h2> Translation is treated as a single vector rather than two independent axes:
 *
 * <ul>
 * <li>A <em>radial</em> deadband is applied to the stick magnitude, then the remaining travel is
 * rescaled so that motion begins smoothly from zero.</li>
 * <li>The magnitude saturates at {@link ControlsConfig#translationSaturation()} of full
 * deflection, so a driver need not bottom the stick out to reach full speed.</li>
 * <li>The remaining travel is passed through the profile's {@link ControlsCurve}, which is
 * either a power curve or a monotone spline drawn in the tuner.</li>
 * <li>The result is clamped to the unit circle, so a diagonal commands the same speed as a
 * straight push. Scaling the axes independently, as this class previously did, made diagonals
 * about 41% faster than straight-ahead.</li>
 * </ul>
 *
 * <p>
 * When the translation stick returns to centre the commanded speed decays with the configured
 * time constant instead of stepping to zero. A time constant of zero restores the previous
 * behaviour of commanding an immediate stop.
 *
 * <p>
 * The produced {@link ChassisSpeeds} are expressed in a <em>pseudo-field-relative</em> frame,
 * where forward input is positive X, rightward input is positive Y, and counterclockwise input is
 * positive angular velocity. This method does not perform true field-relative conversion; callers
 * are expected to apply any required frame transformations using the robot's current heading.
 */
@NullMarked
public class TeleopControls {

    private TeleopControls() {}

    /**
     * Creates a supplier that converts driver inputs into desired chassis speeds.
     *
     * @param forward supplier providing the forward/backward driver input
     * @param right supplier providing the left/right driver input
     * @param turnCcw supplier providing the counterclockwise rotation input
     * @param config supplier providing the control configuration currently in force
     * @param maxSpeed supplier providing the translational speed cap, in meters per second
     * @param maxRotSpeed supplier providing the rotational speed cap, in radians per second
     * @return a supplier that generates processed {@link ChassisSpeeds} for teleop control
     */
    public static Supplier<ChassisSpeeds> teleopControls(DoubleSupplier forward,
        DoubleSupplier right, DoubleSupplier turnCcw, Supplier<ControlsConfig> config,
        DoubleSupplier maxSpeed, DoubleSupplier maxRotSpeed) {
        return new Supplier<ChassisSpeeds>() {

            private double lastVx = 0.0;
            private double lastVy = 0.0;
            private double lastTimestamp = Timer.getFPGATimestamp();

            @Override
            public ChassisSpeeds get() {
                ControlsConfig cfg = config.get();
                double now = Timer.getFPGATimestamp();
                double dt = Math.max(0.0, Math.min(0.2, now - lastTimestamp));
                lastTimestamp = now;

                double yaxis = forward.getAsDouble();
                double xaxis = right.getAsDouble();
                double scale = shapeMagnitude(Math.hypot(xaxis, yaxis), cfg.translationDeadband(),
                    cfg.translationSaturation(), cfg.translationCurve(),
                    cfg.translationExponent());

                double vx;
                double vy;
                if (scale > 0.0) {
                    double magnitude = Math.hypot(xaxis, yaxis);
                    double speed = scale * maxSpeed.getAsDouble();
                    vx = yaxis / magnitude * speed;
                    vy = xaxis / magnitude * speed;
                } else {
                    double decay = coastFactor(cfg.releaseTimeConstant(), dt);
                    vx = lastVx * decay;
                    vy = lastVy * decay;
                }
                lastVx = vx;
                lastVy = vy;

                double raxis = turnCcw.getAsDouble();
                double turn = shapeMagnitude(Math.abs(raxis), cfg.rotationDeadband(), 1.0,
                    cfg.rotationCurve(), cfg.rotationExponent()) * Math.signum(raxis);
                return new ChassisSpeeds(vx, vy, turn * maxRotSpeed.getAsDouble());
            }
        };
    }

    /**
     * Apply a deadband, saturation point and response curve to a non-negative stick magnitude.
     *
     * @param magnitude raw stick magnitude, normally in [0, 1]
     * @param deadband fraction of travel ignored around centre
     * @param saturation fraction of travel at which full output is reached
     * @param curve the response curve applied to the rescaled travel
     * @param exponent exponent used when the curve is in power mode
     * @return the shaped magnitude, in [0, 1]
     */
    public static double shapeMagnitude(double magnitude, double deadband, double saturation,
        ControlsCurve curve, double exponent) {
        double top = Math.max(saturation, deadband + 1e-6);
        if (magnitude <= deadband) {
            return 0.0;
        }
        double normalized = Math.min(1.0, (magnitude - deadband) / (top - deadband));
        return curve.evaluate(normalized, exponent);
    }

    /**
     * The per-loop decay applied to a released translation command.
     *
     * @param timeConstant coast time constant in seconds; zero stops immediately
     * @param dt elapsed time since the previous evaluation, in seconds
     * @return a multiplier in [0, 1]
     */
    public static double coastFactor(double timeConstant, double dt) {
        if (timeConstant <= 0.0) {
            return 0.0;
        }
        return Math.exp(-dt / timeConstant);
    }

}
