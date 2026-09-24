package frc.robot.subsystems.swerve.util;

import org.ejml.data.DMatrix3;
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.jspecify.annotations.NullMarked;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.controls.ControlsConfig;
import frc.robot.controls.ControlsField;

/**
 * Applies rate limiting to robot-relative swerve chassis commands to ensure physically achievable,
 * stable, and predictable motion.
 *
 * <p>
 * This limiter constrains translational acceleration using a multi-stage process inspired by Team
 * 1690's swerve control methodology:
 * <ul>
 * <li>Limits forward acceleration based on current speed and remaining headroom to maximum
 * velocity</li>
 * <li>Clamps directional acceleration to prevent excessive robot tilt</li>
 * <li>Caps overall acceleration magnitude to avoid wheel slip and skidding</li>
 * </ul>
 *
 * <p>
 * The limiter operates in discrete time (20&nbsp;ms control loop) and should be called once per
 * cycle. The current robot velocity must be provided via {@link #update(ChassisSpeeds)} before
 * calling {@link #limit(ChassisSpeeds)}.
 *
 * <p>
 * All limits are read from the live {@link ControlsConfig} each cycle, so they can be tuned from
 * the driver web interface. A limit at or above {@link ControlsField#LIMIT_DISABLED} disables that
 * stage. Diagnostic values are published to AdvantageKit logs for visualization and debugging.
 *
 * <p>
 * Rotational velocity ({@code omegaRadiansPerSecond}) is passed through unchanged and is not
 * subject to rate limiting.
 *
 * <p>
 * <b>Usage pattern:</b>
 *
 * <pre>{@code
 * limiter.update(currentRobotRelativeSpeeds);
 * ChassisSpeeds safeSpeeds = limiter.limit(desiredSpeeds);
 * }</pre>
 *
 * <p>
 * Based on concepts presented by FRC Team 1690:
 * <a href="https://www.youtube.com/watch?v=vUtVXz7ebEE">
 * https://www.youtube.com/watch?v=vUtVXz7ebEE</a>
 */
@NullMarked
public class SwerveRateLimiter {

    private final Supplier<ControlsConfig> config;

    /**
     * Creates a new {@code SwerveRateLimiter} reading its limits from the supplied configuration.
     *
     * @param config supplier of the control configuration currently in force
     */
    public SwerveRateLimiter(Supplier<ControlsConfig> config) {
        this.config = config;
    }

    /**
     * Updates the current robot-relative chassis velocity used as the basis for acceleration
     * limiting.
     *
     * <p>
     * This method should be called once per control loop using velocity data from odometry or state
     * estimation before calling {@link #limit(ChassisSpeeds)}.
     *
     * @param robotRelative the current robot-relative chassis speeds
     */
    public void update(ChassisSpeeds robotRelative) {
        currentVel.a1 = robotRelative.vxMetersPerSecond;
        currentVel.a2 = robotRelative.vyMetersPerSecond;
        currentVel.a3 = robotRelative.omegaRadiansPerSecond;
    }

    private final DMatrix3 currentVel = new DMatrix3();
    private final DMatrix3 wantedVel = new DMatrix3();
    private final DMatrix3 wantedAcc = new DMatrix3();

    private void publish(String name, DMatrix3 v) {
        Logger.recordOutput("SwerveRateLimiter/" + name, new Twist2d(v.a1, v.a2, v.a3));
    }

    private void publish(String name, double v) {
        Logger.recordOutput("SwerveRateLimiter/" + name, v);
    }

    /**
     * Limits a desired robot-relative chassis velocity based on physical acceleration, tilt, and
     * skid constraints, producing a safe and achievable {@link ChassisSpeeds}.
     *
     * <p>
     * The limiting process operates in discrete time (20 ms) and applies several sequential
     * constraints:
     * <ol>
     * <li><b>Physical acceleration limit:</b> Caps acceleration in the direction of current motion
     * based on available traction and remaining headroom to max speed.</li>
     * <li><b>Tilt prevention:</b> Restricts directional acceleration components to prevent
     * excessive forward, backward, or lateral tilting.</li>
     * <li><b>Skid prevention:</b> Limits the overall translational acceleration magnitude to avoid
     * wheel slip.</li>
     * </ol>
     *
     * <p>
     * The resulting acceleration is integrated over one control loop period and added to the
     * current velocity to produce the final commanded chassis speeds.
     *
     * <p>
     * Rotational velocity ({@code omegaRadiansPerSecond}) is passed through unchanged and is not
     * subject to acceleration limiting.
     *
     * @param wantedSpeedsRobotRelative the desired robot-relative chassis speeds (vx, vy, omega)
     * @return a new {@link ChassisSpeeds} representing the limited, physically achievable
     *         robot-relative velocities for the next control step
     */
    public ChassisSpeeds limit(ChassisSpeeds wantedSpeedsRobotRelative) {
        ControlsConfig cfg = config.get();
        double forwardLimit = cfg.forwardAccelLimit();
        double forwardTiltLimit = cfg.forwardTiltLimit();
        double leftTiltLimit = cfg.leftTiltLimit();
        double rightTiltLimit = cfg.rightTiltLimit();
        double backTiltLimit = cfg.backTiltLimit();
        double skidLimit = cfg.skidLimit();
        double currentSpeed = Math.hypot(currentVel.a1, currentVel.a2);
        double wantedSpeed = Math.hypot(wantedSpeedsRobotRelative.vxMetersPerSecond,
            wantedSpeedsRobotRelative.vyMetersPerSecond);

        publish("currentSpeed", currentSpeed);
        publish("wantedSpeed", wantedSpeed);

        wantedVel.a1 = wantedSpeedsRobotRelative.vxMetersPerSecond;
        wantedVel.a2 = wantedSpeedsRobotRelative.vyMetersPerSecond;
        wantedVel.a3 = wantedSpeedsRobotRelative.omegaRadiansPerSecond;
        publish("wantedVel", wantedVel);
        publish("currentVel", currentVel);

        CommonOps_DDF3.subtract(wantedVel, currentVel, wantedAcc);
        CommonOps_DDF3.divide(wantedAcc, 0.02);
        publish("wantedAccStep0", wantedAcc);

        // Step 1: Robot cannot accelerate indefinitely. Ensure accelerations are achievable
        // (sub-max to ensure correctness even under degradation).
        double subphysicalAccelerationLimit =
            Math.max(1.0 - (currentSpeed / Constants.Swerve.maxSpeed), 0.0);
        publish("subphysicalAccelerationLimit", subphysicalAccelerationLimit);
        double maxForwardAccel = forwardLimit * subphysicalAccelerationLimit;
        publish("maxForwardAccel", maxForwardAccel);

        // Acceleration in the direction of current velocity. At a standstill that direction is
        // undefined; dividing by zero here used to yield NaN, and because every NaN comparison is
        // false the limit below was skipped for exactly the launch it was meant to shape. From
        // rest any acceleration is "forward", so use its magnitude instead.
        double wantedAccMagnitude;
        if (currentSpeed > 1e-6) {
            wantedAccMagnitude = (wantedAcc.a1 * currentVel.a1 + wantedAcc.a2 * currentVel.a2)
                / currentSpeed;
        } else {
            wantedAccMagnitude = Math.hypot(wantedAcc.a1, wantedAcc.a2);
        }
        publish("wantedAccMagnitudeStep1", wantedAccMagnitude);
        if (forwardLimit < ControlsField.LIMIT_DISABLED && wantedAccMagnitude > maxForwardAccel) {
            double mul = maxForwardAccel / wantedAccMagnitude;
            wantedAcc.a1 *= mul;
            wantedAcc.a2 *= mul;
        }
        publish("wantedAccStep1", wantedAcc);

        // Step 2: Robot may accelerate too fast and result in tilting. Limit directional
        // acceleration to prevent this.
        if (forwardTiltLimit < ControlsField.LIMIT_DISABLED && wantedAcc.a1 > forwardTiltLimit) {
            wantedAcc.a1 = forwardTiltLimit;
        }
        if (backTiltLimit < ControlsField.LIMIT_DISABLED && wantedAcc.a1 < -backTiltLimit) {
            wantedAcc.a1 = -backTiltLimit;
        }
        if (leftTiltLimit < ControlsField.LIMIT_DISABLED && wantedAcc.a2 > leftTiltLimit) {
            wantedAcc.a2 = leftTiltLimit;
        }
        if (rightTiltLimit < ControlsField.LIMIT_DISABLED && wantedAcc.a2 < -rightTiltLimit) {
            wantedAcc.a2 = -rightTiltLimit;
        }
        publish("wantedAccStep2", wantedAcc);

        // Step 3: Robot accelerating in a different direction may result in skidding. Limit
        // magnitude of acceleration to prevent this.
        wantedAccMagnitude = Math.hypot(wantedAcc.a1, wantedAcc.a2);
        publish("wantedAccMagnitudeStep3", wantedAccMagnitude);
        if (skidLimit < ControlsField.LIMIT_DISABLED && wantedAccMagnitude > skidLimit) {
            double multiplier = skidLimit / wantedAccMagnitude;
            wantedAcc.a1 *= multiplier;
            wantedAcc.a2 *= multiplier;
        }
        publish("wantedAccStep3", wantedAcc);

        CommonOps_DDF3.scale(0.02, wantedAcc);
        CommonOps_DDF3.add(currentVel, wantedAcc, wantedVel);

        publish("resultant", wantedVel);
        publish("resultantSpeed", Math.hypot(wantedVel.a1, wantedVel.a2));

        return new ChassisSpeeds(wantedVel.a1, wantedVel.a2, wantedVel.a3);
    }

}
