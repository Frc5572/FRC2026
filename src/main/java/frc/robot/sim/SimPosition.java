package frc.robot.sim;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Simple position and velocity simulator for mechanism motion.
 *
 * <p>
 * This class models a 1D mechanism using a proportional position controller with configurable
 * velocity and acceleration limits. It is intended for lightweight simulation of mechanism motion
 * where full physics modeling is unnecessary.
 * </p>
 */
public class SimPosition {

    /** Proportional gain applied to position error. */
    private final double kp;

    /** Maximum allowed velocity (units per second). */
    private final double maxVelocity;

    /** Maximum allowed acceleration (units per second squared). */
    private final double maxAccel;

    /** Current simulated position. */
    public double position = 0.0;

    /** Current simulated velocity. */
    public double velocity = 0.0;

    /**
     * Creates a new position simulator.
     *
     * @param kp proportional gain applied to position error
     * @param maxVelocity maximum allowed velocity
     * @param maxAccel maximum allowed acceleration
     */
    public SimPosition(double kp, double maxVelocity, double maxAccel) {
        this.kp = kp;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
    }

    /** Advances the simulation one timestep toward the given target position. */
    public void update(double target) {
        double err = target - this.position;
        double next = this.position + err * this.kp;
        double velocity = (next - this.position) / TimedRobot.kDefaultPeriod;
        if (Math.abs(velocity) > maxVelocity) {
            velocity = Math.copySign(maxVelocity, velocity);
        }
        double accel = (velocity - this.velocity) / TimedRobot.kDefaultPeriod;
        if (Math.abs(accel) > maxAccel) {
            accel = Math.copySign(maxAccel, accel);
            velocity = accel * TimedRobot.kDefaultPeriod + this.velocity;
        }
        next = velocity * TimedRobot.kDefaultPeriod + this.position;
        this.position = next;
        this.velocity = velocity;
    }

}
