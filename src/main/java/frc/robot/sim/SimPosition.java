package frc.robot.sim;

import edu.wpi.first.wpilibj.TimedRobot;

public class SimPosition {

    private final double kp;
    private final double maxVelocity;
    private final double maxAccel;

    public double position = 0.0;
    public double velocity = 0.0;

    public SimPosition(double kp, double maxVelocity, double maxAccel) {
        this.kp = kp;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
    }

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
