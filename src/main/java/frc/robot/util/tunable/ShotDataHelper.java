package frc.robot.util.tunable;

public class ShotDataHelper implements Tunable {

    /** Hood angle in degrees */
    public double hoodAngle;
    /** Flywheel speed in rotations per second */
    public double flywheelSpeed;
    /** Distance from target in feet */
    public double distanceFromTarget;

    public ShotDataHelper() {
        Tunable.setupTunable("/ShotDataHelper", this, ShotDataHelper.class, () -> {
        });
    }

}
