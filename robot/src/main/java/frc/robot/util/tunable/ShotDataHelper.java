package frc.robot.util.tunable;

import frc.robot.util.Tunable;

/** Utility tunable for testing out different shot parameters. */
@Tunable
public class ShotDataHelper {

    /** Hood angle in degrees */
    public double hoodAngle = 5.0;
    /** Flywheel speed in rotations per second */
    public double flywheelSpeed = 60.0;
    /** Distance from target in feet */
    public double distanceFromTarget = 10.0;

    /** Create new helper. */
    public ShotDataHelper() {

    }

}
