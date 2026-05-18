package frc.robot.util.tunable;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Utility tunable for testing out different shot parameters. */
public class ShotDataHelper implements Tunable {

    /** Hood angle in degrees */
    public double hoodAngle = 5.0;
    /** Flywheel speed in rotations per second */
    public double flywheelSpeed = 60.0;
    /** Distance from target in feet */
    public double distanceFromTarget = 10.0;

    private final DoublePublisher timeLastGenerated;

    /** Create new helper. */
    public ShotDataHelper() {
        var nt = NetworkTableInstance.getDefault();
        timeLastGenerated = nt.getDoubleTopic("/ShotDataHelper/TimeLastGenerated").publish();
        Tunable.setupTunable("/ShotDataHelper", this, ShotDataHelper.class, () -> {
        });
    }

}
