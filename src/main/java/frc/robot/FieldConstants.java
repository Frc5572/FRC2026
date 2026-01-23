package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.math.Rectangle;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

    public static final Distance fieldLength = Inches.of(651.2);
    public static final Distance fieldWidth = Inches.of(317.7);
    /** Measured from the inside of starting line */
    public static final Distance startingLineX = Inches.of(299.438);

    static final public Rectangle blueAlliance =
        new Rectangle("Blue Alliacne", new Pose2d(2, 4.05, Rotation2d.kZero), 4.0, 8.1);
    static final public Rectangle blueAllianceClimber = new Rectangle("Blue Alliacne Climber",
        new Pose2d(0.6281201839447021, 3.75, Rotation2d.kZero), 1.2, 1.1);
    static final public Rectangle blueDropper =
        new Rectangle("Blue Dropper", new Pose2d(5.7, 4.05, Rotation2d.kZero), 1.0, 1.7);
    static final public Rectangle neutralZone =
        new Rectangle("Neutral Zone", new Pose2d(8.25, 4.05, Rotation2d.kZero), 6.1, 8.1);
    static final public Rectangle redDropper =
        new Rectangle("Red Dropper", new Pose2d(10.8, 4.05, Rotation2d.kZero), 1.0, 1.7);
    public final static Rectangle redAlliance =
        new Rectangle("Red Alliacne", new Pose2d(14.5, 4.05, Rotation2d.kZero), 4, 8.1);
    static final public Rectangle redAllianceClimber =
        new Rectangle("Red Alliacne Climber", new Pose2d(15.87, 3.75, Rotation2d.kZero), 1.2, 1.1);
}
