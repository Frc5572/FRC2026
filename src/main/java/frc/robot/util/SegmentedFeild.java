package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.math.Rectangle;

public class SegmentedFeild {
    final public Rectangle blueAlliance =
        new Rectangle("Blue Alliacne", new Pose2d(2, 4.05, Rotation2d.kZero), 4, 8.1);
    final public Rectangle blueAllianceClimber = new Rectangle("Blue Alliacne Climber",
        new Pose2d(0.6281201839447021, 3.75, Rotation2d.kZero), 1.2, 1.1);
    final public Rectangle blueDropper =
        new Rectangle("Blue Dropper", new Pose2d(5.7, 4.05, Rotation2d.kZero), 1.0, 1.7);
    final public Rectangle neutralZone =
        new Rectangle("Netural Zone", new Pose2d(8.25, 4.05, Rotation2d.kZero), 6.1, 8.1);

}
