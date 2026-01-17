package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.math.Penetration;
import frc.robot.math.Rectangle;

public class SegmentedFeild {
    final public Rectangle blueAlliance =
        new Rectangle("Blue Alliacne", new Pose2d(2, 4.05, Rotation2d.kZero), 4.0, 8.1);
    final public Rectangle blueAllianceClimber = new Rectangle("Blue Alliacne Climber",
        new Pose2d(0.6281201839447021, 3.75, Rotation2d.kZero), 1.2, 1.1);
    final public Rectangle blueDropper =
        new Rectangle("Blue Dropper", new Pose2d(5.7, 4.05, Rotation2d.kZero), 1.0, 1.7);
    final public Rectangle neutralZone =
        new Rectangle("Netural Zone", new Pose2d(8.25, 4.05, Rotation2d.kZero), 6.1, 8.1);
    final public Rectangle redDropper =
        new Rectangle("Red Dropper", new Pose2d(10.8, 4.05, Rotation2d.kZero), 1.0, 1.7);
    final public Rectangle redAlliance =
        new Rectangle("Red Alliacne", new Pose2d(14.5, 4.05, Rotation2d.kZero), 4, 8.1);
    final public Rectangle redAllianceClimber =
        new Rectangle("Red Alliacne Climber", new Pose2d(15.87, 3.75, Rotation2d.kZero), 1.2, 1.1);

    private Penetration penetration = new Penetration("Dropper Penetration");

    public Trigger underDropper = new Trigger(() -> (penetration.getDepth() != null);
}
