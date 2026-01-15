package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Distance;

public final class FieldConstants {

    private FieldConstants() {}

    public static final Distance fieldLength = Inches.of(651.2225);
    public static final Distance fieldWidth = Inches.of(318.1875);

    public static final class Trench {
        public static final Distance distanceFromAllianceWall = Inches.of(179.11125);
        public static final Distance width = Inches.of(6);
    }

}
