package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import java.util.stream.IntStream;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

    public static final Distance fieldLength = Inches.of(690.876);
    public static final Distance fieldWidth = Inches.of(317);
    /** Measured from the inside of starting line */
    public static final Distance startingLineX = Inches.of(299.438);

    /** Barge Constants */
    public static class Barge {
        public static final Translation2d farCage =
            new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
            new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
            new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final Distance deepHeight = Inches.of(3.125);
        public static final Distance shallowHeight = Inches.of(30.125);
    }

    /** Coral Station Constants */
    public static class CoralStation {

        public static final Translation2d rightFarCorner =
            new Translation2d(Units.inchesToMeters(67.778), 0);
        public static final Translation2d rightCloseCorner =
            new Translation2d(0, Units.inchesToMeters(49.875));

        public static final double rightM = (rightFarCorner.getY() - rightCloseCorner.getY())
            / (rightFarCorner.getX() - rightCloseCorner.getX());
        public static final double rightB = rightFarCorner.getY() - rightM * rightFarCorner.getX();

        public static final Translation2d leftFarCorner =
            new Translation2d(rightFarCorner.getX(), fieldWidth.in(Meters) - rightFarCorner.getY());
        public static final Translation2d leftCloseCorner = new Translation2d(
            rightCloseCorner.getX(), fieldWidth.in(Meters) - rightCloseCorner.getY());

        public static final double leftM = (leftFarCorner.getY() - leftCloseCorner.getY())
            / (leftFarCorner.getX() - leftCloseCorner.getX());
        public static final double leftB = leftFarCorner.getY() - leftM * leftFarCorner.getX();

    }

    /** Reef Constants */
    public static class Reef {

        public static final Translation2d center =
            new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

        public static final Distance inscribedRadius = Inches.of(65.491 / 2);
        public static final Distance circumscribedRadius =
            Meters.of(inscribedRadius.in(Meters) / Math.cos(Math.PI / 6));

        public static final Translation2d[] reefVertices = IntStream.range(0, 6).mapToObj(i -> {
            double y = center.getY() + Math.cos(Math.PI / 3.0 * i) * circumscribedRadius.in(Meters);
            double x = center.getX() + Math.sin(Math.PI / 3.0 * i) * circumscribedRadius.in(Meters);
            return new Translation2d(x, y);
        }).toArray(Translation2d[]::new);

        // public static final Axis[] collisionAxes = new Axis[]
        // {Axis.fromRotation(Rotation2d.kZero),
        // Axis.fromRotation(Rotation2d.fromDegrees(60)),
        // Axis.fromRotation(Rotation2d.fromDegrees(120)),};

    }

}
