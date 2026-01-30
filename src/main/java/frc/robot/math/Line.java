package frc.robot.math;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;

/** Represents a line in 2d space. Uses Ax + By + c = 0 representation. */
public class Line {

    public double a;
    public double b;
    public double c;
    public String name;

    /** Represents a line in 2d space. Uses Ax + By + c = 0 representation. */
    public Line(String name, double a, double b, double c) {
        this.name = name;
        this.a = a;
        this.b = b;
        this.c = c;
    }

    private static final double minX = -20.0;
    private static final double maxX = FieldConstants.fieldLength + 20.0;
    private static final double minY = -20.0;
    private static final double maxY = FieldConstants.fieldWidth + 20.0;


    /** Get intersection of two lines. Returns null if lines are parallel. */
    public Translation2d intersection(Line other) {
        double d = a * other.b - b * other.a;
        if (d != 0) {
            double dx = -c * other.b + b * other.c;
            double dy = -a * other.c + c * other.a;
            return new Translation2d(dx / d, dy / d);
        } else {
            return null;
        }
    }

}
