package frc.robot.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A static hexagon shape. */
public class Hexagon implements ConvexShape {

    private final Axis[] axes = new Axis[] {Axis.fromRotation(Rotation2d.kZero),
        Axis.fromRotation(Rotation2d.fromDegrees(60)),
        Axis.fromRotation(Rotation2d.fromDegrees(120))};

    private final Translation2d[] vertices;

    private final Translation2d center;
    private final String name;

    /** A static hexagon shape. */
    public Hexagon(String name, Translation2d center, double radius, Rotation2d offset) {
        this.name = name;
        this.center = center;
        this.vertices = new Translation2d[7];
        for (int i = 0; i < 6; i++) {
            Rotation2d rot = offset.plus(Rotation2d.fromRotations(i / 6.0));
            this.vertices[i] = center.plus(new Translation2d(radius, rot));
        }
        this.vertices[6] = this.vertices[0];
    }

    @Override
    public Axis[] getAxes() {
        return axes;
    }

    @Override
    public Interval project(Axis axis) {
        return axis.project(this.vertices);
    }

    @Override
    public Translation2d getCenter() {
        return this.center;
    }


    /** Returns true if `p` is inside this hexagon. */
    public boolean contains(Translation2d p) {
        int size = this.vertices.length;
        Translation2d p1 = this.vertices[size - 1];
        Translation2d p2 = this.vertices[0];

        double last = getLocation(p, p1, p2);
        for (int i = 0; i < size - 1; i++) {
            p1 = p2;
            p2 = this.vertices[i + 1];

            double location = getLocation(p, p1, p2);

            if (location == 0) {
                return false;
            }

            if (last * location < 0) {
                return false;
            }

            if (Math.abs(location) > 1e-5) {
                last = location;
            }
        }

        return true;
    }

    private static double getLocation(Translation2d point, Translation2d linePoint1,
        Translation2d linePoint2) {
        return (linePoint2.getX() - linePoint1.getX()) * (point.getY() - linePoint1.getY())
            - (point.getX() - linePoint1.getX()) * (linePoint2.getY() - linePoint1.getY());
    }

}
