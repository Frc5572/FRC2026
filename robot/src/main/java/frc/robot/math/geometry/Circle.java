package frc.robot.math.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Represents a circle with a known center and radius. */
public class Circle implements ConvexShape {

    public String name;
    private Translation2d center;
    private double radius;

    /** Represents a circle with a known center and radius. */
    public Circle(String name, Translation2d center, double radius) {
        this.name = name;
        this.center = center;
        this.radius = radius;
    }

    private static final int RESOLUTION = 20;


    private static final Axis[] axes = new Axis[0];

    @Override
    public Axis[] getAxes() {
        return axes;
    }

    @Override
    public Interval project(Axis axis) {
        double c = axis.dot(center);
        return new Interval(c - this.radius, c + this.radius);
    }

    @Override
    public Translation2d getCenter() {
        return center;
    }

    /** Set the circle's center */
    public void setCenter(Translation2d center) {
        this.center = center;
    }

    /** Get the circle's radius */
    public double getRadius() {
        return radius;
    }

    /** Set the circle's radius */
    public void setRadius(double radius) {
        this.radius = radius;
    }

    /** Get if a point is within the boundary of the circle. */
    public boolean contains(Translation2d pt) {
        return pt.minus(center).getNorm() < radius;
    }

    /** Get the signed distance of a point with respect to the circle. */
    public double sdf(Translation2d pt) {
        return pt.minus(center).getNorm() - radius;
    }

    /** Get angle of a point with respect to the circle's center. */
    public Rotation2d getAngle(Translation2d pt) {
        return pt.minus(center).getAngle();
    }

    /** Get point on the circle with a given angle. */
    public Translation2d getVertex(Rotation2d rotation) {
        return getVertex(rotation, 0);
    }

    /** Get point on the circle with a given angle and extra radius. */
    public Translation2d getVertex(Rotation2d rotation, double extra) {
        return center.plus(new Translation2d(radius + extra, rotation));
    }

    /**
     * Get angles for points that are both tangent to the circle and colinear with a given point.
     */
    public RotationInterval circleTangentAngles(Translation2d p) {
        Translation2d diff = p.minus(center);
        double d = diff.getNorm();
        double det = radius / d;
        if (det > 1.0 || det < -1.0) {
            var angle = getAngle(p);
            return new RotationInterval(angle.minus(Rotation2d.fromDegrees(5)),
                angle.plus(Rotation2d.fromDegrees(5)));
        }
        double dtheta = Math.acos(det);
        if (dtheta < 0.0) {
            dtheta = -dtheta;
        }

        Rotation2d dAngle = Rotation2d.fromRadians(dtheta);
        return RotationInterval.acute(diff.getAngle().plus(dAngle), diff.getAngle().minus(dAngle));
    }

}
