package frc.robot.math.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Represents an axis. */
public class Axis {

    private double xDir;
    private double yDir;

    /** Create new axis with a given direction. Either xDir or yDir must be non-zero. */
    public Axis(double xDir, double yDir) {
        setDirectionImpl(xDir, yDir);
    }

    /** Create an axis with the same direction as the given rotation. */
    public static Axis fromRotation(Rotation2d rot) {
        return new Axis(rot.getCos(), rot.getSin());
    }

    /** Set axis direction to the same direction as the given rotation. */
    public void setFromRotation(Rotation2d rot) {
        setDirectionImpl(rot.getCos(), rot.getSin());
    }

    /**
     * Get opposite direction. Is equivalent as far as the definition of an axis is concerned, but
     * for implementation purposes, has a negative direction.
     */
    public Axis unaryMinus() {
        return new Axis(-xDir, -yDir);
    }

    /**
     * Get the X direction.
     */
    public double getX() {
        return xDir;
    }

    /**
     * Get the Y direction.
     */
    public double getY() {
        return yDir;
    }

    /**
     * Set the direction. Either xDir or yDir must be non-zero.
     */
    public void setDirection(double xDir, double yDir) {
        setDirectionImpl(xDir, yDir);
    }

    private void setDirectionImpl(double xDir, double yDir) {
        double norm = Math.hypot(xDir, yDir);
        this.xDir = xDir / norm;
        this.yDir = yDir / norm;
    }

    /**
     * Get the dot product between this direction and a given point.
     */
    public double dot(Translation2d point) {
        return this.xDir * point.getX() + this.yDir * point.getY();
    }

    /** Project multiple points onto this axis. */
    public Interval project(Translation2d[] points) {
        double v = 0.0;
        Translation2d p = points[0];
        double min = this.dot(p);
        double max = min;

        for (int i = 1; i < points.length; i++) {
            p = points[i];
            v = this.dot(p);

            if (v < min) {
                min = v;
            } else if (v > max) {
                max = v;
            }
        }

        return new Interval(min, max);
    }

}
