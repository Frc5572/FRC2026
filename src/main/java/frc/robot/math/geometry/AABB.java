package frc.robot.math.geometry;

import edu.wpi.first.math.geometry.Translation2d;

public class AABB {

    public final Translation2d min;
    public final Translation2d max;

    public AABB(Translation2d min, Translation2d max) {
        this.min = min;
        this.max = max;
    }

    public AABB(Translation2d center, double x, double y) {
        this(center.minus(new Translation2d(x / 2.0, y / 2.0)),
            center.plus(new Translation2d(x / 2.0, y / 2.0)));
    }

    public boolean contains(Translation2d point) {
        if (point.getX() < min.getX()) {
            return false;
        }
        if (point.getY() < min.getY()) {
            return false;
        }

        if (point.getX() > max.getX()) {
            return false;
        }
        if (point.getY() > max.getY()) {
            return false;
        }

        return true;
    }

}
