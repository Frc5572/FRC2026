package frc.robot.math.geometry;

import edu.wpi.first.math.geometry.Translation2d;

/** 2D triangle made up of 3 vertices. */
public class Triangle2d {

    public final Translation2d a;
    public final Translation2d b;
    public final Translation2d c;

    /** Create new Triangle2d. */
    public Triangle2d(Translation2d a, Translation2d b, Translation2d c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    /** Record holding results of a call to {@link Triangle2d#closestPoint}. */
    public static record ClosestPoint(Translation2d point, double v, double w,
        double squaredDistance, boolean inside) {
    }

    /** Finds closest point on triangle to a query point. */
    public ClosestPoint closestPoint(Translation2d p) {
        Translation2d ab = b.minus(a);
        Translation2d ac = c.minus(a);
        Translation2d ap = p.minus(a);

        double d1 = dot(ab, ap);
        double d2 = dot(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0) {
            return new ClosestPoint(a, 0.0, 0.0, ap.getSquaredNorm(), false);
        }

        Translation2d bp = p.minus(b);
        double d3 = dot(ab, bp);
        double d4 = dot(ac, bp);
        if (d3 >= 0.0 && d4 <= d3) {
            return new ClosestPoint(b, 1.0, 0.0, bp.getSquaredNorm(), false);
        }

        Translation2d cp = p.minus(c);
        double d5 = dot(ab, cp);
        double d6 = dot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6) {
            return new ClosestPoint(c, 0.0, 1.0, cp.getSquaredNorm(), false);
        }

        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            double v = d1 / (d1 - d3);
            Translation2d r = a.plus(ab.times(v));
            Translation2d vw = barycentric(a, ab, ac, r);
            return new ClosestPoint(r, vw.getX(), vw.getY(), r.getSquaredDistance(p), false);
        }

        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            double v = d2 / (d2 - d6);
            Translation2d r = a.plus(ac.times(v));
            Translation2d vw = barycentric(a, ab, ac, r);
            return new ClosestPoint(r, vw.getX(), vw.getY(), r.getSquaredDistance(p), false);
        }

        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            double v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            Translation2d r = b.plus((c.minus(b)).times(v));
            Translation2d vw = barycentric(a, ab, ac, r);
            return new ClosestPoint(r, vw.getX(), vw.getY(), r.getSquaredDistance(p), false);
        }

        double denom = 1.0 / (va + vb + vc);
        double v = vb * denom;
        double w = vc * denom;
        Translation2d r = a.plus(ab.times(v)).plus(ac.times(w));
        Translation2d vw = barycentric(a, ab, ac, r);
        return new ClosestPoint(r, vw.getX(), vw.getY(), r.getSquaredDistance(p), true);
    }

    private static Translation2d barycentric(Translation2d a_, Translation2d ab, Translation2d ac,
        Translation2d r) {
        double a = ab.getX();
        double b = ac.getX();
        double c = ab.getY();
        double d = ac.getY();
        double det = 1.0 / (a * d - b * c);

        double rx = r.getX() - a_.getX();
        double ry = r.getY() - a_.getY();

        double v = det * (d * rx - b * ry);
        double w = det * (a * ry - c * rx);
        return new Translation2d(v, w);
    }

    private static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

}
