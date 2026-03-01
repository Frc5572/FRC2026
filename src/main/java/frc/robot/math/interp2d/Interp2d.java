package frc.robot.math.interp2d;

import java.util.Arrays;
import java.util.function.ToDoubleFunction;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.math.geometry.Triangle2d;
import frc.robot.math.geometry.Triangle2d.ClosestPoint;
import io.github.jdiemke.triangulation.DelaunayTriangulator;
import io.github.jdiemke.triangulation.NotEnoughPointsException;
import io.github.jdiemke.triangulation.Vector2D;

/** Interpolate between two independent parameters. */
public class Interp2d<T> {

    private final MulAdd<T> mulAdd;
    private final T[] data;
    private final Translation2d[] points;
    private final int[] triangles;

    /** Create new Interp2d. */
    public Interp2d(T[] data, MulAdd<T> mulAdd, ToDoubleFunction<T> xFunc,
        ToDoubleFunction<T> yFunc) {
        this.points = Arrays.stream(data)
            .map(item -> new Translation2d(xFunc.applyAsDouble(item), yFunc.applyAsDouble(item)))
            .toArray(Translation2d[]::new);
        DelaunayTriangulator triangulator = new DelaunayTriangulator(
            Arrays.stream(this.points).map(x -> new Vector2D(x.getX(), x.getY())).toList());
        try {
            triangulator.triangulate();
        } catch (NotEnoughPointsException e) {
            e.printStackTrace();
        }
        this.triangles = new int[triangulator.getTriangles().size() * 3];
        for (int i = 0; i < triangulator.getTriangles().size(); i++) {
            var tri = triangulator.getTriangles().get(i);
            Vector2D[] points = new Vector2D[] {tri.a, tri.b, tri.c};
            for (int j = 0; j < 3; j++) {
                Vector2D point = points[j];
                int minIdx = 0;
                double minDist = Double.MAX_VALUE;
                for (int k = 0; k < this.points.length; k++) {
                    double dist =
                        this.points[k].getSquaredDistance(new Translation2d(point.x, point.y));
                    if (dist < minDist) {
                        minDist = dist;
                        minIdx = k;
                    }
                }
                this.triangles[3 * i + j] = minIdx;
            }
        }
        this.mulAdd = mulAdd;
        this.data = data;
    }

    /** Result of a query at a point */
    public static record QueryResult<T>(T value, double sdf) {
    }

    /** Get data at a given x,y point */
    public QueryResult<T> query(Translation2d q) {
        double minDist = Double.MAX_VALUE;
        ClosestPoint closestRes = null;
        int closestIndex = 0;
        boolean inside = false;
        double sdf = 0.0;
        for (int i = 0; i < this.triangles.length / 3; i++) {
            Triangle2d tri = new Triangle2d(this.points[this.triangles[3 * i + 0]],
                this.points[this.triangles[3 * i + 1]], this.points[this.triangles[3 * i + 2]]);
            var x = tri.closestPoint(q);
            if (x.squaredDistance() < minDist) {
                minDist = x.squaredDistance();
                closestRes = x;
                closestIndex = i;
                if (x.inside()) {
                    inside = true;
                    sdf = tri.sdf(q);
                    // we can return early if inside, nothing will be closer.
                    break;
                }
            }
        }
        if (!inside) {
            sdf = Math.sqrt(minDist);
        }

        double u = 1.0 - closestRes.v() - closestRes.w();
        T a = this.data[this.triangles[closestIndex * 3 + 0]];
        T b = this.data[this.triangles[closestIndex * 3 + 1]];
        T c = this.data[this.triangles[closestIndex * 3 + 2]];
        return new QueryResult<T>(
            mulAdd.add(mulAdd.add(mulAdd.mul(a, u), mulAdd.mul(b, closestRes.v())),
                mulAdd.mul(c, closestRes.w())),
            sdf);
    }

}
