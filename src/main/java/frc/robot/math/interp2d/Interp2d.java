package frc.robot.math.interp2d;

import java.util.Arrays;
import java.util.function.ToDoubleFunction;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.math.geometry.DelaunayTriangulation;
import frc.robot.math.geometry.Triangle2d;
import frc.robot.math.geometry.Triangle2d.ClosestPoint;

/** Interpolate between two independent parameters. */
public class Interp2d<T> {

    private final DelaunayTriangulation triangulation;
    private final MulAdd<T> mulAdd;
    private final T[] data;
    private final Translation2d[] points;

    /** Create new Interp2d. */
    public Interp2d(T[] data, MulAdd<T> mulAdd, ToDoubleFunction<T> xFunc,
        ToDoubleFunction<T> yFunc) {
        this.points = Arrays.stream(data)
            .map(item -> new Translation2d(xFunc.applyAsDouble(item), yFunc.applyAsDouble(item)))
            .toArray(Translation2d[]::new);
        this.triangulation = new DelaunayTriangulation(points);
        this.mulAdd = mulAdd;
        this.data = data;
    }

    public static record QueryResult<T>(T value, double sdf) {
    }

    /** Get data at a given x,y point */
    public QueryResult<T> query(Translation2d q) {
        double minDist = Double.MAX_VALUE;
        ClosestPoint closestRes = null;
        int closestIndex = 0;
        boolean inside = false;
        double sdf = 0.0;
        for (int i = 0; i < triangulation.triangles.length; i++) {
            Triangle2d tri = triangulation.triangles[i];
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
        T a = this.data[this.triangulation.indices[closestIndex * 3 + 0]];
        T b = this.data[this.triangulation.indices[closestIndex * 3 + 1]];
        T c = this.data[this.triangulation.indices[closestIndex * 3 + 2]];
        return new QueryResult<T>(
            mulAdd.add(mulAdd.add(mulAdd.mul(a, u), mulAdd.mul(b, closestRes.v())),
                mulAdd.mul(c, closestRes.w())),
            sdf);
    }

    /** Create a surrogate. */
    public BilinearSurrogate<T> surrogate(Range xRange, Range yRange) {
        return new BilinearSurrogate<T>(xRange, yRange, this::query, this.mulAdd);
    }

}
