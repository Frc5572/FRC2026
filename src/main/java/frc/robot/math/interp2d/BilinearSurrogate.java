package frc.robot.math.interp2d;

import java.util.function.Function;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.math.interp2d.Interp2d.QueryResult;

/** Bilinear intrpolation of an {@link Interp2d}. */
public class BilinearSurrogate<T> {

    private final Range xRange;
    private final Range yRange;
    private final T[][] data;
    private final double[][] sdf;
    private final MulAdd<T> mulAdd;

    /** Bilinear intrpolation of an {@link Interp2d}. */
    @SuppressWarnings("unchecked")
    public BilinearSurrogate(Range xRange, Range yRange,
        Function<Translation2d, QueryResult<T>> eval, MulAdd<T> mulAdd) {
        var xs = xRange.values();
        var ys = yRange.values();
        this.xRange = xRange;
        this.yRange = yRange;
        this.mulAdd = mulAdd;

        this.data = (T[][]) new Object[xs.length][];
        this.sdf = new double[xs.length][];
        for (int i = 0; i < xs.length; i++) {
            this.data[i] = (T[]) new Object[ys.length];
            this.sdf[i] = new double[ys.length];
        }
        for (int ix = 0; ix < xs.length; ix++) {
            double x = xs[ix];
            for (int iy = 0; iy < ys.length; iy++) {
                double y = ys[iy];
                var qRes = eval.apply(new Translation2d(x, y));
                this.data[ix][iy] = qRes.value();
                this.sdf[ix][iy] = qRes.sdf();
            }
        }
    }

    /** Get data at a given x,y point */
    public QueryResult<T> query(Translation2d q) {
        double x = q.getX();
        double y = q.getY();
        if (x < xRange.min) {
            x = xRange.min;
        }
        if (x > xRange.max) {
            x = xRange.max;
        }
        if (y < yRange.min) {
            y = yRange.min;
        }
        if (y > yRange.max) {
            y = yRange.max;
        }
        int ix1 = xRange.lowerIndex(x);
        int ix2 = ix1 + 1;
        int iy1 = yRange.lowerIndex(y);
        int iy2 = iy1 + 1;
        double x1 = xRange.valueForIndex(ix1);
        double y1 = xRange.valueForIndex(iy1);
        double x2 = xRange.valueForIndex(ix2);
        double y2 = xRange.valueForIndex(iy2);
        T fq11 = data[ix1][iy1];
        T fq12 = data[ix1][iy2];
        T fq21 = data[ix2][iy1];
        T fq22 = data[ix2][iy2];
        T fxy1 = mulAdd.add(mulAdd.mul(fq11, (x2 - x) / (x2 - x1)),
            mulAdd.mul(fq21, (x - x1) / (x2 - x1)));
        T fxy2 = mulAdd.add(mulAdd.mul(fq12, (x2 - x) / (x2 - x1)),
            mulAdd.mul(fq22, (x - x1) / (x2 - x1)));
        T fxy = mulAdd.add(mulAdd.mul(fxy1, (y2 - y) / (y2 - y1)),
            mulAdd.mul(fxy2, (y - y1) / (y2 - y1)));

        double sq11 = sdf[ix1][iy1];
        double sq12 = sdf[ix1][iy2];
        double sq21 = sdf[ix2][iy1];
        double sq22 = sdf[ix2][iy2];
        double sxy1 = (x2 - x) / (x2 - x1) * sq11 + (x - x1) / (x2 - x1) * sq21;
        double sxy2 = (x2 - x) / (x2 - x1) * sq12 + (x - x1) / (x2 - x1) * sq22;
        double sxy = (y2 - y) / (y2 - y1) * sxy1 + (y - y1) / (y2 - y1) * sxy2;

        return new QueryResult<T>(fxy, sxy);
    }

}
