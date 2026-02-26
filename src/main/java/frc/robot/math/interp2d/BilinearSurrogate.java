package frc.robot.math.interp2d;

import java.util.function.Function;
import edu.wpi.first.math.geometry.Translation2d;

public class BilinearSurrogate<T> {

    private final double[] xs;
    private final double[] ys;
    private final T[][] data;
    private final double[][] sdf;
    private final MulAdd<T> mulAdd;

    public BilinearSurrogate(Range xRange, Range yRange,
        Function<Translation2d, Interp2d.QueryResult<T>> eval, MulAdd<T> mulAdd) {
        this.xs = xRange.values();
        this.ys = yRange.values();
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

}
