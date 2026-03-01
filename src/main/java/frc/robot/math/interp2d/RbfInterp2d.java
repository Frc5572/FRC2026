package frc.robot.math.interp2d;

import java.util.Arrays;
import java.util.function.DoubleUnaryOperator;
import java.util.function.ToDoubleFunction;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class RbfInterp2d {

    private final double[] xs;
    private final double[] ys;
    private final DMatrixRMaj ws;
    private final DoubleUnaryOperator rbf;

    public <T> RbfInterp2d(T[] data, ToDoubleFunction<T> xFunc, ToDoubleFunction<T> yFunc,
        ToDoubleFunction<T> zFunc, DoubleUnaryOperator rbf) {
        this.xs = Arrays.stream(data).mapToDouble(xFunc).toArray();
        this.ys = Arrays.stream(data).mapToDouble(yFunc).toArray();
        this.rbf = rbf;
        DMatrixRMaj zs =
            DMatrixRMaj.wrap(xs.length, 1, Arrays.stream(data).mapToDouble(zFunc).toArray());
        this.ws = new DMatrixRMaj(xs.length, 1);
        DMatrixRMaj A = new DMatrixRMaj(xs.length, xs.length);
        for (int i = 0; i < xs.length; i++) {
            for (int j = 0; j < xs.length; j++) {
                if (i == j) {
                    A.set(i, j, rbf.applyAsDouble(0.0));
                } else {
                    double dx = xs[i] - xs[j];
                    double dy = ys[i] - ys[j];
                    double d = Math.hypot(dx, dy);
                    A.set(i, j, rbf.applyAsDouble(d));
                }
            }
        }
        CommonOps_DDRM.solve(A, zs, ws);
        System.out.println("A: " + A);
        System.out.println("zs: " + zs);
        System.out.println("ws: " + ws);
    }

    public double query(double x, double y) {
        double value = 0;
        for (int i = 0; i < xs.length; i++) {
            double dx = x - xs[i];
            double dy = y - ys[i];
            double d = Math.hypot(dx, dy);
            // System.out.println("d: " + d);
            double phi = rbf.applyAsDouble(d);
            // System.out.println("phi: " + phi);
            value += ws.get(i) * phi;
        }
        return value;
    }

}
