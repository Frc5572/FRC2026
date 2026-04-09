package frc.robot.shotdata;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.ToDoubleFunction;
import frc.robot.math.interp2d.MulAdd;

public class SemiGriddedBilinearInterpolation<T> {

    public final double gridStep;
    private final ToDoubleFunction<T> yFunc;
    private final Map<Integer, ArrayList<T>> data = new HashMap<>();
    private final MulAdd<T> mulAdd;
    private final int minBin;
    private final int maxBin;

    public SemiGriddedBilinearInterpolation(double gridStep, T[] data, MulAdd<T> mulAdd,
        ToDoubleFunction<T> xFunc, ToDoubleFunction<T> yFunc) {
        this.gridStep = gridStep;
        this.mulAdd = mulAdd;
        this.yFunc = yFunc;

        int min = Integer.MAX_VALUE;
        int max = Integer.MIN_VALUE;

        for (var datum : data) {
            double x = xFunc.applyAsDouble(datum);
            int bin = calculateBin(x);
            min = Math.min(min, bin);
            max = Math.max(max, bin);
            this.data.computeIfAbsent(bin, _x -> new ArrayList<>()).add(datum);
        }

        this.minBin = min;
        this.maxBin = max;

        for (var key : this.data.keySet()) {
            this.data.get(key)
                .sort((a, b) -> Double.compare(yFunc.applyAsDouble(a), yFunc.applyAsDouble(b)));
        }
    }

    public T interpolate(double x, double y) {
        int bin = calculateBin(x);
        int otherBin = calculateBin(x + gridStep / 2.0);
        if (otherBin == bin) {
            otherBin = calculateBin(x - gridStep / 2.0);
        }
        if (data.get(bin) == null) {
            int distToMin = Math.abs(bin - minBin);
            int distToMax = Math.abs(bin - maxBin);
            if (distToMax > distToMin) {
                return interpolateY(minBin, y);
            } else {
                return interpolateY(maxBin, y);
            }
        }
        if (data.get(otherBin) == null) {
            return interpolateY(bin, y);
        }
        double x0 = binX(bin);
        double x1 = binX(otherBin);
        T v0 = interpolateY(bin, y);
        T v1 = interpolateY(otherBin, y);
        // if x0 > x1 (i.e. next closest bin is less than this bin), then swap the xs and vs.
        if (x0 > x1) {
            double x2 = x1;
            x1 = x0;
            x0 = x2;

            T v2 = v1;
            v1 = v0;
            v0 = v2;
        }
        double t = (x - x0) / (x1 - x0);
        return mulAdd.add(mulAdd.mul(v0, 1 - t), mulAdd.mul(v1, t));
    }

    private T interpolateY(int bin, double y) {
        var ys = data.get(bin);
        if (y < yFunc.applyAsDouble(ys.get(0))) {
            return ys.get(0);
        } else if (y > yFunc.applyAsDouble(ys.get(ys.size() - 1))) {
            return ys.get(ys.size() - 1);
        }
        int start = 0;
        int end = ys.size() - 1;
        while (end - start > 1) {
            int mid = (start + end) / 2;
            double midY = yFunc.applyAsDouble(ys.get(mid));
            if (midY == y) {
                return ys.get(mid);
            } else if (midY < y) {
                start = mid;
            } else {
                end = mid;
            }
        }
        double startY = yFunc.applyAsDouble(ys.get(start));
        double endY = yFunc.applyAsDouble(ys.get(end));
        double t = (y - startY) / (endY - startY);
        System.out.println(t);
        return mulAdd.add(mulAdd.mul(ys.get(start), 1 - t), mulAdd.mul(ys.get(end), t));
    }

    private int calculateBin(double x) {
        return (int) Math.round(x / gridStep);
    }

    private double binX(int bin) {
        return bin * gridStep;
    }

}
