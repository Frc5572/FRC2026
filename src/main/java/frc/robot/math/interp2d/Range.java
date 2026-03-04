package frc.robot.math.interp2d;

import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Uniform grid range */
public class Range {

    public final double min;
    public final double max;
    public final int discretization;
    public final double range;
    public final double step;

    /** Uniform grid range */
    @TypeStateBuilder("RangeOf")
    public Range(@RequiredField double min, @RequiredField double max,
        @RequiredField int discretization) {
        this.min = min;
        this.max = max;
        this.discretization = discretization;
        this.range = max - min;
        this.step = this.range / (discretization + 1);
    }

    /** Individual grid values */
    public double[] values() {
        double[] res = new double[discretization + 2];
        for (int i = 0; i < discretization + 1; i++) {
            res[i] = min + step * i;
        }
        res[discretization + 1] = max;
        return res;
    }

    /** Index into values lower than given value */
    public int lowerIndex(double v) {
        return (int) Math.min(discretization, Math.max(0, Math.floor((v - min) / step)));
    }

    /** Index into values greater than or equal to given value */
    public int upperIndex(double v) {
        return lowerIndex(v) + 1;
    }

    /** Get grid value for a given index */
    public double valueForIndex(int index) {
        return this.min + this.step * index;
    }

}
