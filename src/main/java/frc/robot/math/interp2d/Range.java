package frc.robot.math.interp2d;

import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

public class Range {

    public final double min;
    public final double max;
    public final int discretization;
    public final double range;
    public final double step;

    @TypeStateBuilder("RangeOf")
    public Range(@RequiredField double min, @RequiredField double max,
        @RequiredField int discretization) {
        this.min = min;
        this.max = max;
        this.discretization = discretization;
        this.range = max - min;
        this.step = this.range / (discretization + 1);
    }

    public double[] values() {
        double[] res = new double[discretization + 2];
        for (int i = 0; i < discretization + 1; i++) {
            res[i] = min + step * i;
        }
        res[discretization + 1] = max;
        return res;
    }

    public int lowerIndex(double v) {
        return (int) Math.min(discretization, Math.max(0, Math.floor((v - min) / step)));
    }

    public int upperIndex(double v) {
        return lowerIndex(v) + 1;
    }

    public double valueForIndex(int index) {
        return this.min + this.step * index;
    }

}
