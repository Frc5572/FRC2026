package frc.robot.math.interp2d;

import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

public class Range {

    public final double min;
    public final double max;
    public final int discretization;

    @TypeStateBuilder("RangeOf")
    public Range(@RequiredField double min, @RequiredField double max,
        @RequiredField int discretization) {
        this.min = min;
        this.max = max;
        this.discretization = discretization;
    }

    public double[] values() {
        double[] res = new double[discretization + 2];
        double range = max - min;
        double step = range / (discretization + 1);
        for (int i = 0; i < discretization + 1; i++) {
            res[i] = min + step * i;
        }
        res[discretization + 1] = max;
        return res;
    }

}
