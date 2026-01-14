package frc.lib.math;

/** A one-dimensional range. */
public class Interval {

    private double min;
    private double max;

    /** A one-dimensional range. */
    public Interval(double min, double max) {
        this.min = min;
        this.max = max;
    }

    /** Get lower extent of the range. */
    public double getMin() {
        return min;
    }

    /** Set lower extent of the range. */
    public void setMin(double min) {
        this.min = min;
    }

    /** Get upper extent of the range. */
    public double getMax() {
        return max;
    }

    /** Set upper extent of the range. */
    public void setMax(double max) {
        this.max = max;
    }

    /** Get if two intervals share some common subset. */
    public boolean overlaps(Interval other) {
        return !(this.min > other.max || other.min > this.max);
    }

    /** Get length of common subset. */
    public double getOverlap(Interval other) {
        // make sure they overlap
        if (this.overlaps(other)) {
            return Math.min(this.max, other.max) - Math.max(this.min, other.min);
        }
        return 0;
    }

    /** Get if `other` is completely contained by this range. */
    public boolean contains(Interval other) {
        return other.min > this.min && other.max < this.max;
    }

}
