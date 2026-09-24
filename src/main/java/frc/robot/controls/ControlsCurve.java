package frc.robot.controls;

import java.util.Arrays;
import org.jspecify.annotations.NullMarked;

/**
 * The response curve mapping normalised stick travel to normalised output.
 *
 * <p>
 * Input and output are both in {@code [0, 1]}: the deadband and saturation point have already
 * been applied by the caller, so {@code 0} is "just leaving the deadband" and {@code 1} is "at or
 * past the saturation point".
 *
 * <h2>Modes</h2>
 * <ul>
 * <li>{@link Mode#POWER} raises the input to {@link ControlsConfig#translationExponent()}. An
 * exponent of 1.0 is linear and 2.0 is the squared response this code used to hard-code. These
 * are the presets offered in the tuner.</li>
 * <li>{@link Mode#CUSTOM} interpolates a set of knots, in the manner of a curve editor.</li>
 * </ul>
 *
 * <h2>Why a monotone spline</h2> Custom curves are evaluated with monotone cubic Hermite
 * interpolation (the Fritsch&ndash;Carlson construction). An ordinary spline through
 * driver-placed knots can overshoot, producing a curve that dips or briefly runs backwards, which
 * on a drivetrain feels like the robot fighting the stick. Monotone interpolation cannot do that:
 * more stick always means at least as much output.
 *
 * <p>
 * Knots are held as a flat {@code [x0, y0, x1, y1, ...]} array so the curve logs as a single
 * {@code double[]} and survives replay unchanged. The first and last knots are pinned to
 * {@code (0, 0)} and {@code (1, 1)} so that centring the stick always stops and burying it always
 * commands full output.
 */
@NullMarked
public final class ControlsCurve {

    /** How the curve is evaluated. */
    public enum Mode {
        /** Input raised to the configured exponent. */
        POWER,
        /** Monotone spline through {@link ControlsCurve#knots()}. */
        CUSTOM
    }

    /** Largest number of knots a custom curve may hold. */
    public static final int MAX_KNOTS = 10;

    private static final double MIN_SPACING = 1e-3;

    private final Mode mode;
    private final double[] knots;

    private ControlsCurve(Mode mode, double[] knots) {
        this.mode = mode;
        this.knots = knots;
    }

    /** A curve that defers to the configured exponent. */
    public static ControlsCurve power() {
        return new ControlsCurve(Mode.POWER, defaultKnots());
    }

    /**
     * A custom curve through the given knots.
     *
     * @param flatKnots knots as {@code [x0, y0, x1, y1, ...]}; sanitised on the way in
     * @return the curve
     */
    public static ControlsCurve custom(double[] flatKnots) {
        return new ControlsCurve(Mode.CUSTOM, sanitize(flatKnots));
    }

    /**
     * Build a curve from a mode name and knots, tolerating anything unrecognised.
     *
     * @param modeName mode name, matched case-insensitively; anything else means POWER
     * @param flatKnots knots as {@code [x0, y0, x1, y1, ...]}
     * @return the curve
     */
    public static ControlsCurve of(String modeName, double[] flatKnots) {
        Mode parsed = Mode.POWER;
        for (Mode candidate : Mode.values()) {
            if (candidate.name().equalsIgnoreCase(modeName)) {
                parsed = candidate;
            }
        }
        return new ControlsCurve(parsed, sanitize(flatKnots));
    }

    /** The straight line through {@code (0, 0)} and {@code (1, 1)}. */
    public static double[] defaultKnots() {
        return new double[] {0.0, 0.0, 1.0, 1.0};
    }

    /** How this curve is evaluated. */
    public Mode mode() {
        return mode;
    }

    /** A defensive copy of the knots, as {@code [x0, y0, x1, y1, ...]}. */
    public double[] knots() {
        return knots.clone();
    }

    /** The number of knots. */
    public int knotCount() {
        return knots.length / 2;
    }

    /** This curve with a different mode, keeping the knots so a toggle loses no work. */
    public ControlsCurve withMode(Mode newMode) {
        return new ControlsCurve(newMode, knots.clone());
    }

    /**
     * Evaluate the curve.
     *
     * @param x normalised stick travel, clamped to {@code [0, 1]}
     * @param exponent exponent used when the mode is {@link Mode#POWER}
     * @return normalised output in {@code [0, 1]}
     */
    public double evaluate(double x, double exponent) {
        double clamped = Math.max(0.0, Math.min(1.0, x));
        if (mode == Mode.POWER) {
            return Math.pow(clamped, Math.max(exponent, 1e-6));
        }
        return Math.max(0.0, Math.min(1.0, interpolate(clamped)));
    }

    /** Monotone cubic Hermite interpolation through the knots. */
    private double interpolate(double x) {
        int n = knots.length / 2;
        if (n < 2) {
            return x;
        }
        // Locate the span containing x.
        int i = 0;
        for (int k = 0; k < n - 1; k++) {
            if (x >= knots[2 * k] && x <= knots[2 * k + 2]) {
                i = k;
                break;
            }
            i = k;
        }
        double x0 = knots[2 * i];
        double y0 = knots[2 * i + 1];
        double x1 = knots[2 * i + 2];
        double y1 = knots[2 * i + 3];
        double h = x1 - x0;
        if (h <= 0.0) {
            return y1;
        }
        double m0 = tangent(i);
        double m1 = tangent(i + 1);
        double t = (x - x0) / h;
        double t2 = t * t;
        double t3 = t2 * t;
        return (2 * t3 - 3 * t2 + 1) * y0 + (t3 - 2 * t2 + t) * h * m0
            + (-2 * t3 + 3 * t2) * y1 + (t3 - t2) * h * m1;
    }

    /** The Fritsch&ndash;Carlson tangent at knot {@code i}, which cannot induce overshoot. */
    private double tangent(int i) {
        int n = knots.length / 2;
        if (i == 0) {
            return secant(0);
        }
        if (i == n - 1) {
            return secant(n - 2);
        }
        double prev = secant(i - 1);
        double next = secant(i);
        if (prev * next <= 0.0) {
            return 0.0;
        }
        double hPrev = knots[2 * i] - knots[2 * i - 2];
        double hNext = knots[2 * i + 2] - knots[2 * i];
        double w1 = 2 * hNext + hPrev;
        double w2 = hNext + 2 * hPrev;
        return (w1 + w2) / (w1 / prev + w2 / next);
    }

    private double secant(int i) {
        double h = knots[2 * i + 2] - knots[2 * i];
        if (h <= 0.0) {
            return 0.0;
        }
        return (knots[2 * i + 3] - knots[2 * i + 1]) / h;
    }

    /**
     * Put an arbitrary knot array into a usable state.
     *
     * <p>
     * Coordinates are clamped to the unit square, knots sorted by x, knots closer together than
     * {@link #MIN_SPACING} dropped, the count capped at {@link #MAX_KNOTS}, and the endpoints
     * pinned to {@code (0, 0)} and {@code (1, 1)}. A garbled array degrades to a straight line
     * rather than producing an undrivable curve.
     *
     * @param flat knots as {@code [x0, y0, x1, y1, ...]}
     * @return a sanitised knot array with at least two knots
     */
    public static double[] sanitize(double[] flat) {
        if (flat.length < 4 || flat.length % 2 != 0) {
            return defaultKnots();
        }
        int n = flat.length / 2;
        double[][] pts = new double[n][2];
        for (int i = 0; i < n; i++) {
            double x = flat[2 * i];
            double y = flat[2 * i + 1];
            if (!Double.isFinite(x) || !Double.isFinite(y)) {
                return defaultKnots();
            }
            pts[i][0] = Math.max(0.0, Math.min(1.0, x));
            pts[i][1] = Math.max(0.0, Math.min(1.0, y));
        }
        Arrays.sort(pts, (a, b) -> Double.compare(a[0], b[0]));

        double[] out = new double[2 * Math.min(n, MAX_KNOTS)];
        int count = 0;
        for (double[] pt : pts) {
            if (count > 0 && pt[0] - out[2 * count - 2] < MIN_SPACING) {
                continue;
            }
            if (count >= MAX_KNOTS) {
                break;
            }
            out[2 * count] = pt[0];
            out[2 * count + 1] = pt[1];
            count++;
        }
        if (count < 2) {
            return defaultKnots();
        }
        double[] trimmed = Arrays.copyOf(out, 2 * count);
        // Pin the endpoints: centred stick must stop, buried stick must command full output.
        trimmed[0] = 0.0;
        trimmed[1] = 0.0;
        trimmed[trimmed.length - 2] = 1.0;
        trimmed[trimmed.length - 1] = 1.0;
        return trimmed;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof ControlsCurve c && c.mode == mode && Arrays.equals(c.knots, knots);
    }

    @Override
    public int hashCode() {
        return mode.hashCode() * 31 + Arrays.hashCode(knots);
    }

    @Override
    public String toString() {
        return "ControlsCurve{" + mode + ", " + Arrays.toString(knots) + "}";
    }

}
