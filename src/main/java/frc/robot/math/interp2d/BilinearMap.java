package frc.robot.math.interp2d;

import java.util.Optional;
import java.util.TreeMap;

/**
 * Bilinear interpolation over a sparse 2D grid.
 *
 * <p>Entries are stored at (key1, key2) grid points. Missing grid points are handled:
 * a query that lacks one surrounding axis row degrades to 1D interpolation along the other axis,
 * and queries outside the populated region clamp to the nearest boundary value. Returns empty only
 * when the map contains no data at all.
 *
 * @param <T> value type; arithmetic is defined by a {@link MulAdd} instance
 */
public class BilinearMap<T> {

    private final MulAdd<T> ops;
    private final TreeMap<Double, TreeMap<Double, T>> data = new TreeMap<>();

    public BilinearMap(MulAdd<T> ops) {
        this.ops = ops;
    }

    public void put(double key1, double key2, T value) {
        data.computeIfAbsent(key1, k -> new TreeMap<>()).put(key2, value);
    }

    /**
     * Returns the bilinearly interpolated value at (key1, key2), clamping to the populated region
     * when the query falls outside it.
     */
    public Optional<T> get(double key1, double key2) {
        if (data.isEmpty()) return Optional.empty();

        var lo1 = data.floorEntry(key1);
        var hi1 = data.ceilingEntry(key1);

        Optional<T> atLo = lo1 == null ? Optional.empty() : interp1D(lo1.getValue(), key2);
        Optional<T> atHi = hi1 == null ? Optional.empty() : interp1D(hi1.getValue(), key2);

        if (atLo.isEmpty()) return atHi;
        if (atHi.isEmpty()) return atLo;

        double d0 = lo1.getKey(), d1 = hi1.getKey();
        if (d0 == d1) return atLo;
        return Optional.of(lerp(atLo.get(), atHi.get(), (key1 - d0) / (d1 - d0)));
    }

    private Optional<T> interp1D(TreeMap<Double, T> row, double key) {
        if (row.isEmpty()) return Optional.empty();
        var lo = row.floorEntry(key);
        var hi = row.ceilingEntry(key);
        if (lo == null) return Optional.of(hi.getValue());
        if (hi == null) return Optional.of(lo.getValue());
        double k0 = lo.getKey(), k1 = hi.getKey();
        if (k0 == k1) return Optional.of(lo.getValue());
        return Optional.of(lerp(lo.getValue(), hi.getValue(), (key - k0) / (k1 - k0)));
    }

    private T lerp(T a, T b, double t) {
        return ops.add(ops.mul(a, 1.0 - t), ops.mul(b, t));
    }
}
