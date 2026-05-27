package frc.gen;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

/**
 * Finds exit-velocity / exit-angle pairs that cause a simulated ball to enter a horizontal hoop.
 *
 * <p>
 * Hoop geometry and backspin are fixed at construction time. Distance and radial velocity are
 * supplied per call so the caller can grid over them.
 */
public class HoopSolver {

    /** A (exitVelocity, exitAngle, tof) triple that passes all validity checks. */
    public record ShotParams(LinearVelocity exitVelocity, Angle exitAngle, Time tof) {
    }

    private static final double DT = 0.001; // simulation step (s)
    private static final double MAX_TIME = 5.0; // bail-out (s)

    private final double hoopHeight; // m above shooter
    private final double hoopRadius; // m — hoop opening radius
    private final double lipHeight; // m — additional clearance above hoopHeight required at the near rim
    private final double backspin; // rad/s — positive = backspin on a forward shot

    /**
     * @param hoopHeight vertical distance from shooter to hoop center
     * @param hoopRadius radius of the hoop opening
     * @param lipHeight minimum height (above shooter) the ball must reach before the hoop; excludes
     *        shots that arc too flat and would clip the near lip
     * @param backspin ball backspin (positive = backspin for a forward shot)
     */
    public HoopSolver(Distance hoopHeight, Distance hoopRadius, Distance lipHeight,
        AngularVelocity backspin) {
        this.hoopHeight = hoopHeight.in(Meters);
        this.hoopRadius = hoopRadius.in(Meters);
        this.lipHeight = lipHeight.in(Meters);
        this.backspin = backspin.in(RadiansPerSecond);
    }

    /**
     * Simulates a shot and returns the interpolated time of flight (s) at which the ball descends
     * through the hoop plane, or -1 if the shot misses or fails the lip check.
     */
    private double simulate(double distanceM, double rvMs, double exitVelocityMs,
        double exitAngleRad) {
        SimulatedShot shot = new SimulatedShot(MetersPerSecond.of(exitVelocityMs),
            Radians.of(exitAngleRad), MetersPerSecond.of(rvMs), RadiansPerSecond.of(backspin));

        double nearRimX = distanceM - hoopRadius;
        boolean clearedLip = false;
        double prevX = shot.getX();
        double prevY = shot.getY();

        for (double t = 0; t < MAX_TIME; t += DT) {
            shot.step(DT);
            double x = shot.getX();
            double y = shot.getY();

            // Lip check: ball must be above hoopHeight + lipHeight at the near rim.
            if (!clearedLip && prevX < nearRimX && x >= nearRimX) {
                double frac = (nearRimX - prevX) / (x - prevX);
                double rimY = prevY + frac * (y - prevY);
                clearedLip = rimY >= hoopHeight + lipHeight;
            }

            // Detect the ball descending through the hoop plane (horizontal hoop).
            if (prevY > hoopHeight && y <= hoopHeight) {
                double frac = (hoopHeight - prevY) / (y - prevY);
                double crossX = prevX + frac * (x - prevX);
                if (clearedLip && Math.abs(crossX - distanceM) <= hoopRadius) {
                    return t + frac * DT;
                }
                return -1;
            }

            // past the far rim and below hoop height; missed
            if (x > distanceM + hoopRadius && y < hoopHeight) return -1;
            if (y < -0.5) return -1;

            prevX = x;
            prevY = y;
        }

        return -1;
    }

    /**
     * Returns true if a ball shot at the given exit conditions enters the hoop.
     *
     * @param distance horizontal distance to hoop center
     * @param radialVelocity robot radial velocity (positive = moving away from target)
     * @param exitVelocity ball exit speed
     * @param exitAngle ball exit angle (0 = horizontal, 90 deg = straight up)
     */
    public boolean isValidShot(Distance distance, LinearVelocity radialVelocity,
        LinearVelocity exitVelocity, Angle exitAngle) {
        return simulate(distance.in(Meters), radialVelocity.in(MetersPerSecond),
            exitVelocity.in(MetersPerSecond), exitAngle.in(Radians)) >= 0;
    }

    /**
     * Returns every (exitVelocity, exitAngle, tof) grid point that results in a valid shot.
     *
     * @param distance horizontal distance to hoop center
     * @param radialVelocity robot radial velocity (positive = moving away from target)
     * @param vMin minimum exit speed to search
     * @param vMax maximum exit speed to search
     * @param vSteps number of speed grid points (inclusive on both ends)
     * @param angleMin minimum exit angle to search
     * @param angleMax maximum exit angle to search
     * @param angleSteps number of angle grid points (inclusive on both ends)
     */
    public List<ShotParams> findValidShots(Distance distance, LinearVelocity radialVelocity,
        LinearVelocity vMin, LinearVelocity vMax, int vSteps, Angle angleMin, Angle angleMax,
        int angleSteps) {
        double distM = distance.in(Meters);
        double rvMs = radialVelocity.in(MetersPerSecond);
        double vMinMs = vMin.in(MetersPerSecond);
        double vMaxMs = vMax.in(MetersPerSecond);
        double angleMinRad = angleMin.in(Radians);
        double angleMaxRad = angleMax.in(Radians);

        List<ShotParams> valid = new ArrayList<>();
        for (int vi = 0; vi < vSteps; vi++) {
            double v = vMinMs + (vMaxMs - vMinMs) * vi / (vSteps - 1);
            for (int ai = 0; ai < angleSteps; ai++) {
                double angle = angleMinRad + (angleMaxRad - angleMinRad) * ai / (angleSteps - 1);
                double tof = simulate(distM, rvMs, v, angle);
                if (tof >= 0) {
                    valid.add(new ShotParams(MetersPerSecond.of(v), Radians.of(angle),
                        Seconds.of(tof)));
                }
            }
        }
        return valid;
    }

    /** The optimal shot and the semi-axes of the largest inscribed ellipse in the valid region. */
    public record OptimalResult(ShotParams shot, Angle angleSemiAxis, LinearVelocity speedSemiAxis) {
    }

    /**
     * From a list of valid shots, returns the shot whose centered, axis-aligned ellipse inscribed
     * in the valid (angle, speed) region has the largest area.
     *
     * <p>For each candidate center angle, the center speed is the midpoint of the valid speed range
     * at that angle. The algorithm then tries every discrete angle step as the ellipse semi-axis
     * {@code a}, and for each {@code a} computes the largest {@code b} (speed semi-axis) such that
     * the ellipse {@code ((α−α₀)/a)²+((v−v₀)/b)²≤1} is fully enclosed by the sampled valid region.
     * The pair {@code (a, b)} that maximises {@code a·b} is returned.
     *
     * @param validShots list of valid shots, typically from {@link #findValidShots}
     * @param distance horizontal distance used to simulate the optimal center shot for ToF
     * @param radialVelocity radial velocity used to simulate the optimal center shot for ToF
     * @return the optimal result, or empty if the list is empty
     */
    public Optional<OptimalResult> findOptimal(List<ShotParams> validShots, Distance distance,
        LinearVelocity radialVelocity) {
        if (validShots.isEmpty()) return Optional.empty();

        double distM = distance.in(Meters);
        double rvMs = radialVelocity.in(MetersPerSecond);

        // Group by angle (rad) → [vMin (m/s), vMax (m/s)]
        TreeMap<Double, double[]> byAngle = new TreeMap<>();
        for (ShotParams sp : validShots) {
            double angle = sp.exitAngle().in(Radians);
            double v = sp.exitVelocity().in(MetersPerSecond);
            double[] r = byAngle.computeIfAbsent(angle, k -> new double[]{v, v});
            r[0] = Math.min(r[0], v);
            r[1] = Math.max(r[1], v);
        }

        List<Double> angles = new ArrayList<>(byAngle.keySet());
        int N = angles.size();

        // Global hard limits: ellipse must not extend outside the overall valid region.
        double globalAngleMin = angles.get(0);
        double globalVMin = Double.MAX_VALUE, globalVMax = -Double.MAX_VALUE;
        for (double[] r : byAngle.values()) {
            globalVMin = Math.min(globalVMin, r[0]);
            globalVMax = Math.max(globalVMax, r[1]);
        }

        double bestArea = -1;
        OptimalResult best = null;

        for (int ci = 0; ci < N; ci++) {
            double alpha0 = angles.get(ci);
            double[] r0 = byAngle.get(alpha0);
            double v0 = (r0[0] + r0[1]) / 2.0;
            // b is bounded by the local speed range AND the global speed limits.
            double maxB = Math.min(v0 - globalVMin, globalVMax - v0);
            double centerB = Math.min((r0[1] - r0[0]) / 2.0, maxB);
            if (centerB <= 0) continue;

            // Try each possible angle semi-axis 'a', defined by extending to grid point ri.
            for (int ri = ci + 1; ri < N; ri++) {
                double a = angles.get(ri) - alpha0;

                // Hard limit: the left edge of the symmetric ellipse must stay within bounds.
                // Right edge is angles[ri] <= globalAngleMax by construction.
                if (alpha0 - a < globalAngleMin) break;

                // Compute the largest b that keeps the ellipse inside the valid region at all
                // interior grid points (strictly inside: |dalpha| < a).
                double b = centerB;
                boolean valid = true;

                for (int k = 0; k < N; k++) {
                    double dalpha = angles.get(k) - alpha0;
                    if (Math.abs(dalpha) >= a) continue; // at or outside the ellipse edge

                    double[] rk = byAngle.get(angles.get(k));
                    if (v0 < rk[0] || v0 > rk[1]) {
                        // Center speed is not in the valid range at this angle; extending further
                        // won't help since the same angle stays inside larger ellipses.
                        valid = false;
                        break;
                    }

                    double clearance = Math.min(v0 - rk[0], rk[1] - v0);
                    double sinTheta = Math.sqrt(1.0 - (dalpha / a) * (dalpha / a));
                    b = Math.min(b, clearance / sinTheta);
                }

                if (!valid) break;
                if (b <= 0) continue;

                double area = a * b;
                if (area > bestArea) {
                    bestArea = area;
                    double tof = simulate(distM, rvMs, v0, alpha0);
                    best = new OptimalResult(
                        new ShotParams(MetersPerSecond.of(v0), Radians.of(alpha0),
                            Seconds.of(Math.max(0, tof))),
                        Radians.of(a), MetersPerSecond.of(b));
                }
            }
        }

        return Optional.ofNullable(best);
    }
}
