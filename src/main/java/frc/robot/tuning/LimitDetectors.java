package frc.robot.tuning;

import java.util.Arrays;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The failure conditions the three acceleration-limit procedures are tuned against.
 *
 * <p>
 * 1690's procedures (software-sessions.md, lines 157&ndash;176) are each phrased as "change the
 * limit until X happens": raise the forward limit until the robot cannot follow the wanted
 * velocity, raise the tilt limit until it starts tipping, lower the skid limit until it stops
 * skidding. Each X is measurable, so this class turns all three into numbers rather than leaving
 * them to the eye of whoever is watching.
 *
 * <p>
 * Every method is static and takes its inputs explicitly, so the detectors can be tested without
 * a robot.
 */
@NullMarked
public final class LimitDetectors {

    private LimitDetectors() {}

    /**
     * How far the measured chassis speed lags the commanded one.
     *
     * <p>
     * This is the forward-limit failure condition: once the limit lets the command ask for more
     * acceleration than the drivetrain can deliver, the measured speed falls behind and this
     * grows. Below the limit it stays near zero.
     *
     * @param commandedSpeed commanded translational speed, m/s
     * @param measuredSpeed measured translational speed, m/s
     * @return the shortfall in m/s, never negative
     */
    public static double followError(double commandedSpeed, double measuredSpeed) {
        return Math.max(0.0, commandedSpeed - measuredSpeed);
    }

    /**
     * How far the chassis is off level.
     *
     * <p>
     * This is the tilt-limit failure condition. A robot that is about to tip lifts its wheels on
     * one side first, so pitch and roll rise well before anything dramatic happens &mdash; which
     * is what makes it safe to stop a ramp on this rather than on the tipping itself.
     *
     * @param pitch chassis pitch
     * @param roll chassis roll
     * @return the combined tilt from level, in degrees
     */
    public static double tiltDegrees(Rotation2d pitch, Rotation2d roll) {
        return Math.hypot(pitch.getDegrees(), roll.getDegrees());
    }

    /**
     * How unevenly the modules are translating, as a max-to-median ratio.
     *
     * <p>
     * This is 1690's own skid test (lines 275&ndash;289). Subtract each module's share of the
     * chassis rotation from its measured velocity and what remains is that module's contribution
     * to translation. With every wheel gripping, those contributions are identical whatever the
     * robot is doing, so the ratio sits at 1.0. A module that breaks traction reads faster than
     * the rest and pushes the ratio up.
     *
     * <p>
     * Comparing against the median rather than the mean matters: a skidding module drags the mean
     * up with it, which would mask exactly the case being detected.
     *
     * @param states measured module states
     * @param translations module positions relative to the chassis centre
     * @param omegaRadPerSec measured chassis angular velocity
     * @return the ratio, 1.0 when every module agrees; 1.0 when the robot is essentially still
     */
    public static double skidRatio(SwerveModuleState[] states, Translation2d[] translations,
        double omegaRadPerSec) {
        int n = Math.min(states.length, translations.length);
        if (n < 3) {
            return 1.0;
        }
        double[] translational = new double[n];
        for (int i = 0; i < n; i++) {
            Rotation2d angle = states[i].angle;
            double vx = states[i].speedMetersPerSecond * angle.getCos();
            double vy = states[i].speedMetersPerSecond * angle.getSin();
            // The velocity this module would have from rotation alone: omega x r.
            double rx = -omegaRadPerSec * translations[i].getY();
            double ry = omegaRadPerSec * translations[i].getX();
            translational[i] = Math.hypot(vx - rx, vy - ry);
        }
        double[] sorted = translational.clone();
        Arrays.sort(sorted);
        double median = sorted.length % 2 == 0
            ? (sorted[sorted.length / 2 - 1] + sorted[sorted.length / 2]) / 2.0
            : sorted[sorted.length / 2];
        double max = sorted[sorted.length - 1];
        // Below a crawl the ratio is dominated by encoder noise and means nothing.
        if (median < 0.15) {
            return 1.0;
        }
        return max / median;
    }

}
