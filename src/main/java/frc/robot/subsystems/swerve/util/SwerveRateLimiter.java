package frc.robot.subsystems.swerve.util;

import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

/**
 * Applies acceleration rate limiting to robot-relative swerve chassis commands so that commanded
 * motion is physically achievable, stable, and predictable.
 *
 * <p>
 * The scheme follows Team 1690's swerve control methodology: derive the wanted acceleration from the
 * velocity delta, pass it through a series of limits, then re-integrate over one control period.
 * Limits are applied in this order:
 * <ol>
 * <li><b>Traction:</b> caps acceleration along the direction of travel. Motors produce less torque
 * the faster they spin, so the available acceleration falls off linearly with speed. The falloff is
 * keyed on the <i>fastest module</i> rather than the chassis centre, because that module saturates
 * first when translating and rotating at the same time.</li>
 * <li><b>Tip:</b> clamps the robot-frame acceleration components independently to keep the robot
 * from tipping over its wheelbase. Translational only — rotation about the vertical axis produces no
 * horizontal CG acceleration when the CG sits at the rotation centre.</li>
 * <li><b>Angular:</b> clamps angular acceleration, so commanded omega cannot step.</li>
 * <li><b>Skid:</b> caps the total tire demand to keep the wheels from breaking traction. Module
 * acceleration is {@code a + alpha x r_i - omega^2 r_i}, so translation and rotation share one
 * friction budget and the centripetal term is reserved out of it before the rest is divided up. A
 * spin at 4&nbsp;rad/s already consumes roughly 6.3&nbsp;m/s&sup2; of that budget on this
 * drivetrain.</li>
 * </ol>
 *
 * <p>
 * The limiter is stateless: the caller supplies both the current and the wanted velocity, and the
 * result is written into a caller-owned {@code out} buffer so that the steady-state path allocates
 * nothing. Neither input is modified.
 *
 * <p>
 * All acceleration limits are in m/s&sup2; except the angular limit, which is in rad/s&sup2;. A
 * limit of {@link Double#POSITIVE_INFINITY} disables that stage.
 *
 * <p>
 * Based on concepts presented by FRC Team 1690:
 * <a href="https://www.youtube.com/watch?v=vUtVXz7ebEE">
 * https://www.youtube.com/watch?v=vUtVXz7ebEE</a>
 */
@NullMarked
public class SwerveRateLimiter {

    /** Speeds below this are treated as stationary when picking a direction of travel. */
    private static final double kSpeedEpsilon = 1e-6;

    /**
     * Acceleration limits for a {@link SwerveRateLimiter}. Translational and skid limits are in
     * m/s&sup2;, {@code angularAccel} is in rad/s&sup2;. Any limit may be
     * {@link Double#POSITIVE_INFINITY} to disable it.
     */
    public record Limits(double tractionAccel, double tipForwardAccel, double tipBackAccel,
        double tipLeftAccel, double tipRightAccel, double skidAccel, double angularAccel) {

        /** Every limit disabled — {@link #limit} becomes a pass-through. */
        public static Limits none() {
            double inf = Double.POSITIVE_INFINITY;
            return new Limits(inf, inf, inf, inf, inf, inf, inf);
        }

        /** The limits configured in {@link Constants.Swerve}. */
        public static Limits fromConstants() {
            return new Limits(Constants.Swerve.tractionAccelLimit,
                Constants.Swerve.tipForwardAccelLimit, Constants.Swerve.tipBackAccelLimit,
                Constants.Swerve.tipLeftAccelLimit, Constants.Swerve.tipRightAccelLimit,
                Constants.Swerve.skidAccelLimit, Constants.Swerve.angularAccelLimit);
        }

        /**
         * Returns a copy with a different traction limit.
         *
         * @param value traction acceleration limit in m/s&sup2;
         * @return a new {@code Limits}
         */
        public Limits withTractionAccel(double value) {
            return new Limits(value, tipForwardAccel, tipBackAccel, tipLeftAccel, tipRightAccel,
                skidAccel, angularAccel);
        }

        /**
         * Returns a copy with different tip limits.
         *
         * @param forward forward tip limit in m/s&sup2;
         * @param back backward tip limit in m/s&sup2;
         * @param left leftward tip limit in m/s&sup2;
         * @param right rightward tip limit in m/s&sup2;
         * @return a new {@code Limits}
         */
        public Limits withTipAccel(double forward, double back, double left, double right) {
            return new Limits(tractionAccel, forward, back, left, right, skidAccel, angularAccel);
        }

        /**
         * Returns a copy with a different skid limit.
         *
         * @param value skid acceleration limit in m/s&sup2;
         * @return a new {@code Limits}
         */
        public Limits withSkidAccel(double value) {
            return new Limits(tractionAccel, tipForwardAccel, tipBackAccel, tipLeftAccel,
                tipRightAccel, value, angularAccel);
        }

        /**
         * Returns a copy with a different angular limit.
         *
         * @param value angular acceleration limit in rad/s&sup2;
         * @return a new {@code Limits}
         */
        public Limits withAngularAccel(double value) {
            return new Limits(tractionAccel, tipForwardAccel, tipBackAccel, tipLeftAccel,
                tipRightAccel, skidAccel, value);
        }
    }

    private final double tractionAccelLimit;
    private final double tipForwardAccelLimit;
    private final double tipBackAccelLimit;
    private final double tipLeftAccelLimit;
    private final double tipRightAccelLimit;
    private final double skidAccelLimit;
    private final double angularAccelLimit;

    private final double maxSpeed;
    private final double dt;

    /** Module x offsets from the chassis centre, robot frame, metres. */
    private final double[] moduleX;
    /** Module y offsets from the chassis centre, robot frame, metres. */
    private final double[] moduleY;
    /** Largest module offset magnitude, metres — the worst-case lever arm. */
    private final double moduleRadius;

    /** Creates a rate limiter configured from {@link Constants}. */
    public SwerveRateLimiter() {
        this(Limits.fromConstants(), Constants.Swerve.maxSpeed, Constants.loopPeriodSecs,
            Constants.Swerve.swerveTranslations);
    }

    /**
     * Creates a rate limiter with explicit limits and the drivetrain geometry from
     * {@link Constants}.
     *
     * @param limits the acceleration limits to enforce
     */
    public SwerveRateLimiter(Limits limits) {
        this(limits, Constants.Swerve.maxSpeed, Constants.loopPeriodSecs,
            Constants.Swerve.swerveTranslations);
    }

    /**
     * Creates a rate limiter with explicit limits, geometry, and control period.
     *
     * @param limits the acceleration limits to enforce
     * @param maxSpeed maximum achievable module speed in m/s, used for the traction falloff
     * @param dt control loop period in seconds
     * @param moduleTranslations module offsets from the chassis centre in the robot frame
     * @throws IllegalArgumentException if any limit is negative or NaN, if {@code maxSpeed} or
     *         {@code dt} is not positive and finite, or if {@code moduleTranslations} is empty
     */
    public SwerveRateLimiter(Limits limits, double maxSpeed, double dt,
        Translation2d[] moduleTranslations) {
        this.tractionAccelLimit = requireLimit(limits.tractionAccel(), "tractionAccel");
        this.tipForwardAccelLimit = requireLimit(limits.tipForwardAccel(), "tipForwardAccel");
        this.tipBackAccelLimit = requireLimit(limits.tipBackAccel(), "tipBackAccel");
        this.tipLeftAccelLimit = requireLimit(limits.tipLeftAccel(), "tipLeftAccel");
        this.tipRightAccelLimit = requireLimit(limits.tipRightAccel(), "tipRightAccel");
        this.skidAccelLimit = requireLimit(limits.skidAccel(), "skidAccel");
        this.angularAccelLimit = requireLimit(limits.angularAccel(), "angularAccel");
        this.maxSpeed = requirePositiveFinite(maxSpeed, "maxSpeed");
        this.dt = requirePositiveFinite(dt, "dt");

        if (moduleTranslations.length == 0) {
            throw new IllegalArgumentException("moduleTranslations must not be empty");
        }
        this.moduleX = new double[moduleTranslations.length];
        this.moduleY = new double[moduleTranslations.length];
        double radius = 0.0;
        for (int i = 0; i < moduleTranslations.length; i++) {
            this.moduleX[i] = moduleTranslations[i].getX();
            this.moduleY[i] = moduleTranslations[i].getY();
            radius = Math.max(radius, moduleTranslations[i].getNorm());
        }
        this.moduleRadius = radius;
    }

    private static double requireLimit(double value, String name) {
        if (Double.isNaN(value) || value < 0.0) {
            throw new IllegalArgumentException(
                name + " must be non-negative and not NaN, got " + value);
        }
        return value;
    }

    private static double requirePositiveFinite(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                name + " must be positive and finite, got " + value);
        }
        return value;
    }

    /** Replaces a non-finite value with zero so no NaN can reach the modules. */
    private static double sanitize(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    /**
     * Limits a wanted robot-relative chassis velocity to one reachable from the current velocity
     * within the configured acceleration, tip, angular, and skid constraints.
     *
     * <p>
     * Writes the result into {@code out}; neither {@code current} nor {@code wanted} is modified.
     * Allocates nothing. Non-finite components in either input are treated as zero, so a bad pose
     * estimate upstream cannot put a NaN on the CAN bus.
     *
     * @param current the measured robot-relative chassis speeds
     * @param wanted the desired robot-relative chassis speeds
     * @param out buffer receiving the limited robot-relative chassis speeds; may not alias either
     *        input
     */
    public void limit(ChassisSpeeds current, ChassisSpeeds wanted, ChassisSpeeds out) {
        final double cvx = sanitize(current.vxMetersPerSecond);
        final double cvy = sanitize(current.vyMetersPerSecond);
        final double cw = sanitize(current.omegaRadiansPerSecond);
        final double wvx = sanitize(wanted.vxMetersPerSecond);
        final double wvy = sanitize(wanted.vyMetersPerSecond);
        final double ww = sanitize(wanted.omegaRadiansPerSecond);

        double ax = (wvx - cvx) / dt;
        double ay = (wvy - cvy) / dt;
        double alpha = (ww - cw) / dt;
        boolean limited = false;

        // Stage 1: traction. Available acceleration falls off with module speed, and the cap
        // applies along the direction of travel — perpendicular demand is a friction question,
        // which the skid stage owns.
        double maxModuleSpeed = 0.0;
        for (int i = 0; i < moduleX.length; i++) {
            maxModuleSpeed =
                Math.max(maxModuleSpeed, Math.hypot(cvx - cw * moduleY[i], cvy + cw * moduleX[i]));
        }
        double tractionFalloff = Math.max(1.0 - maxModuleSpeed / maxSpeed, 0.0);
        // Guarded rather than a plain product: with the limit disabled and the falloff at zero,
        // INFINITY * 0.0 is NaN, and every subsequent comparison against it silently succeeds.
        double maxTractionAccel = Double.isInfinite(tractionAccelLimit) ? Double.POSITIVE_INFINITY
            : tractionAccelLimit * tractionFalloff;

        // Direction of travel. When stopped there is no direction of travel, so fall back to the
        // direction we are being asked to accelerate in — this is what makes a from-rest command
        // limited rather than skipped.
        double dirX = 0.0;
        double dirY = 0.0;
        double currentSpeed = Math.hypot(cvx, cvy);
        if (currentSpeed > kSpeedEpsilon) {
            dirX = cvx / currentSpeed;
            dirY = cvy / currentSpeed;
        } else {
            double accelMagnitude = Math.hypot(ax, ay);
            if (accelMagnitude > kSpeedEpsilon) {
                dirX = ax / accelMagnitude;
                dirY = ay / accelMagnitude;
            }
        }

        double alongMotionAccel = ax * dirX + ay * dirY;
        if (alongMotionAccel > maxTractionAccel) {
            double excess = alongMotionAccel - maxTractionAccel;
            ax -= excess * dirX;
            ay -= excess * dirY;
            limited = true;
        }

        // Stage 2: tip. Independent clamps per robot-frame axis.
        if (ax > tipForwardAccelLimit) {
            ax = tipForwardAccelLimit;
            limited = true;
        } else if (ax < -tipBackAccelLimit) {
            ax = -tipBackAccelLimit;
            limited = true;
        }
        if (ay > tipLeftAccelLimit) {
            ay = tipLeftAccelLimit;
            limited = true;
        } else if (ay < -tipRightAccelLimit) {
            ay = -tipRightAccelLimit;
            limited = true;
        }

        // Stage 3: angular. Hard clamp before the shared budget below.
        if (alpha > angularAccelLimit) {
            alpha = angularAccelLimit;
            limited = true;
        } else if (alpha < -angularAccelLimit) {
            alpha = -angularAccelLimit;
            limited = true;
        }

        // Stage 4: skid. One friction budget shared by translation and rotation, with the
        // centripetal term reserved out of it first. |a + alpha x r| <= |a| + |alpha| * r, so
        // bounding the sum is conservative but never optimistic.
        double centripetalAccel = cw * cw * moduleRadius;
        double availableAccel = Math.max(skidAccelLimit - centripetalAccel, 0.0);
        double demandedAccel = Math.hypot(ax, ay) + Math.abs(alpha) * moduleRadius;
        if (demandedAccel > availableAccel) {
            double scale = availableAccel / demandedAccel;
            ax *= scale;
            ay *= scale;
            alpha *= scale;
            limited = true;
        }

        if (limited) {
            out.vxMetersPerSecond = cvx + ax * dt;
            out.vyMetersPerSecond = cvy + ay * dt;
            out.omegaRadiansPerSecond = cw + alpha * dt;
        } else {
            // Nothing bound, so the wanted velocity is already reachable. Assign it directly
            // rather than round-tripping through the acceleration, which would introduce
            // floating-point drift on every cycle.
            out.vxMetersPerSecond = wvx;
            out.vyMetersPerSecond = wvy;
            out.omegaRadiansPerSecond = ww;
        }
    }
}
