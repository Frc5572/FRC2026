package frc.robot.subsystems.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.util.SwerveRateLimiter.Limits;

/**
 * Tests for {@link SwerveRateLimiter}. Geometry and speeds match the 2026 drivetrain, but the
 * limiter is constructed explicitly rather than from {@code Constants} so these run without HAL and
 * stay independent of tuning changes.
 */
public class SwerveRateLimiterTest {

    private static final double kWheelBase = 0.55372;
    private static final double kTrackWidth = 0.55118;
    private static final double kMaxSpeed = 7.0;
    private static final double kDt = 0.02;
    private static final double kEpsilon = 1e-9;

    private static final Translation2d[] kModules = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)};

    /** Worst-case module lever arm, ~0.3906 m. */
    private static final double kRadius = Math.hypot(kWheelBase / 2.0, kTrackWidth / 2.0);

    private static SwerveRateLimiter limiter(Limits limits) {
        return new SwerveRateLimiter(limits, kMaxSpeed, kDt, kModules);
    }

    private static ChassisSpeeds run(Limits limits, ChassisSpeeds current, ChassisSpeeds wanted) {
        ChassisSpeeds out = new ChassisSpeeds();
        limiter(limits).limit(current, wanted, out);
        return out;
    }

    // ---------------------------------------------------------------- traction

    /**
     * From rest the limiter must bound the step to exactly one period of the traction limit. The
     * previous implementation divided by a zero current speed, producing NaN, and every comparison
     * against NaN was false — so the one case where the driver asks for the most acceleration was
     * the one case that went unlimited.
     */
    @Test
    public void fromRestIsLimited() {
        var out = run(Limits.none().withTractionAccel(10.0), new ChassisSpeeds(),
            new ChassisSpeeds(kMaxSpeed, 0.0, 0.0));

        assertEquals(10.0 * kDt, out.vxMetersPerSecond, kEpsilon);
        assertEquals(0.0, out.vyMetersPerSecond, kEpsilon);
    }

    /** Available acceleration falls off linearly with speed: at 90% of max, 10% remains. */
    @Test
    public void tractionFalloffScalesWithSpeed() {
        var out = run(Limits.none().withTractionAccel(10.0), new ChassisSpeeds(6.3, 0.0, 0.0),
            new ChassisSpeeds(kMaxSpeed, 0.0, 0.0));

        assertEquals(6.3 + 1.0 * kDt, out.vxMetersPerSecond, kEpsilon);
    }

    /** At and above max speed the falloff clamps to zero — no further acceleration is allowed. */
    @Test
    public void tractionFalloffClampsAtMaxSpeed() {
        var limits = Limits.none().withTractionAccel(10.0);

        var atMax = run(limits, new ChassisSpeeds(kMaxSpeed, 0.0, 0.0),
            new ChassisSpeeds(10.0, 0.0, 0.0));
        assertEquals(kMaxSpeed, atMax.vxMetersPerSecond, kEpsilon);

        var aboveMax = run(limits, new ChassisSpeeds(8.0, 0.0, 0.0),
            new ChassisSpeeds(10.0, 0.0, 0.0));
        assertEquals(8.0, aboveMax.vxMetersPerSecond, kEpsilon);
    }

    /**
     * The traction stage bounds acceleration along the direction of travel only, so braking is not
     * affected by it. What bounds braking is the skid limit — see
     * {@link #skidLimitsDeceleration()}.
     */
    @Test
    public void decelerationIsNotTractionLimited() {
        var out = run(Limits.none().withTractionAccel(10.0), new ChassisSpeeds(5.0, 0.0, 0.0),
            new ChassisSpeeds());

        assertEquals(0.0, out.vxMetersPerSecond, kEpsilon);
    }

    /**
     * The falloff keys on the fastest module rather than the chassis centre. Under pure rotation the
     * chassis centre is stationary, so a chassis-speed formulation would report full available
     * acceleration while the modules are already spinning at 3.1 m/s.
     */
    @Test
    public void tractionFalloffUsesFastestModuleNotChassisCentre() {
        var out = run(Limits.none().withTractionAccel(10.0), new ChassisSpeeds(0.0, 0.0, 8.0),
            new ChassisSpeeds(100.0, 0.0, 8.0));

        double expectedFalloff = 1.0 - (8.0 * kRadius) / kMaxSpeed;
        assertEquals(10.0 * expectedFalloff * kDt, out.vxMetersPerSecond, kEpsilon);
        // Strictly less than what a chassis-speed falloff would have permitted.
        assertTrue(out.vxMetersPerSecond < 10.0 * kDt);
    }

    // -------------------------------------------------------------------- tip

    /**
     * Each tip clamp must produce that clamp's own limit. The previous implementation tested against
     * {@code forwardTiltLimit} but assigned {@code forwardLimit}, which could raise the commanded
     * acceleration above the request.
     */
    @Test
    public void tipClampsUseTheirOwnLimits() {
        var limits = Limits.none().withTipAccel(3.0, 4.0, 5.0, 6.0);

        assertEquals(3.0 * kDt,
            run(limits, new ChassisSpeeds(), new ChassisSpeeds(100.0, 0.0, 0.0)).vxMetersPerSecond,
            kEpsilon);
        assertEquals(-4.0 * kDt,
            run(limits, new ChassisSpeeds(), new ChassisSpeeds(-100.0, 0.0, 0.0)).vxMetersPerSecond,
            kEpsilon);
        assertEquals(5.0 * kDt,
            run(limits, new ChassisSpeeds(), new ChassisSpeeds(0.0, 100.0, 0.0)).vyMetersPerSecond,
            kEpsilon);
        assertEquals(-6.0 * kDt,
            run(limits, new ChassisSpeeds(), new ChassisSpeeds(0.0, -100.0, 0.0)).vyMetersPerSecond,
            kEpsilon);
    }

    // ---------------------------------------------------------------- angular

    /** Angular acceleration is clamped in both directions; omega cannot step from rest. */
    @Test
    public void angularAccelIsClamped() {
        var limits = Limits.none().withAngularAccel(20.0);

        assertEquals(20.0 * kDt, run(limits, new ChassisSpeeds(),
            new ChassisSpeeds(0.0, 0.0, 100.0)).omegaRadiansPerSecond, kEpsilon);
        assertEquals(-20.0 * kDt, run(limits, new ChassisSpeeds(),
            new ChassisSpeeds(0.0, 0.0, -100.0)).omegaRadiansPerSecond, kEpsilon);
    }

    // ------------------------------------------------------------------- skid

    /** Total translational demand is capped at the skid limit, preserving direction. */
    @Test
    public void skidCapsTotalMagnitude() {
        var out = run(Limits.none().withSkidAccel(8.0), new ChassisSpeeds(),
            new ChassisSpeeds(100.0, 100.0, 0.0));

        assertEquals(8.0 * kDt, Math.hypot(out.vxMetersPerSecond, out.vyMetersPerSecond), kEpsilon);
        // Direction preserved: the request was 45 degrees.
        assertEquals(out.vxMetersPerSecond, out.vyMetersPerSecond, kEpsilon);
    }

    /** Skid bounds braking, which the traction stage deliberately does not. */
    @Test
    public void skidLimitsDeceleration() {
        var out = run(Limits.none().withSkidAccel(10.0), new ChassisSpeeds(5.0, 0.0, 0.0),
            new ChassisSpeeds());

        assertEquals(5.0 - 10.0 * kDt, out.vxMetersPerSecond, kEpsilon);
    }

    /**
     * Rotation consumes the friction budget through the centripetal term, leaving less for
     * translation. At 2 rad/s that is 1.56 m/s&sup2; of a 10 m/s&sup2; budget.
     */
    @Test
    public void skidReservesCentripetalBudgetForRotation() {
        var out = run(Limits.none().withSkidAccel(10.0), new ChassisSpeeds(0.0, 0.0, 2.0),
            new ChassisSpeeds(100.0, 0.0, 2.0));

        double expectedAccel = 10.0 - 2.0 * 2.0 * kRadius;
        assertEquals(expectedAccel * kDt, out.vxMetersPerSecond, kEpsilon);
    }

    /**
     * Once the centripetal load alone exceeds the friction budget there is nothing left, so no
     * change in velocity is permitted at all. At 6 rad/s that is 14.1 m/s&sup2; against a
     * 10 m/s&sup2; budget — which is why the existing {@code maxAngularVelocity} constants deserve
     * a look during tuning.
     */
    @Test
    public void skidSuppressesTranslationWhenCentripetalExceedsBudget() {
        var current = new ChassisSpeeds(0.0, 0.0, 6.0);
        var out = run(Limits.none().withSkidAccel(10.0), current,
            new ChassisSpeeds(100.0, 0.0, 6.0));

        assertEquals(0.0, out.vxMetersPerSecond, kEpsilon);
        assertEquals(0.0, out.vyMetersPerSecond, kEpsilon);
        assertEquals(6.0, out.omegaRadiansPerSecond, kEpsilon);
    }

    /** Translation and rotation share one budget, and their requested ratio is preserved. */
    @Test
    public void skidSharesBudgetBetweenTranslationAndRotation() {
        var out = run(Limits.none().withSkidAccel(10.0), new ChassisSpeeds(),
            new ChassisSpeeds(4.0, 0.0, 10.0));

        double ax = out.vxMetersPerSecond / kDt;
        double alpha = out.omegaRadiansPerSecond / kDt;
        assertEquals(10.0, ax + Math.abs(alpha) * kRadius, 1e-9);
        // Requested ax:alpha was 200:500, i.e. 0.4.
        assertEquals(0.4, ax / alpha, 1e-9);
    }

    // ------------------------------------------------------- contract, safety

    /** With every limit disabled and a reachable request, the wanted velocity passes through. */
    @Test
    public void unlimitedRequestPassesThroughExactly() {
        var wanted = new ChassisSpeeds(1.05, 2.05, 3.05);
        var out = run(Limits.none(), new ChassisSpeeds(1.0, 2.0, 3.0), wanted);

        // Exact equality, not approximate: an unlimited request must not accumulate the
        // floating-point drift of a divide-by-dt/multiply-by-dt round trip.
        assertEquals(wanted.vxMetersPerSecond, out.vxMetersPerSecond);
        assertEquals(wanted.vyMetersPerSecond, out.vyMetersPerSecond);
        assertEquals(wanted.omegaRadiansPerSecond, out.omegaRadiansPerSecond);
    }

    /** A zero request that is already reachable settles at exactly zero, with no overshoot. */
    @Test
    public void zeroRequestSettlesExactlyAtZero() {
        var out = run(Limits.none().withSkidAccel(10.0), new ChassisSpeeds(0.1, 0.0, 0.0),
            new ChassisSpeeds());

        assertEquals(0.0, out.vxMetersPerSecond);
        assertEquals(0.0, out.vyMetersPerSecond);
        assertEquals(0.0, out.omegaRadiansPerSecond);
    }

    /**
     * Neither input may be modified. {@code driveRobotRelative} passes whatever its supplier
     * returns, which may be a command's own field or an object derived from a trajectory sample, so
     * writing through to it would corrupt state the limiter does not own.
     */
    @Test
    public void inputsAreNotModified() {
        var current = new ChassisSpeeds(1.0, 2.0, 3.0);
        var wanted = new ChassisSpeeds(100.0, -100.0, 50.0);
        var limits = Limits.none().withTractionAccel(10.0).withSkidAccel(12.0)
            .withAngularAccel(20.0).withTipAccel(5.0, 5.0, 5.0, 5.0);

        limiter(limits).limit(current, wanted, new ChassisSpeeds());

        assertEquals(1.0, current.vxMetersPerSecond);
        assertEquals(2.0, current.vyMetersPerSecond);
        assertEquals(3.0, current.omegaRadiansPerSecond);
        assertEquals(100.0, wanted.vxMetersPerSecond);
        assertEquals(-100.0, wanted.vyMetersPerSecond);
        assertEquals(50.0, wanted.omegaRadiansPerSecond);
    }

    /** The output buffer carries no state between calls, so reusing it is safe. */
    @Test
    public void outputBufferCarriesNoState() {
        var limits = Limits.none().withTractionAccel(10.0).withSkidAccel(12.0);
        var first = new ChassisSpeeds(3.0, 1.0, 0.5);
        var second = new ChassisSpeeds(0.0, -2.0, 1.0);
        var subject = limiter(limits);

        ChassisSpeeds reused = new ChassisSpeeds();
        subject.limit(new ChassisSpeeds(1.0, 0.0, 0.0), first, reused);
        subject.limit(new ChassisSpeeds(2.0, 0.0, 0.0), second, reused);

        ChassisSpeeds fresh = new ChassisSpeeds();
        subject.limit(new ChassisSpeeds(2.0, 0.0, 0.0), second, fresh);

        assertEquals(fresh.vxMetersPerSecond, reused.vxMetersPerSecond);
        assertEquals(fresh.vyMetersPerSecond, reused.vyMetersPerSecond);
        assertEquals(fresh.omegaRadiansPerSecond, reused.omegaRadiansPerSecond);
    }

    /** Non-finite inputs are sanitised, so no NaN can reach the modules. */
    @Test
    public void nonFiniteInputsProduceFiniteOutput() {
        var limits = Limits.none().withTractionAccel(10.0).withSkidAccel(12.0);
        var out = run(limits, new ChassisSpeeds(Double.NaN, 0.0, 0.0),
            new ChassisSpeeds(1.0, Double.POSITIVE_INFINITY, Double.NaN));

        assertTrue(Double.isFinite(out.vxMetersPerSecond));
        assertTrue(Double.isFinite(out.vyMetersPerSecond));
        assertTrue(Double.isFinite(out.omegaRadiansPerSecond));
    }

    // ------------------------------------------------------------ multi-cycle

    /**
     * Holding a full-forward request from rest must produce a smooth bounded ramp, not a step: every
     * cycle within the traction limit, monotonically increasing, never past max speed, and
     * converging close to it. This is the behaviour the driver actually feels, and the closest thing
     * to a sim drive that can be asserted deterministically.
     */
    @Test
    public void heldRequestRampsSmoothlyFromRest() {
        var subject = limiter(Limits.none().withTractionAccel(10.0));
        var wanted = new ChassisSpeeds(kMaxSpeed, 0.0, 0.0);
        var current = new ChassisSpeeds();
        var out = new ChassisSpeeds();
        double previous = 0.0;

        for (int cycle = 0; cycle < 500; cycle++) {
            subject.limit(current, wanted, out);
            double speed = out.vxMetersPerSecond;

            assertTrue(Double.isFinite(speed), "non-finite speed at cycle " + cycle);
            assertTrue(speed >= previous, "speed decreased at cycle " + cycle);
            assertTrue(speed <= kMaxSpeed + kEpsilon, "overshot max speed at cycle " + cycle);
            assertTrue(speed - previous <= 10.0 * kDt + kEpsilon,
                "step exceeded traction limit at cycle " + cycle);

            previous = speed;
            current.vxMetersPerSecond = out.vxMetersPerSecond;
            current.vyMetersPerSecond = out.vyMetersPerSecond;
            current.omegaRadiansPerSecond = out.omegaRadiansPerSecond;
        }

        // Asymptotic because of the falloff, so it approaches max speed without reaching it.
        assertTrue(previous > 0.99 * kMaxSpeed, "did not converge near max speed: " + previous);
    }

    /** Holding a zero request from speed decelerates within the skid limit and settles at zero. */
    @Test
    public void heldZeroRequestDeceleratesToRest() {
        var subject = limiter(Limits.none().withTractionAccel(10.0).withSkidAccel(12.0));
        var wanted = new ChassisSpeeds();
        var current = new ChassisSpeeds(kMaxSpeed, 0.0, 0.0);
        var out = new ChassisSpeeds();
        double previous = kMaxSpeed;

        for (int cycle = 0; cycle < 200; cycle++) {
            subject.limit(current, wanted, out);
            double speed = out.vxMetersPerSecond;

            assertTrue(speed <= previous + kEpsilon, "speed increased at cycle " + cycle);
            assertTrue(speed >= 0.0, "undershot rest at cycle " + cycle + ": " + speed);
            assertTrue(previous - speed <= 12.0 * kDt + kEpsilon,
                "step exceeded skid limit at cycle " + cycle);

            previous = speed;
            current.vxMetersPerSecond = out.vxMetersPerSecond;
        }

        assertEquals(0.0, previous);
    }

    // ------------------------------------------------------- contract, safety

    /** Nonsense configuration fails at construction rather than silently misbehaving on the field. */
    @Test
    public void invalidConfigurationIsRejected() {
        assertThrows(IllegalArgumentException.class,
            () -> limiter(Limits.none().withTractionAccel(-1.0)));
        assertThrows(IllegalArgumentException.class,
            () -> limiter(Limits.none().withSkidAccel(Double.NaN)));
        assertThrows(IllegalArgumentException.class,
            () -> new SwerveRateLimiter(Limits.none(), 0.0, kDt, kModules));
        assertThrows(IllegalArgumentException.class,
            () -> new SwerveRateLimiter(Limits.none(), kMaxSpeed, -kDt, kModules));
        assertThrows(IllegalArgumentException.class,
            () -> new SwerveRateLimiter(Limits.none(), kMaxSpeed, kDt, new Translation2d[0]));
    }
}
