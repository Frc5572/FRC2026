package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Tests for the acceleration-limit failure detectors. */
public class LimitDetectorsTest {

    private static final double EPS = 1e-9;

    private static final Translation2d[] SQUARE = {
        new Translation2d(0.28, 0.28), new Translation2d(0.28, -0.28),
        new Translation2d(-0.28, 0.28), new Translation2d(-0.28, -0.28)};

    private static SwerveModuleState[] straight(double... speeds) {
        SwerveModuleState[] out = new SwerveModuleState[speeds.length];
        for (int i = 0; i < speeds.length; i++) {
            out[i] = new SwerveModuleState(speeds[i], Rotation2d.kZero);
        }
        return out;
    }

    /** Follow error only reports a shortfall, never a surplus. */
    @Test
    public void followErrorIsOneSided() {
        assertEquals(0.5, LimitDetectors.followError(3.0, 2.5), EPS);
        assertEquals(0.0, LimitDetectors.followError(2.0, 2.5), EPS,
            "running faster than commanded is not a follow failure");
    }

    /** Tilt combines pitch and roll. */
    @Test
    public void tiltCombinesBothAxes() {
        assertEquals(0.0, LimitDetectors.tiltDegrees(Rotation2d.kZero, Rotation2d.kZero), EPS);
        assertEquals(5.0,
            LimitDetectors.tiltDegrees(Rotation2d.fromDegrees(5), Rotation2d.kZero), 1e-9);
        assertEquals(5.0,
            LimitDetectors.tiltDegrees(Rotation2d.fromDegrees(3), Rotation2d.fromDegrees(4)),
            1e-9);
    }

    /** Pure translation with every wheel gripping gives a ratio of exactly one. */
    @Test
    public void grippingTranslationGivesUnityRatio() {
        assertEquals(1.0, LimitDetectors.skidRatio(straight(3, 3, 3, 3), SQUARE, 0.0), 1e-9);
    }

    /**
     * Rotation must be subtracted out, or spinning in place would look like a total skid: the
     * modules genuinely do move at different velocities in the chassis frame.
     */
    @Test
    public void rotationAloneIsNotSkid() {
        double omega = 2.0;
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            double rx = -omega * SQUARE[i].getY();
            double ry = omega * SQUARE[i].getX();
            states[i] = new SwerveModuleState(Math.hypot(rx, ry),
                new Rotation2d(Math.atan2(ry, rx)));
        }
        assertEquals(1.0, LimitDetectors.skidRatio(states, SQUARE, omega), 1e-6);
    }

    /** Translating while rotating is still not a skid when the wheels grip. */
    @Test
    public void combinedMotionIsNotSkid() {
        double omega = 1.5;
        double vx = 2.0;
        double vy = 0.5;
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            double mx = vx - omega * SQUARE[i].getY();
            double my = vy + omega * SQUARE[i].getX();
            states[i] = new SwerveModuleState(Math.hypot(mx, my),
                new Rotation2d(Math.atan2(my, mx)));
        }
        assertEquals(1.0, LimitDetectors.skidRatio(states, SQUARE, omega), 1e-6);
    }

    /** One wheel spinning faster than the rest raises the ratio. */
    @Test
    public void oneSpinningWheelIsDetected() {
        double ratio = LimitDetectors.skidRatio(straight(3, 3, 4.5, 3), SQUARE, 0.0);
        assertEquals(1.5, ratio, 1e-6);
        assertTrue(ratio > 1.2, "a 50% faster module must read as a skid");
    }

    /**
     * Two modules skidding must still be caught. Comparing against the mean would let the
     * skidders drag the reference up and hide themselves; the median does not move.
     */
    @Test
    public void medianResistsMultipleSkidders() {
        double ratio = LimitDetectors.skidRatio(straight(3, 3, 5, 5), SQUARE, 0.0);
        assertTrue(ratio > 1.2, "two skidding modules should still register, got " + ratio);
    }

    /** At a crawl the ratio is encoder noise, so it reports clean rather than alarming. */
    @Test
    public void ignoresNoiseAtStandstill() {
        assertEquals(1.0, LimitDetectors.skidRatio(straight(0.01, 0.0, 0.03, 0.0), SQUARE, 0.0),
            EPS);
    }

    /** Fewer than three modules cannot support a median comparison. */
    @Test
    public void degenerateInputIsSafe() {
        assertEquals(1.0, LimitDetectors.skidRatio(straight(1, 2), SQUARE, 0.0), EPS);
    }

}
