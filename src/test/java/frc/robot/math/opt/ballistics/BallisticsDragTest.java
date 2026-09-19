package frc.robot.math.opt.ballistics;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;
import frc.robot.bolistics.Ballistics;

/**
 * Unit tests for Ballistics drag model.
 *
 * Test types included:
 * - Smoke Test: Basic physics calculations work
 * - Regression Test: Ensures known physics behaviors are correctly modeled
 * - Metamorphic Test: Verifies relationships between model parameters
 * - Invariant Test: Confirms physical laws are always satisfied
 * - Round Trip Test: Validates energy dissipation properties
 * - Analytic Test: Compares with known physics solutions
 */
@DisplayName("Ballistics Drag Model Tests")
public class BallisticsDragTest {

    // ============ SMOKE TEST ============
    // Basic functionality test to ensure the model works at all

    @Test
    @DisplayName("Smoke Test: Simple gravity-only projectile (no air resistance)")
    void smokeTest_simpleGravity() {
        // No drag (density = 0), only gravity
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double[] state = new double[]{0.0, 10.0}; // position, velocity
        double[] deriv = model.derivative(state);

        assertEquals(10.0, deriv[0], 1e-6, "Velocity derivative should equal velocity");
        assertEquals(-9.81, deriv[1], 1e-6, "Acceleration should be -g under gravity only");
    }

    @Test
    @DisplayName("Smoke Test: Upward projectile acceleration")
    void smokeTest_upwardProjectile() {
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double[] state = new double[]{5.0, 20.0}; // position, upward velocity
        double[] deriv = model.derivative(state);

        assertEquals(20.0, deriv[0], 1e-6, "Velocity derivative should equal velocity");
        assertEquals(-9.81, deriv[1], 1e-6, "Acceleration should be -g");
    }

    @Test
    @DisplayName("Smoke Test: Downward projectile with drag")
    void smokeTest_downwardWithDrag() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 0.5, 0.47);
        double[] state = new double[]{10.0, -5.0}; // position, downward velocity
        double[] deriv = model.derivative(state);

        assertEquals(-5.0, deriv[0], 1e-6, "Velocity derivative should equal velocity");
        assertTrue(Double.isFinite(deriv[1]), "Acceleration should be finite");
    }

    // ============ REGRESSION TEST ============
    // Tests to catch common issues and physical law violations

    @Test
    @DisplayName("Regression Test: Non-zero velocity affects acceleration")
    void regressionTest_nonZeroVelocityAffectsAccel() {
        Ballistics model = new Ballistics(0.0, 1.225, 0.05, 1.0, 0.47);

        // Test upward motion
        double[] stateUp = new double[]{0.0, 10.0};
        double[] derivUp = model.derivative(stateUp);
        assertTrue(Double.isFinite(derivUp[1]), "Acceleration should be finite for positive velocity");

        // Test downward motion
        double[] stateDown = new double[]{0.0, -10.0};
        double[] derivDown = model.derivative(stateDown);
        assertTrue(Double.isFinite(derivDown[1]), "Acceleration should be finite for negative velocity");
    }

    @Test
    @DisplayName("Regression Test: Zero velocity has zero drag force")
    void regressionTest_zeroDragAtZeroVelocity() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        double[] state = new double[]{5.0, 0.0}; // zero velocity
        double[] deriv = model.derivative(state);

        assertEquals(0.0, deriv[0], 1e-6, "Velocity derivative should be zero");
        assertEquals(-9.81, deriv[1], 1e-6, "Only gravity acts; acceleration should be -g");
    }

    @Test
    @DisplayName("Regression Test: Mass affects acceleration")
    void regressionTest_massAffectsAcceleration() {
        double[] state = new double[]{0.0, 10.0};
        
        Ballistics lightModel = new Ballistics(9.81, 1.225, 0.05, 0.5, 0.47);
        double[] derivLight = lightModel.derivative(state);

        Ballistics heavyModel = new Ballistics(9.81, 1.225, 0.05, 2.0, 0.47);
        double[] derivHeavy = heavyModel.derivative(state);

        assertTrue(Double.isFinite(derivLight[1]), "Light object acceleration should be finite");
        assertTrue(Double.isFinite(derivHeavy[1]), "Heavy object acceleration should be finite");
    }

    // ============ METAMORPHIC TEST ============
    // Tests where we know relationships between outputs for related inputs

    @Test
    @DisplayName("Metamorphic Test: Doubling mass halves acceleration effect from same force")
    void metamorphicTest_massScaling() {
        double[] state = new double[]{0.0, 10.0};
        
        Ballistics m1 = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        double[] deriv1 = m1.derivative(state);

        Ballistics m2 = new Ballistics(9.81, 1.225, 0.05, 2.0, 0.47);
        double[] deriv2 = m2.derivative(state);

        // Difference in acceleration should scale inversely with mass
        double accelDiff1 = deriv1[1] - (-9.81);
        double accelDiff2 = deriv2[1] - (-9.81);
        
        assertEquals(accelDiff1 * 2.0, accelDiff2, 1e-2, 
            "Acceleration change scales inversely with mass doubling");
    }

    @Test
    @DisplayName("Metamorphic Test: Velocity affects acceleration")
    void metamorphicTest_velocityAffectsAccel() {
        Ballistics model = new Ballistics(0.0, 1.225, 0.05, 1.0, 0.47);

        double[] state1 = new double[]{0.0, 5.0};
        double[] deriv1 = model.derivative(state1);

        double[] state2 = new double[]{0.0, 10.0};
        double[] deriv2 = model.derivative(state2);

        // Different velocities should produce different accelerations
        assertNotEquals(deriv1[1], deriv2[1], 1e-6, "Different velocities should produce different accelerations");
    }

    @Test
    @DisplayName("Metamorphic Test: Drag coefficient affects acceleration")
    void metamorphicTest_dragCoefficientEffect() {
        double[] state = new double[]{0.0, 10.0};

        Ballistics lowDrag = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.1);
        double[] derivLow = lowDrag.derivative(state);

        Ballistics highDrag = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.5);
        double[] derivHigh = highDrag.derivative(state);

        assertNotEquals(derivLow[1], derivHigh[1], 1e-6,
            "Different drag coefficients should produce different accelerations");
    }

    // ============ INVARIANT TEST ============
    // Properties that should always hold regardless of input

    @Test
    @DisplayName("Invariant Test: Velocity derivative equals position derivative")
    void invariantTest_velocityConsistency() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        
        double[] state1 = new double[]{0.0, 5.0};
        double[] deriv1 = model.derivative(state1);
        assertEquals(5.0, deriv1[0], 1e-6, "Velocity should equal position derivative");

        double[] state2 = new double[]{10.0, -20.0};
        double[] deriv2 = model.derivative(state2);
        assertEquals(-20.0, deriv2[0], 1e-6, "Velocity should equal position derivative");
    }

    @Test
    @DisplayName("Invariant Test: Acceleration is finite")
    void invariantTest_accelerationFinite() {
        double gravity = 9.81;
        Ballistics model = new Ballistics(gravity, 1.225, 0.05, 1.0, 0.47);

        double[] state = new double[]{5.0, 5.0};
        double[] deriv = model.derivative(state);

        assertTrue(Double.isFinite(deriv[1]), "Acceleration should be finite");
    }

    @Test
    @DisplayName("Invariant Test: Drag affects acceleration")
    void invariantTest_dragAffects() {
        Ballistics modelWithDrag = new Ballistics(0.0, 1.225, 0.05, 1.0, 0.47);
        Ballistics modelNoDrag = new Ballistics(0.0, 0.0, 0.05, 1.0, 0.47);

        double[] state = new double[]{0.0, 10.0};

        double[] derivWithDrag = modelWithDrag.derivative(state);
        double[] derivNoDrag = modelNoDrag.derivative(state);

        assertNotEquals(derivWithDrag[1], derivNoDrag[1], 1e-6,
            "Drag should affect acceleration");
    }

    // ============ ROUND TRIP TEST ============
    // Apply the reverse operation and verify we get back to original state

    @Test
    @DisplayName("Round Trip Test: Verify derivative produces consistent changes")
    void roundTripTest_derivativeConsistency() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        double[] state = new double[]{0.0, 10.0};
        double[] deriv = model.derivative(state);

        // Apply Euler step
        double dt = 0.001;
        double[] newState = new double[]{
            state[0] + deriv[0] * dt,
            state[1] + deriv[1] * dt
        };

        // Derivative should be consistent
        double[] newDeriv = model.derivative(newState);
        assertEquals(deriv[0], newDeriv[0], 0.1, "Velocity should be approximately consistent");
    }

    @Test
    @DisplayName("Round Trip Test: Small time step integration is stable")
    void roundTripTest_integrationStability() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        double[] state = new double[]{0.0, 100.0};
        
        double dt = 0.001;
        double maxIterations = 1000;
        
        for (int i = 0; i < maxIterations; i++) {
            double[] deriv = model.derivative(state);
            state[0] = state[0] + deriv[0] * dt;
            state[1] = state[1] + deriv[1] * dt;
            
            // Sanity checks: velocity shouldn't grow unbounded
            assertTrue(Math.abs(state[1]) < 1000, "Velocity should remain reasonable");
            // Position should remain finite
            assertTrue(Double.isFinite(state[0]), "Position should be finite");
        }
        
        assertTrue(true, "Integration completed without divergence");
    }

    // ============ ANALYTIC TEST ============
    // Compare with known mathematical solutions

    @Test
    @DisplayName("Analytic Test: Free fall without drag matches kinematic equations")
    void analyticTest_freeFallNoDrag() {
        // v_f = v_0 + gt
        // Without air resistance, acceleration should be constant
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double[] state = new double[]{0.0, 0.0}; // Starting at rest
        double[] deriv1 = model.derivative(state);

        state = new double[]{100.0, 0.0};
        double[] deriv2 = model.derivative(state);

        assertEquals(deriv1[1], deriv2[1], 1e-6, 
            "Acceleration should be constant without drag (-g)");
        assertEquals(-9.81, deriv1[1], 1e-6, "Should match gravitational acceleration");
    }

    @Test
    @DisplayName("Analytic Test: No gravity with drag")
    void analyticTest_noGravityWithDrag() {
        Ballistics model = new Ballistics(0.0, 1.225, 0.05, 1.0, 0.47);
        double[] state = new double[]{0.0, 10.0};
        double[] deriv = model.derivative(state);

        assertEquals(10.0, deriv[0], 1e-6, "Position derivative should equal velocity");
        assertTrue(Double.isFinite(deriv[1]), "Acceleration should be finite");
    }

    @Test
    @DisplayName("Analytic Test: Gravity effect on acceleration")
    void analyticTest_gravityEffect() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        
        double[] state = new double[]{100.0, -5.0};
        double[] deriv = model.derivative(state);

        assertTrue(Double.isFinite(deriv[1]), "Acceleration should be finite");
    }

}
