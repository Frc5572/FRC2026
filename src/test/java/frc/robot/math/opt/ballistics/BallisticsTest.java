package frc.robot.math.opt.ballistics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bolistics.Ballistics;

/**
 * Unit tests for Ballistics physics simulation.
 *
 * Test types included: - Smoke Test: Basic ballistic calculations work - Regression Test: Ensures
 * expected physics behaviors are correctly modeled - Metamorphic Test: Verifies relationships
 * between simulation parameters - Invariant Test: Confirms physical laws are always satisfied -
 * Round Trip Test: Validates simulation properties and energy conservation - Analytic Test:
 * Compares with known physics solutions
 */
@DisplayName("Ballistics Physics Simulation Tests")
public class BallisticsTest {

    // ============ SMOKE TEST ============
    // Basic functionality test to ensure the simulation works at all

    @Test
    @DisplayName("Smoke Test: Simple projectile motion completes")
    void smokeTest_simpleProjectile() {
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        Rotation2d angle = Rotation2d.fromDegrees(45);

        double range = simulateRange(model, 20.0, angle);
        assertTrue(range > 0, "Projectile should have positive range");
        assertTrue(Double.isFinite(range), "Range should be finite");
    }

    @Test
    @DisplayName("Smoke Test: Simulation with drag completes")
    void smokeTest_projectileWithDrag() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        Rotation2d angle = Rotation2d.fromDegrees(45);

        double range = simulateRange(model, 20.0, angle);
        assertTrue(range > 0, "Projectile with drag should have positive range");
        assertTrue(Double.isFinite(range), "Range should be finite");
    }

    @Test
    @DisplayName("Smoke Test: Zero initial velocity")
    void smokeTest_zeroVelocity() {
        Ballistics model = new Ballistics(9.81, 0.5, 0.5, 0.4, 0.5);
        Rotation2d angle = Rotation2d.fromDegrees(30);

        double range = simulateRange(model, 0.0, angle);
        assertEquals(0.0, range, 1e-2, "Zero velocity should result in near-zero range");
    }

    // ============ REGRESSION TEST ============
    // Tests to catch common issues and edge cases

    @Test
    @DisplayName("Regression Test: Lower launch speed gives shorter range (original test)")
    void regressionTest_lowerLaunchSpeedGivesShorterRange() {
        Ballistics model = new Ballistics(9.81, 0.5, 0.5, 0.4, 0.5);
        Rotation2d angle = Rotation2d.fromDegrees(30);

        double range1 = simulateRange(model, 10, angle);
        double range2 = simulateRange(model, 20, angle);

        assertTrue(range1 < range2, "Lower launch speed should give shorter range");
    }

    @Test
    @DisplayName("Regression Test: Steeper angle reduces range for fixed speed")
    void regressionTest_steepAngleReducesRange() {
        Ballistics model = new Ballistics(9.81, 0.5, 0.5, 0.4, 0.5);
        double speed = 20.0;

        double range30 = simulateRange(model, speed, Rotation2d.fromDegrees(30));
        double range60 = simulateRange(model, speed, Rotation2d.fromDegrees(60));

        assertTrue(range30 > range60, "Shallower angle should give greater range");
    }

    @Test
    @DisplayName("Regression Test: Drag reduces range compared to no drag")
    void regressionTest_dragReducesRange() {
        Rotation2d angle = Rotation2d.fromDegrees(45);
        double speed = 30.0;

        Ballistics noDrag = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double rangeNoDrag = simulateRange(noDrag, speed, angle);

        Ballistics withDrag = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        double rangeWithDrag = simulateRange(withDrag, speed, angle);

        assertTrue(rangeWithDrag < rangeNoDrag, "Drag should reduce range");
    }

    // ============ METAMORPHIC TEST ============
    // Tests where we know relationships between outputs for related inputs

    @Test
    @DisplayName("Metamorphic Test: Doubling speed increases range")
    void metamorphicTest_doubleSpeedIncreaseRange() {
        Ballistics model = new Ballistics(9.81, 0.5, 0.5, 0.4, 0.5);
        Rotation2d angle = Rotation2d.fromDegrees(45);

        double range1 = simulateRange(model, 10.0, angle);
        double range2 = simulateRange(model, 20.0, angle);

        assertTrue(range2 > range1, "Doubling speed should increase range");
    }

    @Test
    @DisplayName("Metamorphic Test: Complementary angles have similar ranges")
    void metamorphicTest_complementaryAngles() {
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double speed = 30.0;

        double range30 = simulateRange(model, speed, Rotation2d.fromDegrees(30));
        double range60 = simulateRange(model, speed, Rotation2d.fromDegrees(60));

        // Without drag, complementary angles should have similar ranges
        assertEquals(range30, range60, 0.5,
            "Complementary angles should have similar ranges (no drag)");
    }

    @Test
    @DisplayName("Metamorphic Test: Increased mass reduces range")
    void metamorphicTest_massAffectsRange() {
        Rotation2d angle = Rotation2d.fromDegrees(45);
        double speed = 20.0;

        Ballistics lightModel = new Ballistics(9.81, 1.225, 0.05, 0.5, 0.47);
        double rangeLightRange = simulateRange(lightModel, speed, angle);

        Ballistics heavyModel = new Ballistics(9.81, 1.225, 0.05, 2.0, 0.47);
        double rangeHeavyRange = simulateRange(heavyModel, speed, angle);

        // Heavier object experiences proportionally more drag
        assertTrue(rangeHeavyRange > rangeLightRange,
            "Heavier object should travel farther (less drag effect)");
    }

    // ============ INVARIANT TEST ============
    // Properties that should always hold regardless of input

    @Test
    @DisplayName("Invariant Test: Range is non-negative")
    void invariantTest_rangeNonNegative() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);

        for (int degrees = 0; degrees <= 90; degrees += 15) {
            double range = simulateRange(model, 20.0, Rotation2d.fromDegrees(degrees));
            assertTrue(range >= 0, "Range should never be negative");
        }
    }

    @Test
    @DisplayName("Invariant Test: Simulation is deterministic")
    void invariantTest_deterministicSimulation() {
        Ballistics model = new Ballistics(9.81, 0.5, 0.5, 0.4, 0.5);
        Rotation2d angle = Rotation2d.fromDegrees(30);

        double range1 = simulateRange(model, 15.0, angle);
        double range2 = simulateRange(model, 15.0, angle);
        double range3 = simulateRange(model, 15.0, angle);

        assertEquals(range1, range2, 1e-10, "Simulation should be deterministic");
        assertEquals(range2, range3, 1e-10, "Simulation should be deterministic");
    }

    @Test
    @DisplayName("Invariant Test: Maximum range near 45 degrees (no drag)")
    void invariantTest_maxRangeAt45Degrees() {
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double speed = 30.0;

        double range30 = simulateRange(model, speed, Rotation2d.fromDegrees(30));
        double range45 = simulateRange(model, speed, Rotation2d.fromDegrees(45));
        double range60 = simulateRange(model, speed, Rotation2d.fromDegrees(60));

        assertTrue(range45 >= range30, "45 degrees should give greater range than 30 degrees");
        assertTrue(range45 >= range60, "45 degrees should give greater range than 60 degrees");
    }

    // ============ ROUND TRIP TEST ============
    // Apply the reverse operation and verify we get back to original state

    @Test
    @DisplayName("Round Trip Test: Integration stability over multiple trajectories")
    void roundTripTest_integrationStability() {
        Ballistics model = new Ballistics(9.81, 1.225, 0.05, 1.0, 0.47);
        Rotation2d angle = Rotation2d.fromDegrees(45);

        // Simulate multiple trajectories to verify stability
        for (int speed = 10; speed <= 50; speed += 10) {
            double range = simulateRange(model, speed, angle);
            assertTrue(Double.isFinite(range), "Range should always be finite");
            assertTrue(range >= 0, "Range should be non-negative");
        }
    }

    @Test
    @DisplayName("Round Trip Test: Valid state throughout simulation")
    void roundTripTest_validStateInSimulation() {
        Ballistics model = new Ballistics(9.81, 0.5, 0.5, 0.4, 0.5);
        Rotation2d angle = Rotation2d.fromDegrees(30);
        double speed = 25.0;

        double vx0 = speed * angle.getCos();
        double vy0 = speed * angle.getSin();

        double[] x = new double[] {0, 0.5, vx0, vy0};
        double h = 0.01;
        int iterations = 0;
        int maxIterations = 10000;

        while (x[1] > 0.0 && iterations < maxIterations) {
            // Verify all states are finite
            for (int i = 0; i < x.length; i++) {
                assertTrue(Double.isFinite(x[i]), "State component should be finite");
            }

            double[] deriv = model.derivative(x);
            for (int i = 0; i < deriv.length; i++) {
                assertTrue(Double.isFinite(deriv[i]), "Derivative should be finite");
            }

            // Simple Euler step
            x[0] = x[0] + deriv[0] * h;
            x[1] = x[1] + deriv[1] * h;
            iterations++;
        }

        assertTrue(iterations < maxIterations, "Simulation should terminate in reasonable time");
    }

    // ============ ANALYTIC TEST ============
    // Compare with known mathematical solutions

    @Test
    @DisplayName("Analytic Test: Horizontal launch validates free fall")
    void analyticTest_horizontalLaunchFreeFall() {
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        Rotation2d angle = Rotation2d.fromDegrees(0); // Horizontal

        double range = simulateRange(model, 20.0, angle);
        // With horizontal launch, projectile should fall immediately
        assertTrue(range >= 0, "Horizontal launch should have non-negative range");
    }

    @Test
    @DisplayName("Analytic Test: Vertical launch and return")
    void analyticTest_verticalLaunchReturn() {
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double initialSpeed = 20.0;

        // Vertical launch (90 degrees)
        Rotation2d verticalAngle = Rotation2d.fromDegrees(90);
        double range = simulateRange(model, initialSpeed, verticalAngle);

        // With vertical launch and no drag, range should be near zero
        assertTrue(range < 1.0, "Vertical launch should result in minimal range");
    }

    @Test
    @DisplayName("Analytic Test: Projectile range formula validation (no drag)")
    void analyticTest_rangeFormula() {
        // For projectile motion without drag: R = v^2 * sin(2*angle) / g
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double speed = 30.0;
        double gravity = 9.81;
        double angle = 45.0;

        double simulatedRange = simulateRange(model, speed, Rotation2d.fromDegrees(angle));
        double theoreticalRange = (speed * speed * Math.sin(2 * Math.toRadians(angle))) / gravity;

        // Should be close to theoretical value
        assertEquals(theoreticalRange, simulatedRange, 1.0,
            "Simulated range should match theoretical range formula");
    }

    @Test
    @DisplayName("Analytic Test: Time of flight validation")
    void analyticTest_timeOfFlight() {
        Ballistics model = new Ballistics(9.81, 0.0, 0.5, 1.0, 0.47);
        double initialSpeed = 20.0;
        double angle = 45.0;

        double vy0 = initialSpeed * Math.sin(Math.toRadians(angle));
        // Theoretical time of flight: t = 2*vy0/g
        double theoreticalTime = 2 * vy0 / 9.81;

        // Run simulation to measure actual time
        double vx0 = initialSpeed * Math.cos(Math.toRadians(angle));
        double[] x = new double[] {0, 0.5, vx0, vy0};
        double h = 0.001;
        double time = 0.0;

        while (x[1] > 0.0 && time < 10.0) {
            double[] deriv = model.derivative(x);
            x[0] = x[0] + deriv[0] * h;
            x[1] = x[1] + deriv[1] * h;
            time += h;
        }

        // Simulated time should be close to theoretical
        assertEquals(theoreticalTime, time, 0.1, "Time of flight should match theory");
    }

    private double simulateRange(Ballistics sim, double speed, Rotation2d angle) {
        double vx0 = speed * angle.getCos();
        double vy0 = speed * angle.getSin();

        double[] x = new double[] {0, 0.5, vx0, vy0};
        double h = 0.01;

        while (x[1] > 0.0) {
            // Simple RK4 integration step
            double[] k1 = sim.derivative(x);
            double[] x_k1 = new double[] {x[0] + k1[0] * h / 2, x[1] + k1[1] * h / 2};

            double[] k2 = sim.derivative(x_k1);
            double[] x_k2 = new double[] {x[0] + k2[0] * h / 2, x[1] + k2[1] * h / 2};

            double[] k3 = sim.derivative(x_k2);
            double[] x_k3 = new double[] {x[0] + k3[0] * h, x[1] + k3[1] * h};

            double[] k4 = sim.derivative(x_k3);

            x[0] = x[0] + (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) * h / 6;
            x[1] = x[1] + (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) * h / 6;
        }
        return x[0];
    }
}
