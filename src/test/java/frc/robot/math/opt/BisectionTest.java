package frc.robot.math.opt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import java.util.function.DoubleUnaryOperator;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for Bisection root-finding algorithm.
 *
 * Test types included: - Smoke Test: Basic functionality with simple quadratic roots - Regression
 * Test: Ensures the algorithm converges correctly - Metamorphic Test: Verifies properties are
 * preserved under transformations - Invariant Test: Confirms output always satisfies expected
 * properties - Round Trip Test: Validates that the found root when applied to function is near zero
 * - Analytic Test: Compares results with known mathematical solutions
 */
@DisplayName("Bisection Root Solver Tests")
public class BisectionTest {

    // ============ SMOKE TEST ============
    // Basic functionality test to ensure the method works at all

    @Test
    @DisplayName("Smoke Test: Find root of simple linear function (f(x) = x - 5)")
    void smokeTest_simpleLinear() {
        double root = Bisection.bisection(x -> x - 5.0, 0.0, 10.0);
        assertEquals(5.0, root, 1e-2, "Should find root at x=5");
    }

    @Test
    @DisplayName("Smoke Test: Find root of simple quadratic (f(x) = x^2 - 4)")
    void smokeTest_simpleQuadratic() {
        // Finding root between 0 and 3 should give x=2
        double root = Bisection.bisection(x -> x * x - 4.0, 0.0, 3.0);
        assertEquals(2.0, root, 1e-2, "Should find positive root at x=2");
    }

    // ============ REGRESSION TEST ============
    // Tests to catch common issues and edge cases

    @Test
    @DisplayName("Regression Test: Negative root")
    void regressionTest_negativeRoot() {
        // Finding root between -3 and 0, should give x=-2
        double root = Bisection.bisection(x -> x * x - 4.0, -3.0, 0.0);
        assertEquals(-2.0, root, 1e-2, "Should find negative root at x=-2");
    }

    @Test
    @DisplayName("Regression Test: Root at interval boundary")
    void regressionTest_rootNearBoundary() {
        // Root very close to min boundary
        double root = Bisection.bisection(x -> x - 0.1, 0.0, 1.0);
        assertEquals(0.1, root, 1e-2, "Should find root near min boundary");
    }

    @Test
    @DisplayName("Regression Test: Root in narrow interval")
    void regressionTest_narrowInterval() {
        // Narrow search interval
        double root = Bisection.bisection(x -> x - 5.0, 4.99, 5.01);
        assertEquals(5.0, root, 1e-2, "Should find root in narrow interval");
    }

    // ============ METAMORPHIC TEST ============
    // Tests where we know relationships between outputs for related inputs

    @Test
    @DisplayName("Metamorphic Test: Negated function has same root")
    void metamorphicTest_negatedFunction() {
        // For f(x), if r is a root of f(x), then r is also a root of -f(x)
        // because f(r) = 0 implies -f(r) = 0
        double root1 = Bisection.bisection(x -> x - 5.0, 0.0, 10.0);
        double root2 = Bisection.bisection(x -> -(x - 5.0), 0.0, 10.0);

        assertEquals(root1, root2, 1e-2, "Root of -f(x) should be same as root of f(x)");
    }

    @Test
    @DisplayName("Metamorphic Test: Scaled function preserves root")
    void metamorphicTest_scaledFunction() {
        // For f(x), if r is a root of f(x), then r is also a root of k*f(x) for k≠0
        double root1 = Bisection.bisection(x -> x * x - 4.0, 0.0, 3.0);
        double root2 = Bisection.bisection(x -> 2.0 * (x * x - 4.0), 0.0, 3.0);
        double root3 = Bisection.bisection(x -> -0.5 * (x * x - 4.0), 0.0, 3.0);

        assertEquals(root1, root2, 1e-2, "Scaled function should have same root");
        assertEquals(root1, root3, 1e-2, "Scaled function should have same root");
    }

    @Test
    @DisplayName("Metamorphic Test: Composition preserves root location")
    void metamorphicTest_translatedFunction() {
        // For f(x), finding root of f(x-a) = root_original + a
        double root1 = Bisection.bisection(x -> x * x - 4.0, 0.0, 3.0); // root at 2
        double root2 = Bisection.bisection(x -> (x - 10.0) * (x - 10.0) - 4.0, 10.0, 13.0); // root
                                                                                            // at 12

        assertEquals(root2 - root1, 10.0, 1e-2,
            "Translated function root should be translated by same amount");
    }

    // ============ INVARIANT TEST ============
    // Properties that should always hold regardless of input

    @Test
    @DisplayName("Invariant Test: Result is within interval bounds")
    void invariantTest_resultInBounds() {
        double min = 1.0;
        double max = 10.0;
        double root = Bisection.bisection(x -> x - 5.0, min, max);

        assertTrue(min <= root && root <= max, "Root should always be within [min, max] interval");
    }

    @Test
    @DisplayName("Invariant Test: Function value at root is near zero")
    void invariantTest_functionValueNearZero() {
        double root = Bisection.bisection(x -> x * x - 4.0, 0.0, 3.0);
        double functionValue = root * root - 4.0;

        assertTrue(Math.abs(functionValue) < 1e-2, "Function value at root should be near zero");
    }

    @Test
    @DisplayName("Invariant Test: Result unchanged for multiple calls")
    void invariantTest_deterministic() {
        double root1 = Bisection.bisection(x -> Math.sin(x), 2.0, 4.0);
        double root2 = Bisection.bisection(x -> Math.sin(x), 2.0, 4.0);
        double root3 = Bisection.bisection(x -> Math.sin(x), 2.0, 4.0);

        assertEquals(root1, root2, "Results should be deterministic");
        assertEquals(root2, root3, "Results should be deterministic");
    }

    // ============ ROUND TRIP TEST ============
    // Apply the reverse operation and verify we get back to original state

    @Test
    @DisplayName("Round Trip Test: Applying function to root gives near-zero result")
    void roundTripTest_functionApplicationToRoot() {
        double root = Bisection.bisection(x -> x * x * x - 27.0, 0.0, 10.0);
        double functionValueAtRoot = root * root * root - 27.0;

        assertEquals(0.0, functionValueAtRoot, 1e-2, "f(root) should be approximately zero");
    }

    @Test
    @DisplayName("Round Trip Test: Verify multiple functions give reasonable roots")
    void roundTripTest_multipleRoots() {
        // Test various functions and verify they evaluate to near-zero at found roots
        testRoundTrip(x -> x - 3.0, 0.0, 5.0, 3.0);
        testRoundTrip(x -> x * x - 9.0, 0.0, 4.0, 3.0);
        testRoundTrip(x -> Math.exp(x) - Math.E, 0.0, 2.0, 1.0);
    }

    private void testRoundTrip(DoubleUnaryOperator function, double min, double max,
        double expectedApprox) {
        double root = Bisection.bisection(function, min, max);
        double functionValue = function.applyAsDouble(root);

        assertEquals(0.0, functionValue, 1e-2,
            "f(root) should be approximately zero for all functions");
        assertEquals(expectedApprox, root, 0.1, "Root should be approximately the expected value");
    }

    // ============ ANALYTIC TEST ============
    // Compare with known mathematical solutions

    @Test
    @DisplayName("Analytic Test: Quadratic root matches formula")
    void analyticTest_quadraticFormula() {
        // For x^2 - 4 = 0, analytical root is x = 2
        double root = Bisection.bisection(x -> x * x - 4.0, 0.0, 3.0);
        double analytical = 2.0;

        assertEquals(analytical, root, 1e-2,
            "Bisection should match analytical solution for quadratic");
    }

    @Test
    @DisplayName("Analytic Test: Cubic root matches expected value")
    void analyticTest_cubicRoot() {
        // For x^3 - 8 = 0, analytical root is x = 2
        double root = Bisection.bisection(x -> x * x * x - 8.0, 0.0, 3.0);
        double analytical = 2.0;

        assertEquals(analytical, root, 1e-2,
            "Bisection should match analytical solution for cubic");
    }

    @Test
    @DisplayName("Analytic Test: Sine function root matches known value")
    void analyticTest_trigonometricRoot() {
        // sin(x) has a root at x = π
        double root = Bisection.bisection(x -> Math.sin(x), 2.0, 4.0);
        double analytical = Math.PI;

        assertEquals(analytical, root, 1e-2, "Bisection should find π as root of sin(x)");
    }

    @Test
    @DisplayName("Analytic Test: Exponential minus 1 finds ln(e)")
    void analyticTest_exponentialRoot() {
        // e^x - e = 0 has root at x = 1
        double root = Bisection.bisection(x -> Math.exp(x) - Math.E, 0.0, 2.0);
        double analytical = 1.0;

        assertEquals(analytical, root, 1e-2, "Bisection should find x=1 as root of e^x - e");
    }

}
