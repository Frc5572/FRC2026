package frc.robot.math.geometry;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import java.util.List;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The ShapeTest class is an analytic test used to check if the calculations for the different
 * shapes in the geometry package works.
 * 
 * This is plausable because someone could make a shape class, but not actually calculate the
 * dimensions correctly.
 * 
 * The consequences for this is if the geometry is invalid, the areas the robot needs to calculate
 * may be unaccurate.
 */
public class HexagonTest {
    /**
     * The hexagon method makes sure the verticies are in the correct coordinates.
     */
    @Test
    public void hexagon() {
        DataLogManager.start();
        Hexagon testHexagon =
            new Hexagon("Test Hex", new Translation2d(0, 0), 5, new Rotation2d(0));
        List<Translation2d> hexVerticies = List.of(testHexagon.getVertices());
        double[][] expectedVerticies = {{5.0, 0.0}, {2.5, 4.33}, {-2.5, 4.33}, {-5.0, 0.0},
            {-2.5, -4.33}, {2.5, -4.33}, {5.0, 0.0}};
        assertTrue(checkExpectedVerticies(expectedVerticies, hexVerticies));

        // Checks if hexagon correctly returns false if x and y values aren't within the hexagon
        assertTrue(testHexagon.contains(new Translation2d(3, 3)));

        // Checks if hexagon correctly returns false if x and y values aren't within the hexagon
        assertFalse(testHexagon.contains(new Translation2d(5, 5)));
    }

    /**
     * The hexagonRotation method goes through an array of expected verticies and actual verticies
     * to see if they line up with each other.
     */
    @Test
    public void hexagonRotation() {
        DataLogManager.start();
        Hexagon testHexagon =
            new Hexagon("Test Hex", new Translation2d(0, 0), 5, new Rotation2d(30));
        List<Translation2d> hexVerticies = List.of(testHexagon.getVertices());
        double[][] expectedVerticies = {{0.77, -4.94}, {4.66, -1.8}, {3.89, 3.14}, {-0.77, 4.94},
            {-4.66, 1.8}, {-3.89, -3.14}, {0.77, -4.94}};
        assertTrue(checkExpectedVerticies(expectedVerticies, hexVerticies));
    }

    /*
     * The overloaded methods checkExpectedVerticies go through two arrays, one predetermined
     * expected, one actual and returns false if at least one verticie doesn't match.
     */
    @SuppressWarnings("unused")
    private boolean checkExpectedVerticies(double[][] expectedVerticies,
        double[][] actualVerticies) {
        boolean isEqual = true;
        for (int i = 0; i < expectedVerticies.length; i++) {
            double x = Math.round(actualVerticies[i][0] * 100.0) / 100.0;
            double y = Math.round(actualVerticies[i][1] * 100.0) / 100.0;
            if (!(x == expectedVerticies[i][0] && y == expectedVerticies[i][1])) {
                DataLogManager.log("ERROR: " + i + "(" + x + ", " + y + ")");
                isEqual = false;
            } else {
                DataLogManager.log(i + "(" + x + ", " + y + ")");
            }
        }
        return isEqual;
    }

    private boolean checkExpectedVerticies(double[][] expectedVerticies,
        List<Translation2d> actualVerticies) {
        boolean isEqual = true;
        for (int i = 0; i < expectedVerticies.length; i++) {
            double x = Math.round(actualVerticies.get(i).getX() * 100.0) / 100.0;
            double y = Math.round(actualVerticies.get(i).getY() * 100.0) / 100.0;
            if (!(x == expectedVerticies[i][0] && y == expectedVerticies[i][1])) {
                DataLogManager.log("ERROR: " + i + "(" + x + ", " + y + ")");
                isEqual = false;
            } else {
                DataLogManager.log(i + "(" + x + ", " + y + ")");
            }
        }
        return isEqual;
    }
}
