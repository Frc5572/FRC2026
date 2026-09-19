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
 * This is plausable because we need to make sure that whenever the Hexagon class is instantiated,
 * it initializes correctly, as well as having the correct verticie positions when the hexagon
 * rotates.
 * 
 * The consequences for this is if the geometry is invalid, the areas the robot needs to calculate
 * may be unaccurate.
 */
public class HexagonTest {
    /**
     * The hexagon method makes sure the verticies are in the correct coordinates. Intended to be an
     * invariant test.
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

        // Checks if hexagon correctly returns if a point isn't within the hexagon
        assertTrue(testHexagon.contains(new Translation2d(3, 3)));

        // Checks if hexagon correctly returns false point isn't within the hexagon
        assertFalse(testHexagon.contains(new Translation2d(5, 5)));
    }

    /**
     * The hexagonRotation method goes through an array of expected verticies and actual verticies
     * to see if they line up with each other.
     * 
     * This also makes sure that the value is in radians, not degrees. Intended to be an invariant
     * test.
     */
    @Test
    public void hexagonRotation() {
        DataLogManager.start();
        Rotation2d testRot = new Rotation2d(30);
        Hexagon testHexagon = new Hexagon("Test Hex", new Translation2d(0, 0), 5, testRot);
        List<Translation2d> hexVerticies = List.of(testHexagon.getVertices());
        double[][] expectedVerticies = {{0.77, -4.94}, {4.66, -1.8}, {3.89, 3.14}, {-0.77, 4.94},
            {-4.66, 1.8}, {-3.89, -3.14}, {0.77, -4.94}};
        assertTrue(checkExpectedVerticies(expectedVerticies, hexVerticies));
    }

    /*
     * The overloaded methods checkExpectedVerticies go through two arrays, one expected, and one
     * actual. Returns false if at least one verticie doesn't match.
     */

    /**
     * @param expectedVerticies sets what the verticies that are expected from the result of
     *        Hexagon.getVerticies()
     * @param actualVerticies the actual verticies from the hexagon if it is returned as a 2D array
     * @return
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

    /**
     * @param expectedVerticies sets what the verticies that are expected from the result
     *        ofHexagon.getVerticies()
     * @param actualVerticies the actual verticies from the hexagon if it is returned as a List<>
     * @return
     */
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
