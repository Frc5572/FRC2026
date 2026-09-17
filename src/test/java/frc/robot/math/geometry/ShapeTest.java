package frc.robot.math.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
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
public class ShapeTest {
    /**
     * The hexagon method makes sure the verticies are in the correct coordinates
     */
    @Test
    public void hexagon() {
        DataLogManager.start();
        Hexagon testHexagon =
            new Hexagon("Test Hex", new Translation2d(0, 0), 5, new Rotation2d(0));
        List<Translation2d> hexVerticies = List.of(testHexagon.getVertices());
        DataLogManager.log("Results: " + hexVerticies.toString());
        // assertArrayEquals(new Translation2d[] {new Translation2d(5.00, 0.00),
        // new Translation2d(2.50, 4.33), new Translation2d(-2.50, 4.33),
        // new Translation2d(-5.00, 0.00), new Translation2d(-2.50, -4.33),
        // new Translation2d(2.50, -4.33), new Translation2d(5.00, 0.00)}, hexVerticies.toArray());
        assertTrue(testHexagon.contains(new Translation2d(3, 3)));
        assertFalse(testHexagon.contains(new Translation2d(5, 5)));
    }

    @Test
    public void hexagonRotation() {
        DataLogManager.start();
        Hexagon testHexagon =
            new Hexagon("Test Hex", new Translation2d(0, 0), 5, new Rotation2d(30));
        Translation2d hexVerticie = testHexagon.getVertices()[0];
        DataLogManager.log("X: " + hexVerticie.getX() + "Y: " + hexVerticie.getY());
        assertEquals(4.33012701892, hexVerticie.getX());
        assertEquals(2.50, hexVerticie.getY());
    }
}
