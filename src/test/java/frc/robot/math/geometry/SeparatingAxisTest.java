package frc.robot.math.geometry;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SeparatingAxisTest {
    @Test
    public void solveAxis() {
        Circle testCircle = new Circle("Test Circle", new Translation2d(0, 0), 5);
        Hexagon testHexagon1 =
            new Hexagon("Test Hexagon 1", new Translation2d(0, 0), 5, new Rotation2d(0));
        Hexagon testHexagon2 =
            new Hexagon("Test Hexagon 2", new Translation2d(5, 5), 5, new Rotation2d(0));
        Hexagon testHexagon3 =
            new Hexagon("Test Hexagon 3", new Translation2d(10, 10), 5, new Rotation2d(0));
        assertTrue(
            SeparatingAxis.solve(testHexagon1, testHexagon2, new Penetration("Test Penetration")));
        assertTrue(
            SeparatingAxis.solve(testHexagon1, testCircle, new Penetration("Test Penetration")));
        assertFalse(
            SeparatingAxis.solve(testHexagon1, testHexagon3, new Penetration("Test Penetration")));
    }
}
