package frc.robot.math.geometry;

import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Translation2d;

public class CircleTest {
    @Test
    public void circleChange() {
        Circle testCircle = new Circle("Test", new Translation2d(0, 0), 10);
        assertTrue(testCircle.contains(new Translation2d(7, 7)));
    }
}
