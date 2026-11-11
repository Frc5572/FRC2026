package frc.robot.math.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotationIntervalTests {
    @Test
    public void testComplement() {
        RotationInterval complement1 =
            new RotationInterval(new Rotation2d(1.0), new Rotation2d(2.0)).complement();
        assertEquals(2.0, complement1.getMin().getRadians());
        assertEquals(1.0, complement1.getMax().getRadians());

        RotationInterval complement =
            new RotationInterval(new Rotation2d(3.0), new Rotation2d(1.5)).complement();
        assertEquals(1.5, complement.getMin().getRadians());
        assertEquals(3.0, complement.getMax().getRadians());
    }
}
