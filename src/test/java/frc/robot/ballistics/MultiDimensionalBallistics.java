package frc.robot.ballistics;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

/**
 * The MultiDimensionalBallistics class is meant to compare the 2D values of both the Ballistics2D
 * and Ballistics3D arrays.
 * 
 * This test is plausable because it could happen due to a logical error within either of the
 * Ballistics2D or Ballistics3D array code where the calculations could be altered to the point of
 * completely different 2D values.
 * 
 * It matters if the 2D portions of the arrays aren't equal because our robot could have a distorted
 * value of our goal, and might have an unaccurate shot.
 */
public class MultiDimensionalBallistics {
    /**
     * This test is making sure the 2D values of Ballistics2D and Ballistics3D are in sync.
     */
    @Test
    public void multiDimensionalTest() {
        BallisticsDerivative2D derivatives2D = new BallisticsDerivative2D();
        BallisticsDerivative3D derivatives3D = new BallisticsDerivative3D();

        double[] derivatives2DResult = derivatives2D.derivative(new double[] {2.0, 2.0, 3.0, 5.0});
        double[] derivatives3DResult =
            derivatives3D.derivative(new double[] {2.0, 0, 2.0, 3.0, 0, 5.0});

        List<Double> list = Arrays.stream(derivatives3DResult).boxed().collect(Collectors.toList());

        list.remove(1);
        list.remove(3);

        derivatives3DResult = list.stream().mapToDouble(Double::doubleValue).toArray();

        assertArrayEquals(derivatives3DResult, derivatives2DResult, 0.01);
    }
}
