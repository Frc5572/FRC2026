package frc.robot.ballistics;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

public class MultiDBallistics {
    @Test
    public void multiDTest() {
        BallisticsDerivative2D derivatives2D = new BallisticsDerivative2D();
        BallisticsDerivative3D derivatives3D = new BallisticsDerivative3D();

        double[] derivatives2DResult = derivatives2D.derivative(new double[] {2.0, 2.0, 3.0, 5.0});
        double[] derivatives3DResult =
            derivatives3D.derivative(new double[] {2.0, 0, 2.0, 3.0, 0, 5.0});

        List<Double> list = Arrays.stream(derivatives3DResult).boxed().collect(Collectors.toList());

        list.remove(1);
        list.remove(4);

        derivatives3DResult = list.stream().mapToDouble(Double::doubleValue).toArray();

        assertArrayEquals(derivatives3DResult, derivatives2DResult, 0.1);
    }
}
