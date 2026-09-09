package frc.robot.ballistics;

import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;

class Ballistics2D {

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @Test
    public void higherLaunchSpeedGivesLongerRange() {
        BallisticsDerivative2D model = new BallisticsDerivative2D(0.5, 0.5, 0.5, 9.81, 0.4);
        Rotation2d angle = Rotation2d.fromDegrees(30);

        double range1 = simulateRange(model, 10, angle);
        double range2 = simulateRange(model, 20, angle);

        assertTrue(range1 < range2);
    }

    private double simulateRange(BallisticsDerivative2D sim, double speed, Rotation2d angle) {
        double vx0 = speed * angle.getCos();
        double vy0 = speed * angle.getSin();

        double[] x = new double[] {0, 0.5, vx0, vy0};
        double h = 0.01;

        while (x[1] > 0.0) {
            Integrators.RK4.step(sim, x, h, x);
        }

        return x[0];
    }

}
