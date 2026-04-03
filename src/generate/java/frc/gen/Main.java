package frc.gen;

import java.util.Arrays;
import edu.wpi.first.math.util.Units;
import frc.robot.math.opt.BFGS;
import frc.robot.math.opt.DiffFunc;
import frc.robot.math.opt.FiniteDifference;

public class Main {

    public static void main(String[] argv) {
        exitAngle();
    }

    private static record ExitAngleData(double measuredAngleDeg, double distanceInches) {
    };

    public static void exitAngle() {
        double flywheelSpeedRps = 60.0;
        ExitAngleData[] data = new ExitAngleData[] {
            // @formatter:off
            new ExitAngleData(0.0, 24.0), 
            new ExitAngleData(5.0, 76.0),
            // @formatter:on
        };

        double[] initialGuess = new double[] {flywheelSpeedRps * 0.05, 0.0, 1.0, 0.0};

        DiffFunc func = new FiniteDifference((x) -> {
            double v = x[0];
            double a = x[1];
            double b = x[2];
            double c = x[3];

            double loss = 0.0;

            for (var entry : data) {
                double distanceMeters = Units.inchesToMeters(entry.distanceInches);
                double guessTheta = Math.toRadians(
                    a * Math.pow(entry.measuredAngleDeg, 2.0) + b * entry.measuredAngleDeg + c);
                double guessDistanceMeters =
                    2 * Math.pow(v, 2) * Math.cos(guessTheta) * Math.sin(guessTheta) / 9.81;
                loss += Math.pow(distanceMeters - guessDistanceMeters, 2.0);
            }

            return 0.5 * loss;
        }, 1e-6);

        System.out.println(Arrays.toString(func.gradient(initialGuess)));

        BFGS bfgs = new BFGS();
        bfgs.optimize(func, initialGuess);

        System.out.println(bfgs);
    }

}
