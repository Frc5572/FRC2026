package frc.gen;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.function.DoubleUnaryOperator;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import edu.wpi.first.math.util.Units;
import frc.robot.math.opt.Bisection;
import frc.robot.shotdata.ShotData;

public class GenerateLUTs {

    public static void main(String[] argv) {
        NumberFormat formatter = new DecimalFormat("#0.00");

        double slipRatio = 0.0;
        for (var entry : ShotData.entries) {
            slipRatio += entry.theoreticalExitVelocity().in(MetersPerSecond)
                / entry.noSlipExitVelocity().in(MetersPerSecond);
        }
        slipRatio /= ShotData.entries.length;

        OptionalDouble[][] data = new OptionalDouble[20][];
        double[] distances = new double[20];
        double[] flywheelSpeeds = new double[20];
        for (int i = 0; i < 20; i++) {
            distances[i] = Units.feetToMeters(2) + Units.feetToMeters(1) * i;
            flywheelSpeeds[i] = (45.0 + (95.0 - 45.0) / 20.0 * i);
        }
        int numValid = 0;
        for (int i = 0; i < 20; i++) {
            double distance = distances[i];
            data[i] = new OptionalDouble[20];
            for (int j = 0; j < 20; j++) {
                double flywheelSpeed = flywheelSpeeds[j];
                double exitSpeed =
                    Units.rotationsToRadians(flywheelSpeed) * Units.inchesToMeters(2) * slipRatio;
                data[i][j] = solveForAngle(distance, exitSpeed);
                if (data[i][j].isPresent()) {
                    numValid++;
                }
            }
        }
        DMatrixRMaj a = new DMatrixRMaj(numValid, 6);
        DMatrixRMaj b = new DMatrixRMaj(numValid, 1);
        int idx = 0;
        for (int i = 0; i < 20; i++) {
            double distance = distances[i];
            for (int j = 0; j < 20; j++) {
                double flywheelSpeed = flywheelSpeeds[j];
                if (data[i][j].isPresent()) {
                    a.set(idx, 0, 1.0);
                    a.set(idx, 1, distance);
                    a.set(idx, 2, flywheelSpeed);
                    a.set(idx, 3, distance * flywheelSpeed);
                    a.set(idx, 4, distance * distance);
                    a.set(idx, 5, flywheelSpeed * flywheelSpeed);
                    b.set(idx, 90 - Units.radiansToDegrees(data[i][j].getAsDouble()) - 12.695);
                    idx++;
                }
            }
        }

        DMatrixRMaj temp1 = new DMatrixRMaj(numValid, numValid);
        DMatrixRMaj temp2 = new DMatrixRMaj(numValid, numValid);
        DMatrixRMaj res = new DMatrixRMaj(6, 1);
        CommonOps_DDRM.multTransA(a, a, temp1);
        CommonOps_DDRM.invert(temp1, temp2);
        CommonOps_DDRM.multTransB(temp2, a, temp1);
        CommonOps_DDRM.mult(temp1, b, res);

        System.out.println(Arrays.toString(res.getData()));

        try (FileWriter writer = new FileWriter(new File("test.txt"))) {
            for (int i = 0; i < 20; i++) {
                writer.write('\t');
                writer.write(formatter.format(flywheelSpeeds[i]));
            }
            writer.write('\n');
            for (int i = 0; i < 20; i++) {
                writer.write(formatter.format(Units.metersToFeet(distances[i])));
                for (int j = 0; j < 20; j++) {
                    writer.write('\t');
                    if (data[i][j].isPresent()) {
                        writer.write(formatter.format(
                            90 - Units.radiansToDegrees(data[i][j].getAsDouble()) - 12.695));
                    }
                }
                writer.write('\n');
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static OptionalDouble solveForAngle(double distance, double exitSpeed) {
        double z = ShotData.shooterToTargetHeightDiff.in(Meters);
        double v0 = exitSpeed;
        double d = distance;
        double g = 9.81;
        System.out.println("z = " + z);
        System.out.println("v0 = " + v0);
        System.out.println("d = " + d);
        System.out.println("g = " + g);
        DoubleUnaryOperator y = (t) -> -z + v0 * Math.sin(t) * d / v0 / Math.cos(t)
            - 0.5 * g * Math.pow(d / v0 / Math.cos(t), 2.0);
        System.out.println("y(pi/4) = " + y.applyAsDouble(Math.PI / 4.0));
        System.out.println("y(77.4 deg) = " + y.applyAsDouble(Units.degreesToRadians(90 - 12.695)));
        if (y.applyAsDouble(Math.PI / 4.0) < 0.0) {
            return OptionalDouble.empty();
        }
        if (y.applyAsDouble(Units.degreesToRadians(90 - 12.695)) > -1e-3) {
            return OptionalDouble.empty();
        }
        var x = Bisection.bisection(y, Math.PI / 4.0, Units.degreesToRadians(90 - 12.695));
        return OptionalDouble.of(x);
    }

}
