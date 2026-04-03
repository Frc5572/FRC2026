package frc.gen;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Optional;
import java.util.OptionalDouble;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.shotdata.SimulatedShot;

public class GenerateLUTs {

    private static final double dt = 0.001;

    public static void main(String[] argv) {

        double height = FieldConstants.Hub.innerHeight - Constants.Shooter.shooterHeight.in(Meters);

        NumberFormat formatter = new DecimalFormat("#0.00");
        var backspinMultiplier = RotationsPerSecond.of(30);

        try (FileWriter writer = new FileWriter("triples.txt")) {
            for (double radialSpeed = -2.0; radialSpeed <= 2.0; radialSpeed += 0.1) {
                System.out.println("Radial speed: " + radialSpeed);
                for (double distance = 0.5; distance < 8.0; distance += 0.1) {
                    for (double shotVelocity = 6.3; shotVelocity < 10.0; shotVelocity += 0.1) {
                        ShotParameters parameters =
                            new ShotParameters(MetersPerSecond.of(radialSpeed),
                                MetersPerSecond.of(shotVelocity), backspinMultiplier);
                        var optAngle = findValidAngle(parameters, distance, height);
                        if (optAngle.isPresent()) {
                            writer.write(radialSpeed + "\t" + distance + "\t" + shotVelocity + "\t"
                                + optAngle.get().in(Degrees) + "\n");
                        }
                    }
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static record ShotParameters(LinearVelocity radialVelocity, LinearVelocity exitVelocity,
        AngularVelocity backspinMultiplier) {
    };

    private static Optional<Angle> findValidAngle(ShotParameters params, double distance,
        double height) {
        double minAngle = 45;
        double maxAngle = 90;
        OptionalDouble maxDistance = distanceForAngle(params, Degrees.of(maxAngle), height);
        OptionalDouble minDistance = distanceForAngle(params, Degrees.of(minAngle), height);
        if (maxDistance.isEmpty() && minDistance.isEmpty()) {
            return Optional.empty();
        }
        for (int i = 0; i < 200; i++) {
            double midAngle = (maxAngle + minAngle) / 2.0;
            OptionalDouble midDistance = distanceForAngle(params, Degrees.of(midAngle), height);
            if (midDistance.isEmpty() || midDistance.getAsDouble() > distance) {
                minAngle = midAngle;
            } else {
                maxAngle = midAngle;
                maxDistance = midDistance;
            }
        }
        double angle = (maxAngle + minAngle) / 2.0;
        OptionalDouble finalDistance = distanceForAngle(params, Degrees.of(angle), height);
        if (finalDistance.isEmpty() || Math.abs(finalDistance.getAsDouble() - distance) > 0.1) {
            return Optional.empty();
        }
        return Optional.of(Degrees.of(angle));
    }

    private static OptionalDouble distanceForAngle(ShotParameters params, Angle angle,
        double height) {
        SimulatedShot simulatedShot = new SimulatedShot(angle, params.exitVelocity(),
            params.backspinMultiplier().times(Units.degreesToRadians(90 - angle.in(Degrees))));
        simulatedShot.state.a3 -= params.radialVelocity().in(MetersPerSecond);
        if (simulatedShot.state.a3 < 0.0) {
            return OptionalDouble.empty();
        }
        for (; simulatedShot.state.a4 > 0.0;) {
            simulatedShot.step(dt);
        }
        if (simulatedShot.state.a2 <= height) {
            return OptionalDouble.empty();
        }
        for (; simulatedShot.state.a2 > height;) {
            simulatedShot.step(dt);
        }
        return OptionalDouble.of(simulatedShot.state.a1);
    }

    private static InterpolatingDoubleTreeMap shotTrajectory(ShotParameters parameters,
        Angle exitAngle, double[] range) {
        InterpolatingDoubleTreeMap trajectory = new InterpolatingDoubleTreeMap();
        SimulatedShot simulatedShot =
            new SimulatedShot(exitAngle, parameters.exitVelocity(), parameters.backspinMultiplier()
                .times(Units.degreesToRadians(90 - exitAngle.in(Degrees))));
        simulatedShot.state.a1 -= parameters.radialVelocity().in(MetersPerSecond);
        for (; simulatedShot.state.a2 >= 0.0;) {
            trajectory.put(simulatedShot.state.a1, simulatedShot.state.a2);
            range[0] = Math.min(range[0], simulatedShot.state.a1);
            range[1] = Math.max(range[1], simulatedShot.state.a1);
            simulatedShot.step(dt);
        }
        return trajectory;
    }

    private static void writeCSV(File file, InterpolatingDoubleTreeMap[] trajectories,
        String[] titles, double[] range) {
        if (trajectories.length == 0) {
            return;
        }
        double dx = (range[1] - range[0]) / 150.0;
        double x = range[0];
        try (FileWriter writer = new FileWriter(file)) {
            writer.write("x");
            for (int i = 0; i < trajectories.length; i++) {
                writer.write("," + titles[i]);
            }
            writer.write("\n");
            for (; x < range[1]; x += dx) {
                writer.write("" + x);
                for (int i = 0; i < trajectories.length; i++) {
                    writer.write("," + trajectories[i].get(x));
                }
                writer.write("\n");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void writeGnuPlot(String title, File file,
        InterpolatingDoubleTreeMap[] trajectories, String[] titles, double[] range) {
        if (trajectories.length == 0) {
            return;
        }
        double dx = (range[1] - range[0]) / 150.0;
        double x = range[0];
        try (FileWriter writer = new FileWriter("temp.txt")) {
            for (; x < range[1]; x += dx) {
                writer.write("" + x);
                for (int i = 0; i < trajectories.length; i++) {
                    writer.write("\t" + trajectories[i].get(x));
                }
                writer.write("\n");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        StringBuilder gnuplotCommand =
            new StringBuilder("set term pngcairo; unset key; set title '" + title + "'; plot");
        for (int i = 0; i < titles.length; i++) {
            if (i != 0) {
                gnuplotCommand.append(",");
            }
            gnuplotCommand
                .append(" 'temp.txt' using 1:" + (i + 2) + " title '" + titles[i] + "' with lines");
        }
        try {
            ProcessBuilder pb = new ProcessBuilder("gnuplot", "-e", gnuplotCommand.toString());
            pb.redirectOutput(file);
            pb.redirectErrorStream(false);

            Process process = pb.start();

            process.waitFor();
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

}
