package frc.gen;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.shotdata.ShotData;
import frc.robot.shotdata.ShotData.ShotEntry;
import frc.robot.shotdata.ShotEntrySetWriter;
import frc.robot.util.binrw.BinaryData;

/** Generate lookup tables */
public class GenerateLUTs {

    /**
     * Entrypoint for generateLUTs gradle task
     *
     * @throws IOException
     */
    public static void main(String[] argv) throws IOException {
        Distance hubHeight =
            Meters.of(FieldConstants.Hub.height).minus(Constants.Shooter.shooterHeight);
        Distance hubRadius = Meters.of(FieldConstants.Hub.width / 2.0);
        Distance lip = Centimeters.of(15).div(2);
        var backspin = RotationsPerSecond.of(0);
        HoopSolver solver = new HoopSolver(hubHeight, hubRadius, lip, backspin);

        List<ShotEntry> entries = new ArrayList<>();

        double[] speeds = {-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0};

        Distance testDistance = Feet.of(8);
        LinearVelocity testRadialVelocity = MetersPerSecond.of(0.5);

        var shots_ =
            solver.findValidShots(testDistance, testRadialVelocity, MetersPerSecond.of(0.1),
                MetersPerSecond.of(12), 200, Degrees.of(47), Degrees.of(90 - 12), 100);
        String html = ShotVisualizer.generate(testDistance, hubHeight, hubRadius, lip,
            testRadialVelocity, backspin, shots_);
        Files.writeString(Path.of("test.html"), html);

        Lock lock = new ReentrantLock();
        for (double distanceFeet = 3.0; distanceFeet < 30.0; distanceFeet += 1.0) {
            System.out.println(distanceFeet);
            Arrays.stream(speeds).parallel().forEach(speed -> {
                Distance targetDistance = Feet.of(12);
                var radialVelocity = MetersPerSecond.of(2.0);

                var shots =
                    solver.findValidShots(targetDistance, radialVelocity, MetersPerSecond.of(0.1),
                        MetersPerSecond.of(12), 200, Degrees.of(47), Degrees.of(90 - 12), 100);

                var optimalShot = solver.findOptimal(shots, targetDistance, radialVelocity);
                if (optimalShot.isPresent()) {
                    var params = optimalShot.get().shot();
                    lock.lock();
                    entries.add(new ShotEntry(targetDistance, radialVelocity, params.exitAngle(),
                        params.exitVelocity(), params.tof()));
                    lock.unlock();
                }
            });
        }

        BinaryData.writeFile("shots-rt.bin", ShotEntrySetWriter::write,
            new ShotData.ShotEntrySet(entries.toArray(ShotData.ShotEntry[]::new)));
    }

}
