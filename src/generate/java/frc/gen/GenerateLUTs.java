package frc.gen;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.Function;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.FieldConstants;
import frc.robot.math.opt.BFGS;
import frc.robot.math.opt.FiniteDifference;
import frc.robot.shotdata.ShotData;
import frc.robot.shotdata.ShotData.ShotEntry;
import frc.robot.shotdata.SimulatedShot;

public class GenerateLUTs {

    public static void main(String[] argv) {
        BFGS bfgs = new BFGS();
        Function<double[], Function<ShotEntry, SimulatedShot>> entryToShotOpt =
            x -> entry -> new SimulatedShot(entry.exitAngle(),
                entry.noSlipExitVelocity().times(x[0]),
                RotationsPerSecond.of(x[1] + x[2] * entry.hoodAngle().in(Degrees)));
        var func = new FiniteDifference(x -> {
            return rmse(entryToShotOpt.apply(x));
        }, 1e-3);
        bfgs.optimize(func, new double[] {0.44, 5.0, 0.0}, 1e-2);
        var entryToShot = entryToShotOpt.apply(bfgs.getOptValue());

        InterpolatingDoubleTreeMap[] trajectories =
            new InterpolatingDoubleTreeMap[ShotData.entries.length];
        double min = 0.0;
        double max = 0.0;
        for (int i = 0; i < ShotData.entries.length; i++) {
            InterpolatingDoubleTreeMap traj = new InterpolatingDoubleTreeMap();
            SimulatedShot shot = entryToShot.apply(ShotData.entries[i]);
            shot.state.a1 = -ShotData.entries[i].targetDistance().in(Meters);
            while (shot.state.a2 >= 0.0) {
                traj.put(shot.state.a1, shot.state.a2);
                min = Math.min(min, shot.state.a1);
                max = Math.max(max, shot.state.a1);
                shot.step(0.001);
            }
            traj.put(shot.state.a1, 0.0);
            trajectories[i] = traj;
        }

        NumberFormat formatter = new DecimalFormat("#0.000");
        try (FileWriter writer = new FileWriter(new File("trajectories.txt"))) {
            for (double x = min; x < max; x += 0.01) {
                writer.write(formatter.format(x));
                for (var traj : trajectories) {
                    writer.write('\t');
                    writer.write(formatter.format(traj.get(x)));
                }
                writer.write('\t');
                if (Math.abs(x) < FieldConstants.Hub.width / 2.0) {
                    writer.write(formatter.format(ShotData.shooterToTargetHeightDiff.in(Meters)));
                }
                writer.write('\n');
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static double rmse(Function<ShotEntry, SimulatedShot> f) {
        double res = 0.0;
        for (var entry : ShotData.entries) {
            var shot = f.apply(entry);
            double err = trajectoryError(shot, entry);
            res += err * err;
        }
        return res / ShotData.entries.length;
    }

    private static double trajectoryError(SimulatedShot shot, ShotEntry entry) {
        InterpolatingDoubleTreeMap past = new InterpolatingDoubleTreeMap();
        while (shot.state.a1 < entry.targetDistance().in(Meters)) {
            past.put(shot.state.a1, shot.state.a2);
            shot.step(0.001);
        }
        past.put(shot.state.a1, shot.state.a2);
        return ShotData.shooterToTargetHeightDiff.in(Meters)
            - past.get(entry.targetDistance().in(Meters));
    }

}
