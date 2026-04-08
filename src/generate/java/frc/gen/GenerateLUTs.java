package frc.gen;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
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
        }, 1e-5);
        bfgs.optimize(func, new double[] {0.44, 1.0, 0.0}, 1e-4, 1000);
        System.out.println(bfgs);
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

        InterpolatingDoubleTreeMap hub = new InterpolatingDoubleTreeMap();
        hub.put(-FieldConstants.Hub.width / 2.0, FieldConstants.Hub.height);
        hub.put(FieldConstants.Hub.width / 2.0, FieldConstants.Hub.height);
        hub.put(-FieldConstants.Hub.innerWidth / 2.0, FieldConstants.Hub.innerHeight);
        hub.put(FieldConstants.Hub.innerWidth / 2.0, FieldConstants.Hub.innerHeight);

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
                    writer.write(
                        formatter.format(hub.get(x) - Constants.Shooter.shooterHeight.in(Meters)));
                }
                writer.write('\n');
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        List<TrajectoryInputs> inputs = new ArrayList<>();
        for (double hoodAngle = 0.0; hoodAngle < 35.0; hoodAngle += 1.0) {
            for (double flywheelSpeedRps = 40.0; flywheelSpeedRps < 100.0; flywheelSpeedRps +=
                2.0) {
                inputs.add(new TrajectoryInputs(flywheelSpeedRps, hoodAngle));
            }
        }
        int totalCount = inputs.size();
        System.out.println("Generating " + totalCount + " trajectories...");
        var infos = inputs.stream().parallel().map(input -> {
            var shot = entryToShot
                .apply(new ShotEntry(0, input.flywheelSpeedRps(), input.hoodAngleDeg(), 0));
            var res = new TrajectoryInfo(shot.exitAngle, shot.exitVelocity, shot.backspin);
            return new TrajectoryOutputs(input.flywheelSpeedRps, input.hoodAngleDeg, res);
        }).toList();
        infos = new ArrayList<>(infos);
        System.out.println("Cleaning up...");
        Map<Double, Double> maxDistance = new HashMap<>();
        Map<Double, Double> maxDistanceHoodAngle = new HashMap<>();
        Map<Double, Double> minDistance = new HashMap<>();
        for (int i = 0; i < infos.size(); i++) {
            var info = infos.get(i);
            if (!info.info.reachesHub || info.info.clearanceOverLip.lt(Centimeters.of(15))) {
                infos.remove(i);
                i--;
                continue;
            }
            double maxForFlywheel = maxDistance.getOrDefault(info.flywheelSpeedRps, 0.0);
            double minForFlywheel = minDistance.getOrDefault(info.flywheelSpeedRps, 200.0);
            if (info.info.hubDistance.in(Meters) > maxForFlywheel) {
                maxDistance.put(info.flywheelSpeedRps, info.info.hubDistance.in(Meters));
                maxDistanceHoodAngle.put(info.flywheelSpeedRps, info.hoodAngleDeg);
            }
            if (info.info.hubDistance.in(Meters) < minForFlywheel) {
                minDistance.put(info.flywheelSpeedRps, info.info.hubDistance.in(Meters));
            }
        }
        for (int i = 0; i < infos.size(); i++) {
            var info = infos.get(i);
            if (info.hoodAngleDeg > maxDistanceHoodAngle.get(info.flywheelSpeedRps)) {
                infos.remove(i);
                i--;
            }
        }
        System.out.println("done");

        List<ShotEntry> entries = new ArrayList<>();
        List<InterpolatingDoubleTreeMap> drawnTrajectories = new ArrayList<>();

        min = 0.0;
        max = 0.0;
        for (var info : infos) {
            double distance = info.info.hubDistance.in(Meters);
            double flywheelSpeedRps = info.flywheelSpeedRps;
            double hoodAngleDeg = info.hoodAngleDeg;
            double tof = info.info.tofHub.in(Seconds);
            var entry =
                new ShotEntry(Units.metersToFeet(distance), flywheelSpeedRps, hoodAngleDeg, tof);
            entries.add(entry);
            SimulatedShot shot = entryToShot.apply(entry);
            shot.state.a1 = -distance;
            InterpolatingDoubleTreeMap traj = new InterpolatingDoubleTreeMap();
            while (shot.state.a2 >= 0.0) {
                traj.put(shot.state.a1, shot.state.a2);
                min = Math.min(min, shot.state.a1);
                max = Math.max(max, shot.state.a1);
                shot.step(0.001);
            }
            traj.put(shot.state.a1, 0.0);
            drawnTrajectories.add(traj);
        }

        System.out.println(drawnTrajectories.size() + " valid trajectories");

        try (FileWriter writer = new FileWriter(new File("opt.txt"))) {
            for (var info : infos) {
                writer.write(formatter.format(info.info.hubDistance.in(Meters)));
                writer.write('\t');
                writer.write(formatter.format(info.flywheelSpeedRps));
                writer.write('\t');
                writer.write(formatter.format(info.hoodAngleDeg));
                writer.write('\t');
                writer.write(formatter.format(info.info.tofHub.in(Seconds)));
                writer.write('\n');
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        try (FileWriter writer = new FileWriter(new File("validation.txt"))) {
            for (double x = min; x <= max; x += 0.02) {
                writer.write(formatter.format(x));
                for (var traj : drawnTrajectories) {
                    writer.write('\t');
                    writer.write(formatter.format(traj.get(x)));
                }
                writer.write('\t');
                if (Math.abs(x) < FieldConstants.Hub.width / 2.0) {
                    writer.write(
                        formatter.format(hub.get(x) - Constants.Shooter.shooterHeight.in(Meters)));
                }
                writer.write('\n');
            }
        } catch (IOException e) {
            e.printStackTrace();
        }


    }

    private static record TrajectoryInputs(double flywheelSpeedRps, double hoodAngleDeg) {
    };

    private static record TrajectoryOutputs(double flywheelSpeedRps, double hoodAngleDeg,
        TrajectoryInfo info) {
    };

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
