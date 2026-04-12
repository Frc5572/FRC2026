package frc.gen;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
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
import java.util.Map.Entry;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;
import javax.lang.model.element.Modifier;
import com.squareup.javapoet.FieldSpec;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.ParameterSpec;
import com.squareup.javapoet.TypeName;
import com.squareup.javapoet.TypeSpec;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.math.opt.BFGS;
import frc.robot.math.opt.DiffFunc;
import frc.robot.math.opt.FiniteDifference;
import frc.robot.shotdata.ShotData;
import frc.robot.shotdata.ShotData.ShotEntry;
import frc.robot.shotdata.SimulatedShot;
import frc.robot.util.Tuples.Tuple2;

/** Generate lookup tables */
public class GenerateLUTs {

    /** Entrypoint for generateLUTs gradle task */
    public static void main(String[] argv) {
        // v1();
        NumberFormat formatter = new DecimalFormat("#0.000");

        InterpolatingDoubleTreeMap[] trajectories =
            new InterpolatingDoubleTreeMap[ShotData.entries.length];
        List<ShotEntry> entries = new ArrayList<>();
        List<ShotEntry> groundEntries = new ArrayList<>();
        @SuppressWarnings("unchecked")
        Tuple2<AngularVelocity, LinearVelocity>[] speedTransfer =
            new Tuple2[ShotData.entries.length];
        double min = 0.0;
        double max = 0.0;
        double maxHubDistance = 0.0;
        double maxFlywheelSpeed = ShotData.entries[0].flywheelSpeed().in(RotationsPerSecond);
        for (int i = 0; i < ShotData.entries.length; i++) {
            var entry = ShotData.entries[i];
            maxHubDistance = Math.max(maxHubDistance, entry.targetDistance().in(Meters));
            double lower = 1.5;
            double upper = 20.0;
            DoubleUnaryOperator f = (exitSpeed) -> {
                SimulatedShot shot = new SimulatedShot(entry.exitAngle(),
                    MetersPerSecond.of(exitSpeed), RotationsPerSecond.of(5));
                while (shot.state.a1 < entry.targetDistance().in(Meters)) {
                    shot.step(0.001);
                    if (shot.state.a3 < 0.2) {
                        return -1000.0;
                    }
                }
                return shot.state.a2 - ShotData.shooterToTargetHeightDiff.in(Meters);
            };
            double fUpper = f.applyAsDouble(upper);
            for (int j = 0; j < 200; j++) {
                double mid = (lower + upper) / 2.0;
                double fMid = f.applyAsDouble(mid);
                if (fMid * fUpper > 0) {
                    upper = mid;
                    fUpper = fMid;
                } else {
                    lower = mid;
                }
            }

            double res = (lower + upper) / 2.0;
            speedTransfer[i] = new Tuple2<AngularVelocity, LinearVelocity>(entry.flywheelSpeed(),
                MetersPerSecond.of(res));
            SimulatedShot shot = new SimulatedShot(entry.exitAngle(), MetersPerSecond.of(res),
                RotationsPerSecond.of(5));
            shot.state.a1 = -entry.targetDistance().in(Meters);
            InterpolatingDoubleTreeMap traj = new InterpolatingDoubleTreeMap();
            double tof = 0.0;
            double groundTof = 0.0;
            while (shot.state.a2 >= 0.0) {
                traj.put(shot.state.a1, shot.state.a2);
                min = Math.min(min, shot.state.a1);
                max = Math.max(max, shot.state.a1);
                shot.step(0.001);
                if (shot.state.a1 < 0.0) {
                    tof += 0.001;
                }
                groundTof += 0.001;
            }
            traj.put(shot.state.a1, 0.0);
            trajectories[i] = traj;

            System.out.println(
                formatter.format(entry.targetDistance().in(Meters)) + "\t" + formatter.format(tof));

            entries.add(new ShotEntry(entry.targetDistance(), entry.flywheelSpeed(),
                entry.exitAngle(), Seconds.of(tof)));

            groundEntries
                .add(new ShotEntry(Meters.of(entry.targetDistance().in(Meters) + shot.state.a1),
                    entry.flywheelSpeed(), entry.exitAngle(), Seconds.of(groundTof)));
        }

        InterpolatingDoubleTreeMap hub = new InterpolatingDoubleTreeMap();
        hub.put(-FieldConstants.Hub.width / 2.0, FieldConstants.Hub.height);
        hub.put(FieldConstants.Hub.width / 2.0, FieldConstants.Hub.height);
        hub.put(-FieldConstants.Hub.innerWidth / 2.0, FieldConstants.Hub.innerHeight);
        hub.put(FieldConstants.Hub.innerWidth / 2.0, FieldConstants.Hub.innerHeight);

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

        OLS<Tuple2<AngularVelocity, LinearVelocity>> ols = new OLS<>(speedTransfer)
            .term("1.0", _x -> 1.0).term("flywheelSpeed", x -> x._0().in(RotationsPerSecond)).term(
                "flywheelSpeed * flywheelSpeed", x -> Math.pow(x._0().in(RotationsPerSecond), 2.0));

        var olsRes = ols.solve(x -> x._1().in(MetersPerSecond));

        OLS<ShotEntry> ols2 = new OLS<>(ShotData.entries).term("1.0", _x -> 1.0)
            .term("distance", x -> x.targetDistance().in(Meters))
            .term("distance * distance", x -> Math.pow(x.targetDistance().in(Meters), 2.0))
            .term("distance * distance * distance",
                x -> Math.pow(x.targetDistance().in(Meters), 3.0));

        var ols2Res = ols2.solve(x -> x.hoodAngle().in(Degrees));
        System.out.println(ols2Res);

        try (FileWriter writer = new FileWriter(new File("targetToHood.txt"))) {
            for (var entry : ShotData.entries) {
                writer.write(formatter.format(entry.targetDistance().in(Meters)));
                writer.write('\t');
                writer.write(formatter.format(entry.hoodAngle().in(Degrees)));
                writer.write('\t');
                writer.write(formatter.format(ols2Res.evaluate(entry)));
                writer.write('\n');
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        for (double flywheel = maxFlywheelSpeed + 2.0; flywheel < 90.0; flywheel += 2.0) {
            var hoodAngle = Degrees.of(90 - 12.695 - 25);
            var shot = new SimulatedShot(hoodAngle,
                MetersPerSecond.of(olsRes.evaluate(new Tuple2<AngularVelocity, LinearVelocity>(
                    RotationsPerSecond.of(flywheel), null))),
                RotationsPerSecond.of(5));
            double tof = 0.0;
            double groundTof = 0.0;
            double hubDistance = 0.0;
            while (shot.state.a2 >= 0.0) {
                shot.step(0.001);
                if (shot.state.a4 >= 0.0
                    || shot.state.a2 > ShotData.shooterToTargetHeightDiff.in(Meters)) {
                    tof += 0.001;
                    hubDistance = shot.state.a1;
                }
                groundTof += 0.001;
            }
            if (hubDistance < maxHubDistance) {
                continue;
            }
            entries.add(new ShotEntry(Meters.of(hubDistance), RotationsPerSecond.of(flywheel),
                hoodAngle, Seconds.of(tof)));

            groundEntries.add(new ShotEntry(Meters.of(shot.state.a1),
                RotationsPerSecond.of(flywheel), hoodAngle, Seconds.of(groundTof)));
        }

        TypeSpec.Builder classBuilder =
            TypeSpec.classBuilder("GeneratedLUTs2").addModifiers(Modifier.PUBLIC, Modifier.FINAL);

        StringBuilder init = new StringBuilder("new ShotData.ShotEntry[] {");
        for (int i = 0; i < entries.size(); i++) {
            if (i != 0) {
                init.append(',');
            }
            init.append(" new ShotData.ShotEntry(");
            init.append(formatter.format(entries.get(i).targetDistance().in(Feet)));
            init.append(", ");
            init.append(formatter.format(entries.get(i).flywheelSpeed().in(RotationsPerSecond)));
            init.append(", ");
            init.append(formatter.format(entries.get(i).hoodAngle().in(Degrees)));
            init.append(", ");
            init.append(formatter.format(entries.get(i).tof().in(Seconds)));
            init.append(')');
        }
        init.append(" }");
        FieldSpec entriesField = FieldSpec
            .builder(ShotEntry[].class, "hubEntries", Modifier.PUBLIC, Modifier.STATIC,
                Modifier.FINAL)
            .initializer(init.toString())
            .addJavadoc("Pre-computed shot entries for hub-targeted shots.").build();
        classBuilder.addField(entriesField);

        init = new StringBuilder("new ShotData.ShotEntry[] {");
        for (int i = 0; i < groundEntries.size(); i++) {
            if (i != 0) {
                init.append(',');
            }
            init.append(" new ShotData.ShotEntry(");
            init.append(formatter.format(groundEntries.get(i).targetDistance().in(Feet)));
            init.append(", ");
            init.append(
                formatter.format(groundEntries.get(i).flywheelSpeed().in(RotationsPerSecond)));
            init.append(", ");
            init.append(formatter.format(groundEntries.get(i).hoodAngle().in(Degrees)));
            init.append(", ");
            init.append(formatter.format(groundEntries.get(i).tof().in(Seconds)));
            init.append(')');
        }
        init.append(" }");
        entriesField = FieldSpec
            .builder(ShotEntry[].class, "groundEntries", Modifier.PUBLIC, Modifier.STATIC,
                Modifier.FINAL)
            .initializer(init.toString()).addJavadoc("Pre-computed shot entries for passing.")
            .build();
        classBuilder.addField(entriesField);

        classBuilder.addJavadoc(
            "Auto-generated lookup tables (LUTs) and fitted model coefficients for shooter calculations.");
        classBuilder.addJavadoc("");
        classBuilder.addJavadoc(
            "<p>This class is not meant to be instantiated or modified manually — values are");
        classBuilder.addJavadoc("derived from experimental shot data and curve-fitting.");

        try {
            JavaFile.builder("frc.robot.shotdata", classBuilder.build()).build()
                .writeTo(new File("src/main/java"));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void v1() {
        BFGS bfgs = new BFGS();
        Function<double[], Function<ShotEntry, SimulatedShot>> entryToShotOpt =
            x -> entry -> new SimulatedShot(entry.exitAngle(),
                MetersPerSecond.of(entry.noSlipExitVelocity().in(MetersPerSecond) * x[0]),
                RotationsPerSecond.of(x[1] + x[2] * entry.hoodAngle().in(Degrees)
                    + x[3] * entry.flywheelSpeed().in(RotationsPerSecond)));
        var func = new FiniteDifference(x -> {
            return rmse(entryToShotOpt.apply(x));
        }, 1e-8);
        bfgs.optimize(func, new double[] {0.4701222994041102, 0.9300718478163617,
            2.5900070262535433, 0.1028502153770187}, 1e-5, 100);
        System.out.println(bfgs);
        var optValue = bfgs.getOptValue();
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
        List<ShotEntry> groundEntries = new ArrayList<>();
        Map<Double, Double> maxDistance = new HashMap<>();
        Map<Double, Double> maxDistanceHoodAngle = new HashMap<>();
        Map<Double, Double> minDistance = new HashMap<>();
        for (int i = 0; i < infos.size(); i++) {
            var info = infos.get(i);
            var groundEntry = new ShotEntry(info.info.groundDistance.in(Feet),
                info.flywheelSpeedRps, info.hoodAngleDeg, info.info.tofGround.in(Seconds));
            groundEntries.add(groundEntry);
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

        System.out.println("Finding trajectories with given clearance...");

        double targetClearance = Units.feetToMeters(1);
        Map<Double, Double> desiredFlywheelSpeeds = new HashMap<>();
        for (int i = 0; i < entries.size(); i++) {
            System.out.println(i + " / " + entries.size());
            var entry = entries.get(i);
            DiffFunc clearanceFunc = new FiniteDifference(x -> {
                var shot = entryToShot.apply(new ShotEntry(0, x[0], x[1], 0));
                var info = new TrajectoryInfo(shot.exitAngle, shot.exitVelocity, shot.backspin);
                double clearance = info.clearanceOverLip.in(Meters);
                double clearanceErr = clearance - targetClearance;
                double targetErr = info.hubDistance.in(Meters) - entry.targetDistance().in(Meters);
                return 0.5 * clearanceErr * clearanceErr + targetErr * targetErr;
            }, 1e-5);
            bfgs.optimize(clearanceFunc, new double[] {entry.flywheelSpeed().in(RotationsPerSecond),
                entry.hoodAngle().in(Degrees)}, 1e-3, 100);
            desiredFlywheelSpeeds.put(entry.targetDistance().in(Meters), bfgs.getOptValue()[0]);
        }
        @SuppressWarnings("unchecked")
        Entry<Double, Double>[] desiredFlywheelSpeedsArr =
            desiredFlywheelSpeeds.entrySet().toArray(Entry[]::new);

        OLS<Entry<Double, Double>> ols = new OLS<>(desiredFlywheelSpeedsArr).term("1.0", _x -> 1.0)
            .term("distance", x -> x.getKey())
            .term("distance * distance", x -> x.getKey() * x.getKey());
        var olsSoln = ols.solve(x -> x.getValue());

        System.out.println("done");

        try (FileWriter writer = new FileWriter(new File("desiredSpeed.txt"))) {
            for (var entry : desiredFlywheelSpeeds.entrySet()) {
                writer.write(formatter.format(entry.getKey()));
                writer.write('\t');
                writer.write(formatter.format(entry.getValue()));
                writer.write('\n');
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        TypeSpec.Builder classBuilder =
            TypeSpec.classBuilder("GeneratedLUTs").addModifiers(Modifier.PUBLIC, Modifier.FINAL);

        StringBuilder init = new StringBuilder("new ShotData.ShotEntry[] {");
        for (int i = 0; i < entries.size(); i++) {
            if (i != 0) {
                init.append(',');
            }
            init.append(" new ShotData.ShotEntry(");
            init.append(formatter.format(entries.get(i).targetDistance().in(Feet)));
            init.append(", ");
            init.append(formatter.format(entries.get(i).flywheelSpeed().in(RotationsPerSecond)));
            init.append(", ");
            init.append(formatter.format(entries.get(i).hoodAngle().in(Degrees)));
            init.append(", ");
            init.append(formatter.format(entries.get(i).tof().in(Seconds)));
            init.append(')');
        }
        init.append(" }");
        FieldSpec entriesField = FieldSpec
            .builder(ShotEntry[].class, "hubEntries", Modifier.PUBLIC, Modifier.STATIC,
                Modifier.FINAL)
            .initializer(init.toString())
            .addJavadoc("Pre-computed shot entries for hub-targeted shots.").build();
        classBuilder.addField(entriesField);

        init = new StringBuilder("new ShotData.ShotEntry[] {");
        for (int i = 0; i < groundEntries.size(); i++) {
            if (i != 0) {
                init.append(',');
            }
            init.append(" new ShotData.ShotEntry(");
            init.append(formatter.format(groundEntries.get(i).targetDistance().in(Feet)));
            init.append(", ");
            init.append(
                formatter.format(groundEntries.get(i).flywheelSpeed().in(RotationsPerSecond)));
            init.append(", ");
            init.append(formatter.format(groundEntries.get(i).hoodAngle().in(Degrees)));
            init.append(", ");
            init.append(formatter.format(groundEntries.get(i).tof().in(Seconds)));
            init.append(')');
        }
        init.append(" }");
        entriesField = FieldSpec
            .builder(ShotEntry[].class, "groundEntries", Modifier.PUBLIC, Modifier.STATIC,
                Modifier.FINAL)
            .initializer(init.toString())
            .addJavadoc("Pre-computed shot entries for ground-pass trajectories.").build();
        classBuilder.addField(entriesField);

        classBuilder.addMethod(MethodSpec.methodBuilder("desiredFlywheelSpeed")
            .returns(TypeName.DOUBLE).addModifiers(Modifier.PUBLIC, Modifier.STATIC)
            .addParameter(ParameterSpec.builder(TypeName.DOUBLE, "distance").build())
            .addJavadoc("Returns the desired flywheel speed in rotations per second for a given ")
            .addJavadoc("target distance, using a fitted quadratic model.")
            .addCode("return " + olsSoln.res() + ";").build());

        classBuilder
            .addMethod(MethodSpec.methodBuilder("estimatedBackspin").returns(TypeName.DOUBLE)
                .addModifiers(Modifier.PUBLIC, Modifier.STATIC)
                .addParameter(ParameterSpec.builder(TypeName.DOUBLE, "hoodAngleDeg").build())
                .addParameter(ParameterSpec.builder(TypeName.DOUBLE, "flywheelSpeedRps").build())
                .addJavadoc("Estimates the backspin imparted on the ball in rotations per second, ")
                .addJavadoc("using a fitted linear model over hood angle and flywheel speed.")
                .addCode("return " + formatter.format(optValue[1]) + " + "
                    + formatter.format(optValue[2]) + " * hoodAngleDeg + "
                    + formatter.format(optValue[3]) + " * flywheelSpeedRps;")
                .build());

        classBuilder.addField(FieldSpec
            .builder(TypeName.DOUBLE, "SPEED_TRANSFER_COEFF", Modifier.PUBLIC, Modifier.STATIC,
                Modifier.FINAL)
            .initializer(formatter.format(optValue[0]))
            .addJavadoc(
                "Fraction of flywheel surface speed transferred to the ball as exit velocity.")
            .build());

        classBuilder.addJavadoc(
            "Auto-generated lookup tables (LUTs) and fitted model coefficients for shooter calculations.");
        classBuilder.addJavadoc("");
        classBuilder.addJavadoc(
            "<p>This class is not meant to be instantiated or modified manually — values are");
        classBuilder.addJavadoc("derived from experimental shot data and curve-fitting.");

        try {
            JavaFile.builder("frc.robot.shotdata", classBuilder.build()).build()
                .writeTo(new File("src/main/java"));
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
