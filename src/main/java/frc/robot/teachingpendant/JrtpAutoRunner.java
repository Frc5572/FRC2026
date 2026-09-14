package frc.robot.teachingpendant;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;

/** Loads requested .jrtp autos and turns their drive steps into normal WPILib commands. */
public final class JrtpAutoRunner {
    private final Swerve swerve;
    private final AudibleManager audibles = new AudibleManager();
    private final BiFunction<JrtpAuto.Step, Map<String, Object>, Command> existingCommandResolver;
    private JrtpAuto loaded;
    private String loadedFile = "";
    private final StringSubscriber requestedJson = edu.wpi.first.networktables.NetworkTableInstance
        .getDefault().getTable(AudibleManager.ROOT).getStringTopic("RequestedAutoJson").subscribe("");
    private final StringSubscriber requestedAuto = edu.wpi.first.networktables.NetworkTableInstance
        .getDefault().getTable(AudibleManager.ROOT).getStringTopic("RequestedAuto").subscribe("");

    public JrtpAutoRunner(Swerve swerve,
        BiFunction<JrtpAuto.Step, Map<String, Object>, Command> existingCommandResolver) {
        this.swerve = swerve;
        this.existingCommandResolver = existingCommandResolver;
    }

    /** Poll while disabled; only names inside deploy/jrtp-autos are accepted. */
    public void refreshRequestedAuto() {
        String json = requestedJson.get();
        if (!json.isBlank() && !json.equals(loadedFile)) {
            try {
                loaded = JrtpFiles.decode(json); loadedFile = json; audibles.load(loaded); return;
            } catch (Exception exception) {
                edu.wpi.first.wpilibj.DriverStation.reportError("Could not load .jrtp sent by pendant: "
                    + exception.getMessage(), exception.getStackTrace());
                return;
            }
        }
        String requested = requestedAuto.get();
        if (requested.isBlank() || requested.equals(loadedFile) || requested.contains("..")
            || requested.contains("/") || !requested.endsWith(".jrtp")) return;
        try {
            Path file = Filesystem.getDeployDirectory().toPath().resolve("jrtp-autos").resolve(requested);
            loaded = JrtpFiles.load(file); loadedFile = requested; audibles.load(loaded);
        } catch (Exception exception) {
            edu.wpi.first.wpilibj.DriverStation.reportError("Could not load requested .jrtp auto: "
                + exception.getMessage(), exception.getStackTrace());
        }
    }

    public boolean hasLoadedAuto() { return loaded != null; }

    public Command command() {
        if (loaded == null) return Commands.none();
        List<Command> commands = new ArrayList<>();
        for (JrtpAuto.Step step : loaded.steps) {
            switch (step.type) {
                case "checkpoint" -> commands.add(Commands.runOnce(() -> audibles.valuesFor(step.name)));
                case "driveToPose" -> commands.add(driveStep(step));
                case "existingCommand" -> commands.add(existingCommandResolver.apply(step,
                    audibles.valuesFor(String.valueOf(step.parameters.getOrDefault("checkpoint", "")))));
                case "wait" -> commands.add(Commands.waitSeconds(number(step.parameters, "seconds", 0.0)));
                default -> edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "Ignoring unknown .jrtp step type: " + step.type, false);
            }
        }
        return Commands.sequence(commands.toArray(Command[]::new)).withTimeout(loaded.maximumTime)
            .finallyDo(interrupted -> swerve.stop());
    }

    private Command driveStep(JrtpAuto.Step step) {
        double x = step.x == null ? 0.0 : step.x;
        double y = step.y == null ? 0.0 : step.y;
        double heading = step.rotationDegrees == null ? 0.0 : step.rotationDegrees;
        double speed = step.maxSpeed == null ? 2.5 : Math.max(0.1, step.maxSpeed);
        return swerve.moveToPose().target(new Pose2d(x, y, Rotation2d.fromDegrees(heading)))
            .maxSpeed(speed).finish();
    }

    private static double number(Map<String, Object> parameters, String name, double fallback) {
        Object value = parameters.get(name);
        return value instanceof Number number ? number.doubleValue() : fallback;
    }
}
