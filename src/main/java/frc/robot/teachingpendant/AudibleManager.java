package frc.robot.teachingpendant;

import java.util.LinkedHashMap;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Publishes checkpoint definitions and resolves safe dashboard-selected values at execution time.
 * The external dashboard owns presentation; this class owns defaults and validation only.
 */
public final class AudibleManager {
    public static final String ROOT = "/rosbots/Autonomous";
    private final NetworkTable root = NetworkTableInstance.getDefault().getTable(ROOT);
    private JrtpAuto auto;

    public void load(JrtpAuto newAuto) {
        auto = newAuto;
        root.getStringTopic("LoadedAuto").publish().set(newAuto.name);
        root.getDoubleTopic("MaximumTime").publish().set(newAuto.maximumTime);
        for (JrtpAuto.Step step : newAuto.steps) {
            if (!"checkpoint".equals(step.type) || step.name == null)
                continue;
            NetworkTable checkpoint = root.getSubTable("Checkpoints").getSubTable(step.name);
            for (Map.Entry<String, JrtpAuto.Input> entry : step.inputs.entrySet()) {
                publishDefault(checkpoint, entry.getKey(), entry.getValue());
            }
        }
    }

    public Object value(String checkpointName, String inputName, Object fallback) {
        if (auto == null)
            return fallback;
        for (JrtpAuto.Step step : auto.steps) {
            if ("checkpoint".equals(step.type) && checkpointName.equals(step.name)) {
                JrtpAuto.Input input = step.inputs.get(inputName);
                if (input == null)
                    return fallback;
                return readAndValidate(root.getSubTable("Checkpoints").getSubTable(checkpointName),
                    inputName, input);
            }
        }
        return fallback;
    }

    public Map<String, Object> valuesFor(String checkpointName) {
        Map<String, Object> values = new LinkedHashMap<>();
        if (auto == null)
            return values;
        for (JrtpAuto.Step step : auto.steps) {
            if ("checkpoint".equals(step.type) && checkpointName.equals(step.name)) {
                for (Map.Entry<String, JrtpAuto.Input> entry : step.inputs.entrySet()) {
                    values.put(entry.getKey(), value(checkpointName, entry.getKey(), null));
                }
            }
        }
        return values;
    }

    private static void publishDefault(NetworkTable table, String name, JrtpAuto.Input input) {
        Object value = input.defaultValue;
        if (value instanceof Boolean bool)
            table.getBooleanTopic(name).publish().setDefault(bool);
        else if (value instanceof Number number)
            table.getDoubleTopic(name).publish().setDefault(number.doubleValue());
        else
            table.getStringTopic(name).publish().setDefault(String.valueOf(value));
        if (!input.options.isEmpty()) {
            table.getStringArrayTopic(name + "/options").publish()
                .set(input.options.stream().map(String::valueOf).toArray(String[]::new));
        }
        if (input.minimum != null)
            table.getDoubleTopic(name + "/minimum").publish().set(input.minimum);
        if (input.maximum != null)
            table.getDoubleTopic(name + "/maximum").publish().set(input.maximum);
    }

    private static Object readAndValidate(NetworkTable table, String name, JrtpAuto.Input input) {
        Object result;
        if (input.defaultValue instanceof Boolean bool) {
            result = table.getEntry(name).getBoolean(bool);
        } else if (input.defaultValue instanceof Number number) {
            double value = table.getEntry(name).getDouble(number.doubleValue());
            if (input.minimum != null)
                value = Math.max(input.minimum, value);
            if (input.maximum != null)
                value = Math.min(input.maximum, value);
            result = "integer".equals(input.type) ? (int) Math.round(value) : value;
        } else {
            String value = table.getEntry(name).getString(String.valueOf(input.defaultValue));
            result = input.options.isEmpty()
                || input.options.stream().map(String::valueOf).anyMatch(value::equals) ? value
                    : input.defaultValue;
        }
        return result;
    }
}
