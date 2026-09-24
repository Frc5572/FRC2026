package frc.robot.controls;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.jspecify.annotations.NullMarked;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * A named set of {@link ControlsConfig} profiles, one of which is active.
 *
 * <p>
 * This is the in-memory form of {@code profiles.json}. Parsing is deliberately forgiving: unknown
 * keys are ignored and missing ones fall back to {@link ControlsField#defaultValue()}, so a file
 * checked in before a field was added still loads, and a hand-edited file cannot brick the robot.
 */
@NullMarked
public final class ControlsProfiles {

    /** Name of the profile created when no file exists. */
    public static final String DEFAULT_PROFILE = "default";

    private static final ObjectMapper MAPPER = new ObjectMapper();

    private final Map<String, ControlsConfig> profiles = new LinkedHashMap<>();
    private String active;

    /** Create a profile set containing only the factory defaults. */
    public ControlsProfiles() {
        this.active = DEFAULT_PROFILE;
        this.profiles.put(DEFAULT_PROFILE, ControlsConfig.defaults());
    }

    /** The name of the currently selected profile. */
    public String activeName() {
        return active;
    }

    /** The configuration of the currently selected profile. */
    public ControlsConfig active() {
        ControlsConfig cfg = profiles.get(active);
        return cfg == null ? ControlsConfig.defaults() : cfg;
    }

    /** Every known profile name, in insertion order. */
    public List<String> names() {
        return new ArrayList<>(profiles.keySet());
    }

    /**
     * Select a profile by name.
     *
     * @param name the profile to activate
     * @return true if the profile existed and is now active
     */
    public boolean setActive(String name) {
        if (!profiles.containsKey(name)) {
            return false;
        }
        this.active = name;
        return true;
    }

    /** Replace the configuration stored under the active profile. */
    public void putActive(ControlsConfig config) {
        profiles.put(active, config);
    }

    /**
     * Create a profile, or overwrite an existing one, and make it active.
     *
     * @param name the profile name
     * @param config the configuration to store
     */
    public void put(String name, ControlsConfig config) {
        profiles.put(name, config);
        this.active = name;
    }

    /**
     * Delete a profile. The last remaining profile cannot be deleted.
     *
     * @param name the profile to remove
     * @return true if the profile was removed
     */
    public boolean remove(String name) {
        if (profiles.size() <= 1 || !profiles.containsKey(name)) {
            return false;
        }
        profiles.remove(name);
        if (active.equals(name)) {
            active = profiles.keySet().iterator().next();
        }
        return true;
    }

    /**
     * Parse a profile set from JSON.
     *
     * @param json the document to read
     * @return the parsed profiles, or factory defaults if the document is unusable
     */
    public static ControlsProfiles fromJson(String json) {
        ControlsProfiles out = new ControlsProfiles();
        try {
            JsonNode root = MAPPER.readTree(json);
            JsonNode profilesNode = root.get("profiles");
            if (profilesNode == null || !profilesNode.isObject()) {
                return out;
            }
            out.profiles.clear();
            for (Map.Entry<String, JsonNode> entry : profilesNode.properties()) {
                out.profiles.put(entry.getKey(), readConfig(entry.getValue()));
            }
            if (out.profiles.isEmpty()) {
                out.profiles.put(DEFAULT_PROFILE, ControlsConfig.defaults());
            }
            JsonNode activeNode = root.get("active");
            String wanted = activeNode == null ? null : activeNode.asText();
            out.active = wanted != null && out.profiles.containsKey(wanted) ? wanted
                : out.profiles.keySet().iterator().next();
        } catch (Exception e) {
            System.err.println("[Controls] could not parse profiles, using defaults: " + e);
            return new ControlsProfiles();
        }
        return out;
    }

    private static ControlsConfig readConfig(JsonNode node) {
        ControlsConfig cfg = ControlsConfig.defaults();
        if (node == null || !node.isObject()) {
            return cfg;
        }
        JsonNode enabledNode = node.get("enabled");
        for (ControlsField field : ControlsField.values()) {
            JsonNode value = node.get(field.key());
            if (value != null && value.isNumber()) {
                double raw = value.asDouble();
                // Before limits could be switched off, "off" was written as a value too large to
                // bind. Read those back as disabled rather than clamping them to the new maximum,
                // which would silently turn a dormant limit into an active one.
                if (field.canDisable() && raw >= ControlsField.LIMIT_DISABLED) {
                    cfg = cfg.withEnabled(field, false);
                } else {
                    cfg = cfg.with(field, raw);
                }
            }
            if (enabledNode != null && enabledNode.isObject()) {
                JsonNode flag = enabledNode.get(field.key());
                if (flag != null && flag.isBoolean()) {
                    cfg = cfg.withEnabled(field, flag.asBoolean());
                }
            }
        }
        JsonNode schemeNode = node.get("scheme");
        if (schemeNode != null && schemeNode.isTextual()) {
            cfg = cfg.withScheme(ControlScheme.fromName(schemeNode.asText()));
        }
        cfg = cfg.withTranslationCurve(readCurve(node.get("translationCurve")));
        cfg = cfg.withRotationCurve(readCurve(node.get("rotationCurve")));
        return cfg;
    }

    private static ControlsCurve readCurve(JsonNode node) {
        if (node == null || !node.isObject()) {
            return ControlsCurve.power();
        }
        JsonNode modeNode = node.get("mode");
        String mode = modeNode == null ? ControlsCurve.Mode.POWER.name() : modeNode.asText();
        JsonNode knotsNode = node.get("knots");
        if (knotsNode == null || !knotsNode.isArray()) {
            return ControlsCurve.of(mode, ControlsCurve.defaultKnots());
        }
        double[] knots = new double[knotsNode.size()];
        for (int i = 0; i < knotsNode.size(); i++) {
            knots[i] = knotsNode.get(i).asDouble();
        }
        return ControlsCurve.of(mode, knots);
    }

    private static void writeCurve(ObjectNode parent, String name, ControlsCurve curve) {
        ObjectNode node = parent.putObject(name);
        node.put("mode", curve.mode().name());
        ArrayNode knots = node.putArray("knots");
        for (double value : curve.knots()) {
            knots.add(value);
        }
    }

    /** Serialise this profile set as pretty-printed JSON suitable for checking into git. */
    public String toJson() {
        ObjectNode root = MAPPER.createObjectNode();
        root.put("active", active);
        ObjectNode profilesNode = root.putObject("profiles");
        profiles.forEach((name, cfg) -> {
            ObjectNode node = profilesNode.putObject(name);
            for (ControlsField field : ControlsField.values()) {
                node.put(field.key(), cfg.get(field));
            }
            ObjectNode enabledNode = node.putObject("enabled");
            for (ControlsField field : ControlsField.values()) {
                if (field.canDisable()) {
                    enabledNode.put(field.key(), cfg.isEnabled(field));
                }
            }
            node.put("scheme", cfg.scheme().name());
            writeCurve(node, "translationCurve", cfg.translationCurve());
            writeCurve(node, "rotationCurve", cfg.rotationCurve());
        });
        try {
            return MAPPER.writerWithDefaultPrettyPrinter().writeValueAsString(root) + "\n";
        } catch (Exception e) {
            System.err.println("[Controls] could not serialise profiles: " + e);
            return "{}\n";
        }
    }

}
