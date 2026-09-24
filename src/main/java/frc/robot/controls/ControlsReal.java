package frc.robot.controls;

import java.util.EnumMap;
import java.util.Map;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.config.ConfigStore;

/**
 * NetworkTables-backed control tuning, persisted to the robot's USB stick.
 *
 * <h2>Topic layout</h2>
 *
 * <pre>
 * /Controls/meta/&lt;key&gt;/{default,min,max}  robot -&gt; UI, published once at startup
 * /Controls/values/&lt;key&gt;                  robot -&gt; UI, the active profile's values
 * /Controls/enabled/&lt;key&gt;                 robot -&gt; UI, which limits take effect
 * /Controls/curves/&lt;which&gt;{Mode,Knots}    robot -&gt; UI, the active response curves
 * /Controls/set/values/&lt;key&gt;              UI -&gt; robot, requested values
 * /Controls/set/enabled/&lt;key&gt;             UI -&gt; robot, requested on/off
 * /Controls/set/curves/&lt;which&gt;{Mode,Knots} UI -&gt; robot, requested curves
 * /Controls/scheme                         robot -&gt; UI, the active binding scheme
 * /Controls/schemes                        robot -&gt; UI, every scheme name and description
 * /Controls/set/scheme                     UI -&gt; robot, requested binding scheme
 * /Controls/activeProfile                  bidirectional, which profile is selected
 * /Controls/profiles                       robot -&gt; UI, every known profile name
 * /Controls/command/{save,reload}          UI -&gt; robot, momentary; robot clears them
 * /Controls/command/{create,delete}        UI -&gt; robot, profile name; robot clears them
 * /Controls/status/{dirty,message,path}    robot -&gt; UI
 * </pre>
 *
 * <h2>Why the two directions use separate topics</h2> A single bidirectional topic per value
 * cannot be made reliable here. The robot must republish its authoritative value, because a
 * NetworkTables topic drops a disconnecting client's value and falls back to whatever other
 * publishers last sent &mdash; so a tuner that closes would otherwise revert the robot to a stale
 * startup value, and the robot would read that back as a fresh edit. But republishing on the same
 * topic the tuner writes to means the robot overwrites the tuner's value before it ever reads it,
 * and edits are lost instead.
 *
 * <p>
 * Splitting them removes the race. The robot owns {@code values} and republishes freely; the
 * tuner owns {@code set} and the robot only ever reads it. When the tuner disconnects its topics
 * are unannounced, so the subscriber falls back to the default this class supplies &mdash; the
 * current value &mdash; and nothing changes.
 *
 * <p>
 * Every topic is declared explicitly rather than discovered by reflection, so the wire format is
 * readable from this file alone.
 */
@NullMarked
public class ControlsReal implements ControlsIO {

    /** Name of this subsystem's document in the shared configuration directory. */
    public static final String FILE_NAME = "profiles.json";

    private static final String ROOT = "Controls";

    private final ConfigStore store;
    private ControlsProfiles profiles;

    private final Map<ControlsField, DoublePublisher> valuePublishers =
        new EnumMap<>(ControlsField.class);
    private final Map<ControlsField, DoubleSubscriber> valueRequests =
        new EnumMap<>(ControlsField.class);
    private final Map<ControlsField, Double> lastRequest = new EnumMap<>(ControlsField.class);
    private final Map<ControlsField, BooleanPublisher> enabledPublishers =
        new EnumMap<>(ControlsField.class);
    private final Map<ControlsField, BooleanSubscriber> enabledRequests =
        new EnumMap<>(ControlsField.class);
    private final Map<ControlsField, Boolean> lastEnabledRequest =
        new EnumMap<>(ControlsField.class);

    private final StringEntry activeEntry;
    private final StringArrayPublisher profilesPublisher;
    private final StringPublisher schemePublisher;
    private final StringSubscriber schemeRequest;
    private ControlScheme lastSchemeRequest;

    private final StringPublisher translationCurveMode;
    private final DoubleArrayPublisher translationCurveKnots;
    private final StringPublisher rotationCurveMode;
    private final DoubleArrayPublisher rotationCurveKnots;
    private final StringSubscriber translationModeRequest;
    private final DoubleArraySubscriber translationKnotsRequest;
    private final StringSubscriber rotationModeRequest;
    private final DoubleArraySubscriber rotationKnotsRequest;
    private ControlsCurve lastTranslationRequest;
    private ControlsCurve lastRotationRequest;

    private final BooleanEntry saveCommand;
    private final BooleanEntry reloadCommand;
    private final StringEntry createCommand;
    private final StringEntry deleteCommand;
    private final BooleanPublisher dirtyPublisher;
    private final StringPublisher messagePublisher;
    private final StringPublisher pathPublisher;

    private boolean dirty = false;
    private String message = "";
    private int curveReassert = 0;

    /** Wire up the topics, load the stored profiles, and start the controls web server. */
    public ControlsReal() {
        this.store = ConfigStore.getInstance();
        this.profiles = loadProfiles();

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable table = nt.getTable(ROOT);
        NetworkTable meta = table.getSubTable("meta");
        NetworkTable values = table.getSubTable("values");
        NetworkTable setValues = table.getSubTable("set").getSubTable("values");
        NetworkTable enabledTable = table.getSubTable("enabled");
        NetworkTable setEnabled = table.getSubTable("set").getSubTable("enabled");
        ControlsConfig active = profiles.active();

        for (ControlsField field : ControlsField.values()) {
            NetworkTable fieldMeta = meta.getSubTable(field.key());
            fieldMeta.getDoubleTopic("default").publish().set(field.defaultValue());
            fieldMeta.getDoubleTopic("min").publish().set(field.minimum());
            fieldMeta.getDoubleTopic("max").publish().set(field.maximum());
            fieldMeta.getBooleanTopic("canDisable").publish().set(field.canDisable());

            double value = active.get(field);
            DoublePublisher publisher = values.getDoubleTopic(field.key()).publish();
            publisher.set(value);
            valuePublishers.put(field, publisher);
            valueRequests.put(field, setValues.getDoubleTopic(field.key()).subscribe(value));
            lastRequest.put(field, value);

            boolean on = active.isEnabled(field);
            BooleanPublisher enabledPublisher =
                enabledTable.getBooleanTopic(field.key()).publish();
            enabledPublisher.set(on);
            enabledPublishers.put(field, enabledPublisher);
            enabledRequests.put(field, setEnabled.getBooleanTopic(field.key()).subscribe(on));
            lastEnabledRequest.put(field, on);
        }

        activeEntry = table.getStringTopic("activeProfile").getEntry(profiles.activeName());
        activeEntry.set(profiles.activeName());
        profilesPublisher = table.getStringArrayTopic("profiles").publish();

        lastSchemeRequest = active.scheme();
        schemePublisher = table.getStringTopic("scheme").publish();
        schemePublisher.set(lastSchemeRequest.name());
        schemeRequest = table.getSubTable("set").getStringTopic("scheme")
            .subscribe(lastSchemeRequest.name());
        String[] schemeInfo = new String[ControlScheme.values().length];
        for (int i = 0; i < schemeInfo.length; i++) {
            ControlScheme s = ControlScheme.values()[i];
            schemeInfo[i] = s.name() + "|" + s.label() + "|" + s.summary();
        }
        table.getStringArrayTopic("schemes").publish().set(schemeInfo);

        NetworkTable curves = table.getSubTable("curves");
        NetworkTable setCurves = table.getSubTable("set").getSubTable("curves");
        lastTranslationRequest = active.translationCurve();
        lastRotationRequest = active.rotationCurve();
        translationCurveMode = curves.getStringTopic("translationMode").publish();
        translationCurveKnots = curves.getDoubleArrayTopic("translationKnots").publish();
        rotationCurveMode = curves.getStringTopic("rotationMode").publish();
        rotationCurveKnots = curves.getDoubleArrayTopic("rotationKnots").publish();
        translationModeRequest = setCurves.getStringTopic("translationMode")
            .subscribe(lastTranslationRequest.mode().name());
        translationKnotsRequest = setCurves.getDoubleArrayTopic("translationKnots")
            .subscribe(lastTranslationRequest.knots());
        rotationModeRequest = setCurves.getStringTopic("rotationMode")
            .subscribe(lastRotationRequest.mode().name());
        rotationKnotsRequest = setCurves.getDoubleArrayTopic("rotationKnots")
            .subscribe(lastRotationRequest.knots());
        publishCurves();

        NetworkTable command = table.getSubTable("command");
        saveCommand = command.getBooleanTopic("save").getEntry(false);
        saveCommand.set(false);
        reloadCommand = command.getBooleanTopic("reload").getEntry(false);
        reloadCommand.set(false);
        createCommand = command.getStringTopic("create").getEntry("");
        createCommand.set("");
        deleteCommand = command.getStringTopic("delete").getEntry("");
        deleteCommand.set("");

        NetworkTable status = table.getSubTable("status");
        dirtyPublisher = status.getBooleanTopic("dirty").publish();
        dirtyPublisher.set(false);
        messagePublisher = status.getStringTopic("message").publish();
        pathPublisher = status.getStringTopic("path").publish();

        publishProfileList();
        pathPublisher.set(store.file(FILE_NAME).toString());
        setMessage("loaded " + profiles.names().size() + " profile(s)");
    }

    /** Read the stored profiles, falling back to factory defaults when there are none. */
    private ControlsProfiles loadProfiles() {
        String json = store.load(FILE_NAME);
        return json == null ? new ControlsProfiles() : ControlsProfiles.fromJson(json);
    }

    @Override
    public void updateInputs(ControlsInputs inputs) {
        handleProfileSwitch();
        handleValueRequests();
        handleCurveRequests();
        handleCommands();

        ControlsConfig active = profiles.active();
        inputs.activeProfile = profiles.activeName();
        inputs.values = active.toArray();
        inputs.enabled = active.enabledFlags();
        inputs.scheme = active.scheme().name();
        inputs.translationCurveMode = active.translationCurve().mode().name();
        inputs.translationCurveKnots = active.translationCurve().knots();
        inputs.rotationCurveMode = active.rotationCurve().mode().name();
        inputs.rotationCurveKnots = active.rotationCurve().knots();
        inputs.dirty = dirty;
        dirtyPublisher.set(dirty);
    }

    @Override
    public void requestValue(ControlsField field, double value) {
        ControlsConfig updated = profiles.active().with(field, value);
        profiles.putActive(updated);
        dirty = true;
        double applied = updated.get(field);
        valuePublishers.get(field).set(applied);
        // Treat whatever the tuner currently asks for as already seen, so a page left open does
        // not immediately undo a value the robot set itself.
        lastRequest.put(field, valueRequests.get(field).get(applied));
    }

    @Override
    public void requestEnabled(ControlsField field, boolean enabled) {
        ControlsConfig updated = profiles.active().withEnabled(field, enabled);
        profiles.putActive(updated);
        dirty = true;
        boolean applied = updated.isEnabled(field);
        enabledPublishers.get(field).set(applied);
        lastEnabledRequest.put(field, enabledRequests.get(field).get(applied));
    }

    private void handleProfileSwitch() {
        String requested = activeEntry.get(profiles.activeName());
        if (requested.equals(profiles.activeName())) {
            return;
        }
        if (profiles.setActive(requested)) {
            adoptActiveProfile();
            setMessage("switched to '" + requested + "'");
        } else {
            activeEntry.set(profiles.activeName());
            setMessage("no profile named '" + requested + "'");
        }
    }

    /**
     * Apply an operator's requested values, then publish the authoritative result.
     *
     * <p>
     * A request only counts when it differs from the last one seen, so a tuner left open holding
     * a stale value cannot keep overriding a profile switch.
     */
    private void handleValueRequests() {
        ControlsConfig config = profiles.active();
        boolean changed = false;
        for (ControlsField field : ControlsField.values()) {
            double requested = valueRequests.get(field).get(config.get(field));
            if (requested != lastRequest.get(field)) {
                lastRequest.put(field, requested);
                config = config.with(field, requested);
                changed = true;
            }
            if (!field.canDisable()) {
                continue;
            }
            boolean on = enabledRequests.get(field).get(config.isEnabled(field));
            if (on != lastEnabledRequest.get(field)) {
                lastEnabledRequest.put(field, on);
                config = config.withEnabled(field, on);
                changed = true;
            }
        }
        ControlScheme scheme = ControlScheme.fromName(schemeRequest.get(config.scheme().name()));
        if (scheme != lastSchemeRequest) {
            lastSchemeRequest = scheme;
            config = config.withScheme(scheme);
            changed = true;
        }
        if (changed) {
            profiles.putActive(config);
            dirty = true;
        }
        for (ControlsField field : ControlsField.values()) {
            valuePublishers.get(field).set(config.get(field));
            enabledPublishers.get(field).set(config.isEnabled(field));
        }
        schemePublisher.set(config.scheme().name());
    }

    /** Apply a requested response curve, on the same "only when it changes" rule. */
    private void handleCurveRequests() {
        ControlsConfig config = profiles.active();
        boolean changed = false;

        ControlsCurve translation = ControlsCurve.of(
            translationModeRequest.get(lastTranslationRequest.mode().name()),
            translationKnotsRequest.get(lastTranslationRequest.knots()));
        if (!translation.equals(lastTranslationRequest)) {
            lastTranslationRequest = translation;
            config = config.withTranslationCurve(translation);
            changed = true;
        }

        ControlsCurve rotation = ControlsCurve.of(
            rotationModeRequest.get(lastRotationRequest.mode().name()),
            rotationKnotsRequest.get(lastRotationRequest.knots()));
        if (!rotation.equals(lastRotationRequest)) {
            lastRotationRequest = rotation;
            config = config.withRotationCurve(rotation);
            changed = true;
        }

        if (changed) {
            profiles.putActive(config);
            dirty = true;
        }
        // Curve publishes allocate, so re-assert periodically rather than every cycle.
        if (changed || ++curveReassert >= 25) {
            curveReassert = 0;
            publishCurves();
        }
    }

    private void handleCommands() {
        if (saveCommand.get(false)) {
            saveCommand.set(false);
            if (store.save(FILE_NAME, profiles.toJson())) {
                dirty = false;
                setMessage("saved to " + store.file(FILE_NAME));
            } else {
                setMessage("save failed: " + store.lastError());
            }
        }
        if (reloadCommand.get(false)) {
            reloadCommand.set(false);
            profiles = loadProfiles();
            dirty = false;
            adoptActiveProfile();
            publishProfileList();
            activeEntry.set(profiles.activeName());
            setMessage("reloaded from disk");
        }
        String create = createCommand.get("");
        if (!create.isEmpty()) {
            createCommand.set("");
            profiles.put(create, profiles.active());
            dirty = true;
            adoptActiveProfile();
            publishProfileList();
            activeEntry.set(profiles.activeName());
            setMessage("created '" + create + "' from the active profile");
        }
        String delete = deleteCommand.get("");
        if (!delete.isEmpty()) {
            deleteCommand.set("");
            if (profiles.remove(delete)) {
                dirty = true;
                adoptActiveProfile();
                publishProfileList();
                activeEntry.set(profiles.activeName());
                setMessage("deleted '" + delete + "'");
            } else {
                setMessage("cannot delete '" + delete + "'");
            }
        }
    }

    /**
     * Take whatever the tuner is currently requesting as already seen, and publish the new active
     * profile. Without this snapshot a tuner left open would immediately re-apply the values of
     * the profile the operator just switched away from.
     */
    private void adoptActiveProfile() {
        ControlsConfig config = profiles.active();
        for (ControlsField field : ControlsField.values()) {
            double value = config.get(field);
            lastRequest.put(field, valueRequests.get(field).get(value));
            valuePublishers.get(field).set(value);
            boolean on = config.isEnabled(field);
            lastEnabledRequest.put(field, enabledRequests.get(field).get(on));
            enabledPublishers.get(field).set(on);
        }
        lastSchemeRequest = ControlScheme.fromName(schemeRequest.get(config.scheme().name()));
        schemePublisher.set(config.scheme().name());
        lastTranslationRequest = ControlsCurve.of(
            translationModeRequest.get(config.translationCurve().mode().name()),
            translationKnotsRequest.get(config.translationCurve().knots()));
        lastRotationRequest = ControlsCurve.of(
            rotationModeRequest.get(config.rotationCurve().mode().name()),
            rotationKnotsRequest.get(config.rotationCurve().knots()));
        publishCurves();
    }

    private void publishCurves() {
        ControlsConfig config = profiles.active();
        translationCurveMode.set(config.translationCurve().mode().name());
        translationCurveKnots.set(config.translationCurve().knots());
        rotationCurveMode.set(config.rotationCurve().mode().name());
        rotationCurveKnots.set(config.rotationCurve().knots());
    }

    private void publishProfileList() {
        profilesPublisher.set(profiles.names().toArray(new String[0]));
    }

    private void setMessage(String text) {
        this.message = text;
        messagePublisher.set(text);
        System.out.println("[Controls] " + text);
    }

    /** The most recent status message shown to the operator. */
    public String message() {
        return message;
    }

}
