package frc.robot.controls;

import java.io.IOException;
import java.nio.file.Files;
import java.util.EnumMap;
import java.util.Map;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.net.WebServer;

/**
 * NetworkTables-backed control tuning, persisted to the robot's USB stick.
 *
 * <h2>Topic layout</h2>
 *
 * <pre>
 * /Controls/meta/&lt;key&gt;/{default,min,max}  robot -&gt; UI, published once at startup
 * /Controls/values/&lt;key&gt;                  bidirectional, the active profile's values
 * /Controls/activeProfile                  bidirectional, which profile is selected
 * /Controls/profiles                       robot -&gt; UI, every known profile name
 * /Controls/command/{save,reload}          UI -&gt; robot, momentary; robot clears them
 * /Controls/command/{create,delete}        UI -&gt; robot, profile name; robot clears them
 * /Controls/status/{dirty,message,path}    robot -&gt; UI
 * </pre>
 *
 * <p>
 * Every topic is declared explicitly rather than discovered by reflection, so the wire format is
 * readable from this file alone. Because the robot both publishes and subscribes to the value
 * topics, each write is compared against the last value this class published; only a genuine
 * difference counts as an operator edit.
 */
@NullMarked
public class ControlsReal implements ControlsIO {

    /** Port serving the controls directory, so the build can pull profiles over HTTP. */
    public static final int WEB_PORT = 5801;

    private static final String ROOT = "Controls";

    private final ControlsStore store;
    private ControlsProfiles profiles;

    private final Map<ControlsField, DoubleEntry> valueEntries = new EnumMap<>(ControlsField.class);
    private final Map<ControlsField, Double> lastPublished = new EnumMap<>(ControlsField.class);

    private final StringEntry activeEntry;
    private final StringArrayPublisher profilesPublisher;
    private final BooleanEntry saveCommand;
    private final BooleanEntry reloadCommand;
    private final StringEntry createCommand;
    private final StringEntry deleteCommand;
    private final BooleanPublisher dirtyPublisher;
    private final StringPublisher messagePublisher;
    private final StringPublisher pathPublisher;

    private boolean dirty = false;
    private String message = "";

    /** Wire up the topics, load the stored profiles, and start the controls web server. */
    public ControlsReal() {
        this.store = new ControlsStore();
        this.profiles = store.load();

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable table = nt.getTable(ROOT);

        NetworkTable meta = table.getSubTable("meta");
        NetworkTable values = table.getSubTable("values");
        ControlsConfig active = profiles.active();
        for (ControlsField field : ControlsField.values()) {
            NetworkTable fieldMeta = meta.getSubTable(field.key());
            fieldMeta.getDoubleTopic("default").publish().set(field.defaultValue());
            fieldMeta.getDoubleTopic("min").publish().set(field.minimum());
            fieldMeta.getDoubleTopic("max").publish().set(field.maximum());

            DoubleEntry entry = values.getDoubleTopic(field.key()).getEntry(field.defaultValue());
            double value = active.get(field);
            entry.set(value);
            valueEntries.put(field, entry);
            lastPublished.put(field, value);
        }

        activeEntry = table.getStringTopic("activeProfile").getEntry(profiles.activeName());
        activeEntry.set(profiles.activeName());
        profilesPublisher = table.getStringArrayTopic("profiles").publish();

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
        pathPublisher.set(store.file().toString());
        setMessage("loaded " + profiles.names().size() + " profile(s)");
        startWebServer();
    }

    private void startWebServer() {
        try {
            Files.createDirectories(store.directory());
            WebServer.start(WEB_PORT, store.directory().toString());
            System.out.println(
                "[Controls] serving " + store.directory() + " on port " + WEB_PORT);
        } catch (IOException e) {
            System.err.println("[Controls] could not start controls web server: " + e);
        }
    }

    @Override
    public void updateInputs(ControlsInputs inputs) {
        handleProfileSwitch();
        handleValueEdits();
        handleCommands();

        ControlsConfig active = profiles.active();
        inputs.activeProfile = profiles.activeName();
        inputs.values = active.toArray();
        inputs.dirty = dirty;
        dirtyPublisher.set(dirty);
    }

    private void handleProfileSwitch() {
        String requested = activeEntry.get(profiles.activeName());
        if (requested.equals(profiles.activeName())) {
            return;
        }
        if (profiles.setActive(requested)) {
            pushActiveToNetworkTables();
            setMessage("switched to '" + requested + "'");
        } else {
            activeEntry.set(profiles.activeName());
            setMessage("no profile named '" + requested + "'");
        }
    }

    private void handleValueEdits() {
        ControlsConfig config = profiles.active();
        boolean changed = false;
        for (ControlsField field : ControlsField.values()) {
            double incoming = valueEntries.get(field).get(config.get(field));
            if (incoming == lastPublished.get(field)) {
                continue;
            }
            double clamped = field.clamp(incoming);
            config = config.with(field, clamped);
            lastPublished.put(field, clamped);
            if (clamped != incoming) {
                valueEntries.get(field).set(clamped);
            }
            changed = true;
        }
        if (changed) {
            profiles.putActive(config);
            dirty = true;
        }
    }

    private void handleCommands() {
        if (saveCommand.get(false)) {
            saveCommand.set(false);
            if (store.save(profiles)) {
                dirty = false;
                setMessage("saved to " + store.file());
            } else {
                setMessage("save failed: " + store.lastError());
            }
        }
        if (reloadCommand.get(false)) {
            reloadCommand.set(false);
            profiles = store.load();
            dirty = false;
            pushActiveToNetworkTables();
            publishProfileList();
            activeEntry.set(profiles.activeName());
            setMessage("reloaded from disk");
        }
        String create = createCommand.get("");
        if (!create.isEmpty()) {
            createCommand.set("");
            profiles.put(create, profiles.active());
            dirty = true;
            pushActiveToNetworkTables();
            publishProfileList();
            activeEntry.set(profiles.activeName());
            setMessage("created '" + create + "' from the active profile");
        }
        String delete = deleteCommand.get("");
        if (!delete.isEmpty()) {
            deleteCommand.set("");
            if (profiles.remove(delete)) {
                dirty = true;
                pushActiveToNetworkTables();
                publishProfileList();
                activeEntry.set(profiles.activeName());
                setMessage("deleted '" + delete + "'");
            } else {
                setMessage("cannot delete '" + delete + "'");
            }
        }
    }

    private void pushActiveToNetworkTables() {
        ControlsConfig config = profiles.active();
        for (ControlsField field : ControlsField.values()) {
            double value = config.get(field);
            valueEntries.get(field).set(value);
            lastPublished.put(field, value);
        }
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
