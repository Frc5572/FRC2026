package frc.robot.tuning;

import java.util.EnumMap;
import java.util.Map;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.config.ConfigStore;

/**
 * NetworkTables-backed drivetrain tuning, persisted beside the driver profiles.
 *
 * <h2>Topic layout</h2>
 *
 * <pre>
 * /Tuning/meta/&lt;key&gt;/{default,min,max,units}  robot -&gt; UI, published once at startup
 * /Tuning/values/&lt;key&gt;                        robot -&gt; UI, the values in force
 * /Tuning/moduleOffsets                        robot -&gt; UI, captured azimuth offsets
 * /Tuning/set/values/&lt;key&gt;                    UI -&gt; robot, requested values
 * /Tuning/procedure                            UI -&gt; robot, the procedure to run
 * /Tuning/command/{save,reload}                UI -&gt; robot, momentary; robot clears them
 * /Tuning/status/{dirty,message,path}          robot -&gt; UI
 * </pre>
 *
 * <p>
 * The two directions use separate topics for the reasons documented on
 * {@code ControlsReal}: the robot must republish its authoritative values so a disconnecting
 * tuner cannot revert them, and republishing on a topic the tuner also writes would clobber the
 * tuner's edits before they were ever read.
 */
@NullMarked
public class TuningReal implements TuningIO {

    /** Name of this subsystem's document in the shared configuration directory. */
    public static final String FILE_NAME = "tuning.json";

    private static final String ROOT = "Tuning";

    private final ConfigStore store;
    private DrivetrainTuning config;

    private final Map<TuningField, DoublePublisher> valuePublishers =
        new EnumMap<>(TuningField.class);
    private final Map<TuningField, DoubleSubscriber> valueRequests =
        new EnumMap<>(TuningField.class);
    private final Map<TuningField, Double> lastRequest = new EnumMap<>(TuningField.class);

    private final DoubleArrayPublisher offsetsPublisher;
    private final StringSubscriber procedureRequest;
    private final BooleanEntry saveCommand;
    private final BooleanEntry reloadCommand;
    private final BooleanPublisher dirtyPublisher;
    private final StringPublisher messagePublisher;
    private final StringPublisher pathPublisher;

    private boolean dirty = false;

    /** Wire up the topics and load the stored tuning. */
    public TuningReal() {
        this.store = ConfigStore.getInstance();
        String json = store.load(FILE_NAME);
        this.config = json == null ? DrivetrainTuning.defaults() : DrivetrainTuning.fromJson(json);

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable table = nt.getTable(ROOT);
        NetworkTable meta = table.getSubTable("meta");
        NetworkTable values = table.getSubTable("values");
        NetworkTable setValues = table.getSubTable("set").getSubTable("values");

        for (TuningField field : TuningField.values()) {
            NetworkTable fieldMeta = meta.getSubTable(field.key());
            fieldMeta.getDoubleTopic("default").publish().set(field.defaultValue());
            fieldMeta.getDoubleTopic("min").publish().set(field.minimum());
            fieldMeta.getDoubleTopic("max").publish().set(field.maximum());
            fieldMeta.getStringTopic("units").publish().set(field.units());

            double value = config.get(field);
            DoublePublisher publisher = values.getDoubleTopic(field.key()).publish();
            publisher.set(value);
            valuePublishers.put(field, publisher);
            valueRequests.put(field, setValues.getDoubleTopic(field.key()).subscribe(value));
            lastRequest.put(field, value);
        }

        offsetsPublisher = table.getDoubleArrayTopic("moduleOffsets").publish();
        offsetsPublisher.set(config.moduleOffsets());

        // Subscribe only. The page owns this topic: it sets the name while its button is held
        // and clears it on release, so the robot must never write here.
        procedureRequest = table.getStringTopic("procedure").subscribe("");

        NetworkTable command = table.getSubTable("command");
        saveCommand = command.getBooleanTopic("save").getEntry(false);
        saveCommand.set(false);
        reloadCommand = command.getBooleanTopic("reload").getEntry(false);
        reloadCommand.set(false);

        NetworkTable status = table.getSubTable("status");
        dirtyPublisher = status.getBooleanTopic("dirty").publish();
        dirtyPublisher.set(false);
        messagePublisher = status.getStringTopic("message").publish();
        pathPublisher = status.getStringTopic("path").publish();
        pathPublisher.set(store.file(FILE_NAME).toString());
        setMessage("loaded drivetrain tuning");
    }

    @Override
    public void updateInputs(TuningInputs inputs) {
        handleValueRequests();
        handleCommands();

        inputs.values = config.toArray();
        inputs.moduleOffsets = config.moduleOffsets();
        inputs.requestedProcedure = procedureRequest.get("");
        inputs.dirty = dirty;
        dirtyPublisher.set(dirty);
    }

    private void handleValueRequests() {
        boolean changed = false;
        for (TuningField field : TuningField.values()) {
            double requested = valueRequests.get(field).get(config.get(field));
            if (requested != lastRequest.get(field)) {
                lastRequest.put(field, requested);
                config = config.with(field, requested);
                changed = true;
            }
        }
        if (changed) {
            dirty = true;
        }
        for (TuningField field : TuningField.values()) {
            valuePublishers.get(field).set(config.get(field));
        }
    }

    private void handleCommands() {
        if (saveCommand.get(false)) {
            saveCommand.set(false);
            if (store.save(FILE_NAME, config.toJson())) {
                dirty = false;
                setMessage("saved to " + store.file(FILE_NAME));
            } else {
                setMessage("save failed: " + store.lastError());
            }
        }
        if (reloadCommand.get(false)) {
            reloadCommand.set(false);
            String json = store.load(FILE_NAME);
            config = json == null ? DrivetrainTuning.defaults() : DrivetrainTuning.fromJson(json);
            dirty = false;
            pushToNetworkTables();
            setMessage("reloaded from disk");
        }
    }

    @Override
    public void reportValue(TuningField field, double value) {
        config = config.with(field, value);
        dirty = true;
        valuePublishers.get(field).set(config.get(field));
        lastRequest.put(field, valueRequests.get(field).get(config.get(field)));
        setMessage("measured " + field.key() + " = " + config.get(field));
    }

    @Override
    public void reportModuleOffsets(double[] offsets) {
        config = config.withModuleOffsets(offsets);
        dirty = true;
        offsetsPublisher.set(config.moduleOffsets());
        setMessage("captured module offsets");
    }

    /**
     * Republish everything and treat whatever the tuner currently requests as already seen, so a
     * page left open cannot immediately re-apply the values that were just replaced.
     */
    private void pushToNetworkTables() {
        for (TuningField field : TuningField.values()) {
            double value = config.get(field);
            valuePublishers.get(field).set(value);
            lastRequest.put(field, valueRequests.get(field).get(value));
        }
        offsetsPublisher.set(config.moduleOffsets());
    }

    private void setMessage(String text) {
        messagePublisher.set(text);
        System.out.println("[Tuning] " + text);
    }

}
