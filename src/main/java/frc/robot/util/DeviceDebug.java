package frc.robot.util;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Runtime debugging utility for exposing low-level device controls via NetworkTables.
 *
 * <p>
 * This class allows selected CAN devices to be interactively inspected and reconfigured at runtime
 * through NetworkTables (e.g. from Glass, Shuffleboard, or AdvantageScope). It is intended strictly
 * for debugging and tuning and should not be relied upon for normal robot operation.
 *
 * <h2>Design overview</h2>
 * <ul>
 * <li>Devices are explicitly registered by name before debugging is enabled.</li>
 * <li>No NetworkTables listeners or control hooks are installed until debugging is explicitly
 * enabled via {@code /Debug/Enable}.</li>
 * <li>Once enabled, this class publishes per-device debug topics and installs listeners to apply
 * configuration changes live.</li>
 * </ul>
 *
 * <h2>Safety model</h2>
 *
 * <p>
 * Debug functionality is gated behind an explicit NetworkTables boolean ({@code /Debug/Enable}) to
 * prevent accidental configuration changes during normal operation. Once enabled, the enable
 * listener is removed and debugging remains active for the lifetime of the program.
 *
 * <p>
 * This class is <b>not thread-safe</b> and is expected to be initialized and used during robot
 * startup before devices are actively commanded.
 *
 * <p>
 * Support for additional device types and parameters (e.g. CANcoders, current limits, inversion,
 * soft limits) may be added in the future.
 */
public final class DeviceDebug {

    private DeviceDebug() {}

    private static final Map<String, TalonFX> talonFXs = new HashMap<>();
    private static final Map<String, CANcoder> cancoders = new HashMap<>();

    private static int enableListener;

    /**
     * Initializes the debug system and installs the global enable listener.
     *
     * <p>
     * This method publishes {@code /Debug/Enable} (default {@code false}) and waits for it to be
     * set {@code true}. When enabled, all registered devices will have their debug topics created
     * and listeners installed.
     *
     * <p>
     * This method should be called exactly once during robot initialization.
     */
    public static void initialize() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        var enableTopic = instance.getBooleanTopic("/Debug/Enable");
        enableTopic.publish().accept(false);
        enableListener = instance.addListener(enableTopic.subscribe(false),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll), (ev) -> {
                if (ev.valueData.value.getBoolean()) {
                    setupDebug(instance);
                }
            });
    }

    private static void setupDebug(NetworkTableInstance instance) {
        instance.removeListener(enableListener);
        for (var kv : talonFXs.entrySet()) {
            String key = "/Debug/TalonFX/" + kv.getKey();

            var brakeModeTopic = instance.getBooleanTopic(key + "/Brake");
            TalonFX talon = kv.getValue();
            boolean isBrake = getBrakeMode(talon);
            var publisher = brakeModeTopic.publish();
            publisher.accept(isBrake);
            instance.addListener(brakeModeTopic.subscribe(isBrake),
                EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                    setBrakeMode(talon, ev.valueData.value.getBoolean());
                    boolean isBrake_ = getBrakeMode(talon);
                    publisher.accept(isBrake_);
                });
        }
    }

    /**
     * Registers a TalonFX for runtime debugging.
     *
     * <p>
     * The provided name is used as part of the NetworkTables path:
     * {@code /Debug/TalonFX/&lt;name&gt;}.
     *
     * <p>
     * Registration must occur <em>before</em> debugging is enabled.
     *
     * @param name human-readable identifier for the device
     * @param talon TalonFX instance to expose for debugging
     */
    public static void register(String name, TalonFX talon) {
        talonFXs.put(name, talon);
    }

    /**
     * Registers a CANcoder for future runtime debugging.
     *
     * <p>
     * This method currently records the device but does not yet expose any debug controls. It
     * exists to allow future expansion without changing registration semantics.
     *
     * <p>
     * Registration must occur <em>before</em> debugging is enabled.
     *
     * @param name human-readable identifier for the device
     * @param cancoder CANcoder instance to register
     */
    public static void register(String name, CANcoder cancoder) {
        cancoders.put(name, cancoder);
    }

    private static boolean getBrakeMode(TalonFX talon) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        var configurator = talon.getConfigurator();
        configurator.refresh(config);
        return config.NeutralMode.value == NeutralModeValue.Brake.value;
    }

    private static void setBrakeMode(TalonFX talon, boolean enableBrake) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        var configurator = talon.getConfigurator();
        configurator.refresh(config);
        config.NeutralMode = enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        PhoenixSignals.tryUntilOk(5, () -> configurator.apply(config));
    }

}
