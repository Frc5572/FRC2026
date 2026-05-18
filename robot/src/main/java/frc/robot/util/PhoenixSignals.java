package frc.robot.util;

import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

/**
 * Utility class that centralizes registration and synchronized refreshing of CTRE Phoenix
 * {@link BaseStatusSignal}s.
 *
 * <p>
 * Grouping signals and refreshing them together ensures that all sampled values share the same
 * synchronization fence, minimizing timestamp skew between related measurements.
 *
 * <p>
 * Signals are maintained in separate groups depending on whether they are sourced from a CANivore
 * bus or the roboRIO CAN bus, as Phoenix requires refresh calls to be performed per bus.
 *
 * <p>
 * This class is purely static and is not intended to be instantiated.
 */
@NullMarked
public class PhoenixSignals {

    private PhoenixSignals() {}

    /**
     * Repeatedly executes a Phoenix command until it succeeds or the maximum number of attempts is
     * reached.
     *
     * <p>
     * This is useful for Phoenix configuration calls that may transiently fail during startup or
     * bus contention.
     *
     * @param maxAttempts maximum number of attempts before giving up
     * @param command supplier that executes the Phoenix command and returns a {@link StatusCode}
     */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) {
                break;
            }
            System.err.println("Phoenix Error: " + error.toString());
        }
    }

    /** Signals registered on the CANivore bus for synchronized refresh. */
    private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];
    /** Signals registered on the roboRIO CAN bus for synchronized refresh. */
    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

    /**
     * Registers one or more Phoenix signals for synchronized refreshing.
     *
     * <p>
     * All registered signals will be refreshed together when {@link #refreshAll()} is called,
     * ensuring consistent timestamps across related measurements.
     *
     * <p>
     * Signals must be registered to the correct bus (CANivore vs roboRIO). Mixing buses in a single
     * refresh call is not supported by Phoenix.
     *
     * <p>
     * This method should typically be called during subsystem construction or robot initialization,
     * before periodic refreshes occur.
     *
     * @param canivore {@code true} if the signals are on the CANivore bus, {@code false} if they
     *        are on the roboRIO CAN bus
     * @param signals one or more {@link BaseStatusSignal}s to register
     */
    public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
        if (canivore) {
            BaseStatusSignal[] newSignals =
                new BaseStatusSignal[canivoreSignals.length + signals.length];
            System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
            System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
            canivoreSignals = newSignals;
        } else {
            BaseStatusSignal[] newSignals =
                new BaseStatusSignal[rioSignals.length + signals.length];
            System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
            System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
            rioSignals = newSignals;
        }
    }

    /**
     * Refreshes all registered Phoenix signals.
     *
     * <p>
     * This method should be called once per control loop to update all registered signals in a
     * synchronized manner.
     *
     * <p>
     * Signals on the CANivore and roboRIO buses are refreshed separately, but each group is
     * internally synchronized.
     */
    public static void refreshAll() {
        if (canivoreSignals.length > 0) {
            BaseStatusSignal.refreshAll(canivoreSignals);
        }
        if (rioSignals.length > 0) {
            BaseStatusSignal.refreshAll(rioSignals);
        }
    }

}
