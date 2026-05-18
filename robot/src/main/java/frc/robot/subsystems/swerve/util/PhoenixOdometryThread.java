package frc.robot.subsystems.swerve.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.NullMarked;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

/**
 * Background thread responsible for sampling odometry-related signals at a higher frequency than
 * the main robot loop.
 *
 * <p>
 * This thread is primarily intended to reduce latency and aliasing when collecting drivetrain
 * sensor data (e.g., wheel encoders, gyro rates) used for odometry and state estimation.
 *
 * <p>
 * Two categories of signals may be registered:
 * <ul>
 * <li><b>Phoenix signals</b> - {@link StatusSignal} instances that can be synchronized and
 * timestamped by the CTRE Phoenix framework</li>
 * <li><b>Generic signals</b> - arbitrary {@link DoubleSupplier}s sampled at the same rate as
 * Phoenix signals</li>
 * </ul>
 *
 * <p>
 * Each registered signal is associated with a thread-safe {@link Queue} into which samples are
 * pushed. Consumers are expected to poll these queues from the main loop while holding the provided
 * odometry lock.
 *
 * <p>
 * If running on a CAN-FD network, Phoenix signals are synchronized using
 * {@link BaseStatusSignal#waitForAll(double, BaseStatusSignal...)}, allowing all signals to be
 * sampled at nearly the same timestamp. On non-CAN-FD networks, the thread sleeps for the
 * configured odometry period and explicitly refreshes signals.
 *
 * <p>
 * The thread runs as a daemon and will terminate automatically when the JVM exits.
 */
@NullMarked
public class PhoenixOdometryThread extends Thread {

    /** Lock shared with odometry consumers to ensure consistent sampling */
    private final Lock odometryLock;

    /** Lock protecting signal registration and Phoenix signal refresh */
    private final Lock signalsLock = new ReentrantLock();

    /** Registered Phoenix status signals */
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    /** Registered non-Phoenix signal suppliers */
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    /** Output queues corresponding to Phoenix signals */
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    /** Output queues corresponding to generic signals */
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    /** Queues receiving timestamps associated with each sample */
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    /** Whether the active CAN bus supports CAN-FD synchronization */
    private final boolean isCANFD = new CANBus("*").isNetworkFD();

    /**
     * Creates a new odometry sampling thread.
     *
     * @param odometryLock lock shared with consumers to ensure that sampled values and timestamps
     *        are read atomically
     */
    public PhoenixOdometryThread(Lock odometryLock) {
        this.odometryLock = odometryLock;
        this.setName("PhoenixOdometryThread");
        this.setDaemon(true);
    }

    /**
     * Starts the thread if at least one signal has been registered.
     *
     * <p>
     * This prevents spawning an idle background thread when no signals are being sampled.
     */
    @Override
    public synchronized void start() {
        // Only start the thread if stuff is actually registered.
        if (phoenixQueues.size() > 0 || genericQueues.size() > 0) {
            super.start();
        }
    }

    /**
     * Registers a Phoenix {@link StatusSignal} to be sampled by the thread.
     *
     * <p>
     * The returned queue will receive one value per odometry tick.
     *
     * <p>
     * This method is thread-safe and may be called before the thread starts.
     *
     * @param signal Phoenix status signal to sample
     * @return queue containing sampled signal values
     */
    public Queue<Double> registerSignal(StatusSignal<?> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        odometryLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            odometryLock.unlock();
        }
        return queue;
    }

    /**
     * Registers a generic value supplier to be sampled at the odometry rate.
     *
     * <p>
     * This is useful for signals not managed by Phoenix (e.g., WPILib sensors or computed values).
     *
     * @param signal supplier producing the value to sample
     * @return queue containing sampled values
     */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            signalsLock.unlock();
            odometryLock.unlock();
        }
        return queue;
    }

    /**
     * Creates a queue that receives timestamps corresponding to each sampling cycle.
     *
     * <p>
     * The timestamp represents FPGA time in seconds, adjusted for average Phoenix signal latency
     * when applicable.
     *
     * @return queue of sample timestamps (seconds)
     */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            odometryLock.unlock();
        }
        return queue;
    }

    /**
     * Main sampling loop.
     *
     * <p>
     * The thread runs indefinitely, sampling all registered signals at
     * {@code Constants.Swerve.odometryFrequency}.
     *
     * <p>
     * Sampling behavior differs depending on CAN capabilities:
     * <ul>
     * <li><b>CAN-FD:</b> waits for all Phoenix signals to update together</li>
     * <li><b>Non-CAN-FD:</b> sleeps for the target period and manually refreshes</li>
     * </ul>
     *
     * <p>
     * All sampled values and timestamps are enqueued atomically under the odometry lock to ensure
     * consistency for consumers.
     */
    @Override
    public void run() {
        while (true) {
            signalsLock.lock();
            try {
                if (isCANFD && phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(2.0 / Constants.Swerve.odometryFrequency,
                        phoenixSignals);
                } else {
                    Thread.sleep((long) (1000.0 / Constants.Swerve.odometryFrequency));
                    if (phoenixSignals.length > 0) {
                        BaseStatusSignal.refreshAll(phoenixSignals);
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            odometryLock.lock();
            try {
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }

                for (int i = 0; i < phoenixSignals.length; i++) {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            } finally {
                odometryLock.unlock();
            }
        }
    }

}
