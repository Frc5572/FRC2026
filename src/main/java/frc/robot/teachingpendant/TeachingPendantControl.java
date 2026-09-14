package frc.robot.teachingpendant;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Safe, test-mode-only manual-motion contract for the teaching pendant. */
public final class TeachingPendantControl {
    private static final double HEARTBEAT_TIMEOUT_SECONDS = 0.30;
    private final NetworkTable table =
        NetworkTableInstance.getDefault().getTable("/rosbots/TeachingPendant/Manual");
    private final BooleanSubscriber enabled = table.getBooleanTopic("Enabled").subscribe(false);
    private final DoubleSubscriber heartbeat = table.getDoubleTopic("Heartbeat").subscribe(0.0);
    private final DoubleSubscriber translationX =
        table.getDoubleTopic("TranslationX").subscribe(0.0);
    private final DoubleSubscriber translationY =
        table.getDoubleTopic("TranslationY").subscribe(0.0);
    private final DoubleSubscriber rotation = table.getDoubleTopic("Rotation").subscribe(0.0);
    private final BooleanSubscriber pushMode = table.getBooleanTopic("PushMode").subscribe(false);
    private double lastHeartbeatValue;
    private double lastHeartbeatArrival;

    public boolean active() {
        // The pendant and robot clocks are unrelated. Measure heartbeat freshness on the robot,
        // from the arrival of a changed NT value, rather than subtracting their timestamps.
        double receivedHeartbeat = heartbeat.get();
        if (receivedHeartbeat != lastHeartbeatValue) {
            lastHeartbeatValue = receivedHeartbeat;
            lastHeartbeatArrival = Timer.getFPGATimestamp();
        }
        return (DriverStation.isTestEnabled() || DriverStation.isTeleopEnabled()) && enabled.get()
            && Timer.getFPGATimestamp() - lastHeartbeatArrival <= HEARTBEAT_TIMEOUT_SECONDS;
    }

    public double x() {
        return clamp(translationX.get());
    }

    public double y() {
        return clamp(translationY.get());
    }

    public double rotation() {
        return clamp(rotation.get());
    }

    public boolean pushModeRequested() {
        return pushMode.get();
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
