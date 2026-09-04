package frc.robot.subsystems.vision.ofl;

import org.littletonrobotics.junction.AutoLog;

public interface oflIO {

    @AutoLog
    public static class oflInputs {
        public double xMeters = 0.0;
        public double yMeters = 0.0;
        public double thetaDeg = 0.0;

        public double deltaXMeters = 0.0;
        public double deltaYMeters = 0.0;
        public double deltaThetaDeg = 0.0;

        public int inliers = 0;
        public double confidence = 0.0;
        public boolean valid = false;

        public long timestampUs = 0;
        public boolean jetsonConnected = false;
    }

    default void updateInputs(oflInputs inputs) {}
}
