package frc.robot.subsystems.vision.ofl;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ofl extends SubsystemBase {

    private final oflIO io;

    private final oflInputsAutoLogged inputs = new oflInputsAutoLogged();

    public ofl(oflIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("VisualOdometry", inputs);
    }

    public double getX() {
        return inputs.xMeters;
    }

    public double getY() {
        return inputs.yMeters;
    }

    public double getThetaDeg() {
        return inputs.thetaDeg;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(inputs.thetaDeg);
    }

    public Pose2d getPose() {
        return new Pose2d(inputs.xMeters, inputs.yMeters, getRotation());
    }

    public double getDeltaX() {
        return inputs.deltaXMeters;
    }

    public double getDeltaY() {
        return inputs.deltaYMeters;
    }

    public double getDeltaThetaDeg() {
        return inputs.deltaThetaDeg;
    }

    public int getInliers() {
        return inputs.inliers;
    }

    public double getConfidence() {
        return inputs.confidence;
    }

    public boolean isValid() {
        return inputs.valid;
    }

    public long getTimestampUs() {
        return inputs.timestampUs;
    }

    public boolean isJetsonConnected() {
        return inputs.jetsonConnected;
    }

    /**
     * True when the Jetson reports a connection and the current VO frame passed validation.
     */
    public boolean hasValidMeasurement() {
        return inputs.jetsonConnected && inputs.valid;
    }
}
