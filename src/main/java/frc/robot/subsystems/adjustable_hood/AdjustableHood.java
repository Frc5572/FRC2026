package frc.robot.subsystems.adjustable_hood;

import static edu.wpi.first.units.Units.Degrees;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;

/**
 * Adjustable Hood Subsystem
 */
public class AdjustableHood extends SubsystemBase {

    private final AdjustableHoodIO io;
    public final AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();
    public RobotState state;
    private Translation2d robotPosition;
    private double hubDistance;
    private Angle goalAngle;

    private boolean isManualMode = false;
    private double manualAngleDegrees = 0.0;

    private InterpolatingDoubleTreeMap hoodAngles = new InterpolatingDoubleTreeMap();

    /**
     * Creates a new Adjustable Hood subsystem.
     *
     * @param io Hardware abstraction
     */
    public AdjustableHood(AdjustableHoodIO io) {
        super("Adjustable Hood");
        this.io = io;

        hoodAngles.put(0.0, 0.0);
        hoodAngles.put(1.0, 0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Adjustable Hood", inputs);

        this.robotPosition = state.getGlobalPoseEstimate().getTranslation();
        this.hubDistance = this.robotPosition.getDistance(FieldConstants.Hub.centerHub);

        double targetAngle;
        if (isManualMode) {
            targetAngle = manualAngleDegrees;
        } else {
            targetAngle = hoodAngles.get(this.hubDistance);
        }
        this.goalAngle = Degrees.of(targetAngle);

        io.setTargetAngle(this.goalAngle);

        Logger.recordOutput("Robot Position", robotPosition);
        Logger.recordOutput("Distance to Hub Center", this.hubDistance);
        Logger.recordOutput("Hood Angle", this.goalAngle);
        Logger.recordOutput("AdjustableHood/ActualAngle", inputs.relativeAngle);
    }

    /** Sets the angle manually */
    public void increaseManualAngle(Angle angleIncriment) {
        this.isManualMode = true;
        this.manualAngleDegrees += (angleIncriment.in(Degrees));
    }

    /** moves the hood by a specified increment */
    public Command manualMoveToAngle(Angle increment) {
        return runOnce(() -> this.increaseManualAngle(increment));
    }

    /** Uses the distance and angle tables */
    public Command useAutomaticTable() {
        return runOnce(() -> this.isManualMode = false);
    }
}
