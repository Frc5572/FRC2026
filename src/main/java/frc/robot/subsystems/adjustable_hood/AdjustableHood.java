package frc.robot.subsystems.adjustable_hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotations;
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
    private final RobotState state;
    private Translation2d robotPosition = new Translation2d();
    private double hubDistance = 0.0;
    private Angle goalAngle = Degrees.of(0);

    private boolean isManualMode = false;
    private double manualAngleDegrees = 0.0;

    private InterpolatingDoubleTreeMap hoodAngles = new InterpolatingDoubleTreeMap();

    /**
     * Creates a new Adjustable Hood subsystem.
     *
     * @param io Hardware abstraction
     */
    public AdjustableHood(AdjustableHoodIO io, RobotState state) {
        super("Adjustable Hood");
        this.io = io;
        this.state = state;

        hoodAngles.put(0.0, 0.0);
        hoodAngles.put(1.0, 0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Adjustable Hood", inputs);

        var robotPose = state.getGlobalPoseEstimate();
        if (robotPose != null) {
            this.robotPosition = state.getGlobalPoseEstimate().getTranslation();
            this.hubDistance = this.robotPosition.getDistance(FieldConstants.Hub.centerHub);
        }

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
        Logger.recordOutput("Hood Angle (Rad)", this.goalAngle.in(Radian));
        Logger.recordOutput("Hood Angle (Deg)", this.goalAngle.in(Degrees));
        Logger.recordOutput("Hood Angle (Rot)", this.goalAngle.in(Rotations));
        Logger.recordOutput("AdjustableHood/ActualAngle (Deg)", inputs.relativeAngle.in(Degrees));
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

    public Command moveWithVoltage(double voltage) {
        return run(() -> io.setAdjustableHoodVoltage(voltage));
    }

    /** Uses the distance and angle tables */
    public Command useAutomaticTable() {
        return runOnce(() -> this.isManualMode = false);
    }
}
