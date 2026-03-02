package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * Intake subsystem
 */
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    public final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    private final Trigger limitSwitchTouched = new Trigger(() -> inputs.limitSwitch).debounce(0.25);

    public Intake(IntakeIO io) {
        this.io = io;
        io.setEncoderPosition(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double targetRotations =
            distanceToRotations(Constants.IntakeConstants.hopperOutDistance.in(Meters));
        Logger.recordOutput("Intake/LeftError",
            targetRotations - inputs.leftHopperPositionRotations);
        Logger.recordOutput("Intake/RightError",
            targetRotations - inputs.rightHopperPositionRotations);
        Logger.recordOutput("Intake/TargetRotaitons",
            distanceToRotations(Constants.IntakeConstants.hopperOutDistance.in(Meters)));

        // if (inputs.limitSwitch) {
        // io.setLeftHopperVoltage(0);
        // io.setEncoderPosition(0);
        // }
    }


    public void runIntakeOnly(double speed) {
        io.runIntakeMotor(speed);
    }

    private double distanceToRotations(double meters) {
        return meters / (Constants.IntakeConstants.pinionDiameter * Math.PI)
            * Constants.IntakeConstants.gearRatio;
    }

    /** runs hopper */
    public void runHopper(double targetMeters) {
        double targetRotations = distanceToRotations(targetMeters);

        Logger.recordOutput("Intake/TargetMeters", targetMeters);
        Logger.recordOutput("Intake/TargetRotations", targetRotations);

        // if (inputs.limitSwitch && targetRotations <= 0.01) {
        // io.setLeftHopperVoltage(0);
        // io.setEncoderPosition(0);
        // } else
        {
            io.setLeftHopperPosition(targetRotations);
        }
        io.setRightHopperPosition(targetRotations);
    }

    /** Stops the hopper from expanding */
    public Command stop() {
        return this.runOnce(() -> {
            this.io.setRightHopperVoltage(0);
            this.io.setLeftHopperVoltage(0);
        });
    }

    public Command extendHopper() {
        return run(() -> runHopper(Constants.IntakeConstants.hopperOutDistance.in(Meters)));
    }

    public Command retractHopper() {
        return run(() -> runHopper(0.0));
    }

    public Command intakeBalls(double speed) {
        return runEnd(() -> runIntakeOnly(speed), () -> runIntakeOnly(0));
    }

    /**
     * Returns true if the hopper is close to the target extension. Adjust the 0.05 tolerance based
     * on your mechanism's precision.
     */
    public boolean isHopperAtTarget(double targetMeters) {
        double targetRotations = distanceToRotations(targetMeters);
        return Math.abs(inputs.leftHopperPositionRotations - targetRotations) < 0.05;
    }

    /**
     * A command that finishes only when the hopper is extended. We add a 1.5s failsafe timeout in
     * case it gets stuck.
     */
    public Command waitUntilExtended() {
        double target = Constants.IntakeConstants.hopperOutDistance.in(Meters);
        return run(() -> runHopper(target)).until(() -> isHopperAtTarget(target)).withTimeout(1.5);
    }
}
