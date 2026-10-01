package frc.robot.subsystems.intake;

import java.util.Set;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem
 */
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    public final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

    }


    public void runIntakeOnly(double speed) {
        io.runIntakeMotor(speed);
    }

    /** Stops the hopper from expanding */
    public Command stop() {
        return this.runOnce(() -> {
            this.io.setRightHopperVoltage(0);
            this.io.setLeftHopperVoltage(0);
        });
    }

    private Command runHopperToStop(double voltage, double intakeSpeed, boolean isExtended,
        double timeoutSeconds) {
        return Commands.defer(() -> {
            final double[] prev = new double[] {0.0, 0.0};
            final int[] stallCounts = new int[] {0, 0};
            final boolean[] sideStopped = new boolean[] {false, false};
            final int[] loopCounter = new int[] {0};
            final int minRunTicks = 5;
            final int stallThresholdTicks = 5;
            final double minDeltaRotations = 0.01;
            return Commands.runEnd(() -> {
                loopCounter[0]++;
                double left = inputs.leftHopperPositionRotations;
                double right = inputs.rightHopperPositionRotations;
                if (loopCounter[0] > minRunTicks) {

                    double leftDelta = (voltage > 0) ? (left - prev[0]) : (prev[0] - left);
                    double rightDelta = (voltage > 0) ? (right - prev[1]) : (prev[1] - right);
                    if (!sideStopped[0]) {
                        if (leftDelta < minDeltaRotations) {
                            stallCounts[0]++;
                            if (stallCounts[0] >= stallThresholdTicks) {
                                sideStopped[0] = true;
                                io.setLeftHopperVoltage(0);
                            }
                        } else {
                            stallCounts[0] = 0;
                        }
                    }
                    if (!sideStopped[1]) {
                        if (rightDelta < minDeltaRotations) {
                            stallCounts[1]++;
                            if (stallCounts[1] >= stallThresholdTicks) {
                                sideStopped[1] = true;
                                io.setRightHopperVoltage(0);
                            }
                        } else {
                            stallCounts[1] = 0;
                        }
                    }
                }
                prev[0] = left;
                prev[1] = right;
            }, () -> {
                io.setLeftHopperVoltage(0);
                io.setRightHopperVoltage(0);
                runIntakeOnly(0);
            }, this).beforeStarting(() -> {
                prev[0] = inputs.leftHopperPositionRotations;
                prev[1] = inputs.rightHopperPositionRotations;
                stallCounts[0] = 0;
                stallCounts[1] = 0;
                sideStopped[0] = false;
                sideStopped[1] = false;
                loopCounter[0] = 0;
                io.setLeftHopperVoltage(voltage);
                io.setRightHopperVoltage(voltage);
                runIntakeOnly(intakeSpeed);
                SmartDashboard.putBoolean("Intake/HopperExtended", isExtended);
            }).until(() -> sideStopped[0] && sideStopped[1]).withTimeout(timeoutSeconds);
        }, Set.of(this));
    }

    /** Extends hopper */
    public Command extendHopper(double intakeSpeed) {
        return runHopperToStop(5.0, intakeSpeed, true, 1.0);
    }

    /** Retracts hopper */
    public Command retractHopper(double intakeSpeed) {
        return runHopperToStop(-3.0, intakeSpeed, false, 1.0);
    }


    /** Run intake wheels */
    public Command intakeBalls(double speed) {
        return runEnd(() -> runIntakeOnly(speed), () -> runIntakeOnly(0));
    }

    public Command intakeBalls() {
        return intakeBalls(1.0);
    }

    public Command jerkIntake() {
        return extendHopper(1).andThen(Commands.waitSeconds(0.5), retractHopper(1)).repeatedly();
    }
}
