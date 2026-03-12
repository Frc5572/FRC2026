package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    /** Extends hopper */
    public Command extendHopper(double intakeSpeed) {
        int[] counts = new int[] {0, 0};
        double[] prev = new double[] {0.0, 0.0};
        return startEnd(() -> {
            counts[0] = 0;
            counts[1] = 0;
            io.setLeftHopperVoltage(3);
            io.setRightHopperVoltage(3);
            runIntakeOnly(intakeSpeed);
            SmartDashboard.putBoolean("Intake/HopperExtended", true);
        }, () -> {
            io.setLeftHopperVoltage(0);
            io.setRightHopperVoltage(0);
            runIntakeOnly(0);
        }).until(() -> {
            double left = inputs.leftHopperPositionRotations;
            double right = inputs.leftHopperPositionRotations;
            boolean leftStopped = left < prev[0] + 0.01;
            boolean rightStopped = right < prev[1] + 0.01;
            prev[0] = left;
            prev[1] = right;
            if (leftStopped) {
                counts[0]++;
            } else {
                counts[0] = 0;
            }
            if (rightStopped) {
                counts[1]++;
            } else {
                counts[1] = 0;
            }
            return counts[0] > 5 && counts[1] > 5;
        });
    }

    /** Retracts hopper */
    public Command retractHopper(double intakeSpeed) {
        int[] counts = new int[] {0, 0};
        double[] prev = new double[] {0.0, 0.0};
        return startEnd(() -> {
            counts[0] = 0;
            counts[1] = 0;
            io.setLeftHopperVoltage(-5);
            io.setRightHopperVoltage(-5);
            runIntakeOnly(intakeSpeed);
            SmartDashboard.putBoolean("Intake/HopperExtended", false);
        }, () -> {
            io.setLeftHopperVoltage(0);
            io.setRightHopperVoltage(0);
            runIntakeOnly(0);
        }).until(() -> {
            double left = inputs.leftHopperPositionRotations;
            double right = inputs.leftHopperPositionRotations;
            boolean leftStopped = left > prev[0] - 0.01;
            boolean rightStopped = right > prev[1] - 0.01;
            prev[0] = left;
            prev[1] = right;
            if (leftStopped) {
                counts[0]++;
            } else {
                counts[0] = 0;
            }
            if (rightStopped) {
                counts[1]++;
            } else {
                counts[1] = 0;
            }
            return counts[0] > 5 && counts[1] > 5;
        });
    }

    /** Run intake wheels */
    public Command intakeBalls(double speed) {
        return runEnd(() -> runIntakeOnly(speed), () -> runIntakeOnly(0));
    }

    public Command intakeBalls() {
        return intakeBalls(1.0);
    }

    public Command jerkIntake() {
        return extendHopper(0).andThen(retractHopper(0)).repeatedly();
    }
}
