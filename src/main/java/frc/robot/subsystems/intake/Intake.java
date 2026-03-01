package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        // {
        io.setLeftHopperPosition(targetRotations);
        // }
        io.setRightHopperPosition(targetRotations);
    }

    /** Stops the hopper from expanding */
    public Command stop() {
        return this.runOnce(() -> {
            this.io.setRightHopperVoltage(0);
            this.io.setLeftHopperVoltage(0);
        });
    }

    /** Extends hopper */
    public Command extendHopper() {
        int[] counts = new int[] {0, 0};
        double[] prev = new double[] {0.0, 0.0};
        return startEnd(() -> {
            counts[0] = 0;
            counts[1] = 0;
            io.setLeftHopperVoltage(3);
            io.setRightHopperVoltage(3);
            SmartDashboard.putBoolean("Intake/HopperExtended", true);
        }, () -> {
            io.setLeftHopperVoltage(0);
            io.setRightHopperVoltage(0);
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
    public Command retractHopper() {
        int[] counts = new int[] {0, 0};
        double[] prev = new double[] {0.0, 0.0};
        return startEnd(() -> {
            counts[0] = 0;
            counts[1] = 0;
            io.setLeftHopperVoltage(-5);
            io.setRightHopperVoltage(-5);
            SmartDashboard.putBoolean("Intake/HopperExtended", false);
        }, () -> {
            io.setLeftHopperVoltage(0);
            io.setRightHopperVoltage(0);
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
        return runEnd(() -> runIntakeOnly(0.7), () -> runIntakeOnly(0));
    }

    public Command jerkIntake() {
        return Commands.none();
    }
}
