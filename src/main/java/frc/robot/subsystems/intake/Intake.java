package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
        Command current = getCurrentCommand();
        Logger.recordOutput("Intake/CurrentCommand", current == null ? "none" : current.getName());
    }


    public void runIntakeOnly(double speed) {
        io.runIntakeMotor(speed);
    }

    /** Stops the hopper from expanding */
    public Command stop() {
        return this.runOnce(() -> {
            setHopperVoltage(0, 0);
        });
    }

    /**
     * Consecutive ticks a side must be stalled before it counts as stopped. Kept short because the
     * hopper motors heat up quickly under stall.
     */
    private static final int STALL_TICKS = 6;
    /** Minimum movement per tick, in rotations, for a side to count as moving. */
    private static final double STALL_THRESHOLD = 0.01;

    /**
     * Detects when each hopper side has stopped moving in the commanded direction. Must be
     * {@link #reset() reset} whenever a move starts.
     */
    private class HopperStallDetector {
        private final double direction;
        private final double[] prev = new double[2];
        private final int[] counts = new int[2];

        HopperStallDetector(double direction) {
            this.direction = Math.signum(direction);
        }

        void reset() {
            prev[0] = inputs.leftHopperPositionRotations;
            prev[1] = inputs.rightHopperPositionRotations;
            counts[0] = 0;
            counts[1] = 0;
        }

        /** Call once per tick while moving. */
        void update() {
            double[] now =
                {inputs.leftHopperPositionRotations, inputs.rightHopperPositionRotations};
            for (int i = 0; i < 2; i++) {
                // Once stalled, a side stays stalled for the rest of the move.
                boolean moving = direction * (now[i] - prev[i]) >= STALL_THRESHOLD;
                counts[i] = moving && !stalled(i) ? 0 : counts[i] + 1;
                prev[i] = now[i];
            }
            Logger.recordOutput("Intake/StallCounts", counts.clone());
        }

        /** Whether side {@code i} (0 = left, 1 = right) has stalled. */
        boolean stalled(int i) {
            return counts[i] >= STALL_TICKS;
        }
    }

    /**
     * Drives both hopper sides at {@code volts} until both have stopped moving. Each side is cut
     * off as soon as it stalls, so it does not sit hot at its stop while the other side finishes.
     *
     * @param volts hopper voltage; positive extends
     * @param intakeSpeed intake roller duty cycle while moving
     * @param extended whether this move extends the hopper, for the dashboard
     */
    private Command moveHopper(double volts, double intakeSpeed, boolean extended) {
        HopperStallDetector stall = new HopperStallDetector(volts);
        return new FunctionalCommand(() -> {
            stall.reset();
            setHopperVoltage(volts, volts);
            runIntakeOnly(intakeSpeed);
            SmartDashboard.putBoolean("Intake/HopperExtended", extended);
        }, () -> {
            stall.update();
            setHopperVoltage(stall.stalled(0) ? 0 : volts, stall.stalled(1) ? 0 : volts);
        }, interrupted -> {
            setHopperVoltage(0, 0);
            runIntakeOnly(0);
        }, () -> stall.stalled(0) && stall.stalled(1), this).withName("moveHopper(" + volts + ")");
    }

    private void setHopperVoltage(double left, double right) {
        io.setLeftHopperVoltage(left);
        io.setRightHopperVoltage(right);
        Logger.recordOutput("Intake/HopperCommandedVolts", new double[] {left, right});
    }

    /** Extends hopper */
    public Command extendHopper(double intakeSpeed) {
        return moveHopper(5.0, intakeSpeed, true);
    }

    /** Retracts hopper */
    public Command retractHopper(double intakeSpeed) {
        return moveHopper(-3.0, intakeSpeed, false).withTimeout(1.0);
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
