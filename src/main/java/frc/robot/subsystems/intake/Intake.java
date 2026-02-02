package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
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
    }


    public void runIntakeOnly(double speed) {
        io.runIntakeMotor(speed);
    }

    public void runHopperOnly(double setPointDistanceInMeters) {
        io.runHopperMotor(setPointDistanceInMeters);
    }

    /**
     * run the hopper
     *
     * @param distance where the hopper will go
     * @param intakeSpeed how fast the intake will run
     * @return returns command
     */
    public Command intake(Distance distance, double intakeSpeed) {
        return Commands.run(() -> {
            runHopperOnly(MathUtil.clamp(Constants.IntakeConstants.hopperMinDistance,
                distance.in(Meters), Constants.IntakeConstants.hopperMaxDistance));
            runIntakeOnly(intakeSpeed);
        }, this).until(limitSwitchTouched);
    }

}
