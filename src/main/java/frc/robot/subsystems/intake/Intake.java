package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public IntakeIO io;
    public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }


    public void runIntakeOnly(double speed) {
        io.runIntakeMotor(speed);
    }

    public void runHopperOnly(double setPointDistance) {
        io.runHopperMotor(setPointDistance);
    }

    public Command useIntakeCommand(double intakeSpeed, double hopperDesiredPosition) {
        return Commands
            .run(() -> runHopperOnly(MathUtil.clamp(Constants.IntakeConstants.hopperMinDistance,
                hopperDesiredPosition, Constants.IntakeConstants.hopperMaxDistance)), this)
            .alongWith(runEnd(() -> runIntakeOnly(intakeSpeed), () -> runIntakeOnly(intakeSpeed)));
    }

    public double getHopperPosition() {
        return inputs.hopperPositionMeters;
    }

}
