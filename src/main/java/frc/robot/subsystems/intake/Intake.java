package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public IntakeIO io;
    public IntakeIOInputsAutoLogged log = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(null);
    }

    public void runIntake(double intakeSpeed, double armSpeed) {
        io.runArmMotor(armSpeed);
        io.runIntakeMotor(intakeSpeed);
    }

    public void runIntakeOnly(double speed) {
        io.runIntakeMotor(speed);
    }

    public void runArmOnly(double speed) {
        io.runArmMotor(speed);
    }

    public Command useIntakeCommand(double intakeSpeed, double armSpeed, BooleanSupplier angle) {
        return Commands.runEnd(() -> runArmOnly(armSpeed), () -> runArmOnly(armSpeed), this)
            .until(angle).unless(angle)
            .alongWith(runEnd(() -> runIntakeOnly(intakeSpeed), () -> runIntakeOnly(intakeSpeed)))
            .andThen(runEnd(() -> runIntakeOnly(intakeSpeed), () -> runIntakeOnly(intakeSpeed)));
    }

    public double getArmAngle() {
        return log.armAngle;
    }

}
