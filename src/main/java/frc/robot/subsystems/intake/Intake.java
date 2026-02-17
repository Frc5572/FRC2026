package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final PIDController leftController = new PIDController(0.1, 0, 0);
    private final PIDController rightController = new PIDController(0.1, 0, 0);

    public Intake(IntakeIO io) {
        this.io = io;
        io.setEncoderPosition(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double p = SmartDashboard.getNumber("Intake/kP", 0.1);
        leftController.setP(p);
        rightController.setP(p);

        Logger.recordOutput("Intake/LeftError", leftController.getError());
        Logger.recordOutput("Intake/RightError", rightController.getError());
        Logger.recordOutput("Intake/TargetRotaitons",
            distanceToRotations(Constants.IntakeConstants.hopperOutDistance.in(Meters)));

        if (inputs.limitSwitch) {
            io.setEncoderPosition(0);
        }
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

        double leftVolts =
            leftController.calculate(inputs.leftHopperPositionRotations, targetRotations);
        double rightVolts =
            rightController.calculate(inputs.rightHopperPositionRotations, targetRotations);

        if (inputs.limitSwitch && targetRotations <= 0.01) {
            leftVolts = 0;
            io.setLeftHopperVoltage(0);
            io.setEncoderPosition(0);
        }
        io.setLeftHopperVoltage(leftVolts);
        io.setRightHopperVoltage(rightVolts);
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
}
