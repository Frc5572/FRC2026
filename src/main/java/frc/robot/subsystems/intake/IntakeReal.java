package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * intake real implementation
 */
public class IntakeReal implements IntakeIO {
    private TalonFX hopperRightMotor = new TalonFX(Constants.IntakeConstants.hopperRightID);
    private TalonFX hopperLeftMotor = new TalonFX(Constants.IntakeConstants.hopperLeftID);
    private TalonFX intakeMotor;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private DigitalInput limitSwitchMin = new DigitalInput(Constants.IntakeConstants.limitSwitchID);
    private final StatusSignal<Angle> rightMotorPosition = hopperRightMotor.getPosition();
    private final StatusSignal<Angle> leftMotorPosition = hopperLeftMotor.getPosition();

    private boolean intakeConnected = false;

    /** Real Intake Implementation */
    public IntakeReal() {
        try {
            intakeMotor = new TalonFX(Constants.IntakeConstants.intakeID);
            intakeConnected = true;
        } catch (Exception e) {
            System.out.println("Intake initialization failed: " + e.getMessage());
            intakeConnected = false;
        }

        config.Feedback.SensorToMechanismRatio = 1; // change for testing
        config.Slot0.kP = Constants.IntakeConstants.KP; // change for testing
        config.Slot0.kI = Constants.IntakeConstants.KI; // change for testing
        config.Slot0.kD = Constants.IntakeConstants.KD; // change for testing
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hopperLeftMotor.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperRightMotor.getConfigurator().apply(config);

        leftMotorPosition.setUpdateFrequency(50);
        rightMotorPosition.setUpdateFrequency(50);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotor.getConfigurator().apply(config);
    }


    @Override
    public void runIntakeMotor(double speed) {
        if (intakeConnected && intakeMotor != null) {
            intakeMotor.setControl(new DutyCycleOut(speed));
        }
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        BaseStatusSignal.refreshAll(rightMotorPosition, leftMotorPosition);
        inputs.rightHopperPositionRotations = rightMotorPosition.getValue().in(Rotations);
        inputs.leftHopperPositionRotations = leftMotorPosition.getValue().in(Rotations);
        inputs.limitSwitch = limitSwitchMin.get();

        inputs.intakeMotorConnected = intakeConnected;
        if (intakeConnected && intakeMotor != null) {
            inputs.intakeDutyCycle = intakeMotor.get();
        } else {
            inputs.intakeDutyCycle = 0.0;
        }
    }

    @Override
    public void setLeftHopperPosition(double rotations) {
        hopperLeftMotor.setControl(positionVoltage.withPosition(rotations));
    }

    @Override
    public void setRightHopperPosition(double rotations) {
        hopperRightMotor.setControl(positionVoltage.withPosition(rotations));
    }

    @Override
    public void setEncoderPosition(double position) {
        hopperRightMotor.getConfigurator().setPosition(position);
        hopperLeftMotor.getConfigurator().setPosition(position);
    }

    @Override
    public void setLeftHopperVoltage(double volts) {
        hopperLeftMotor.setVoltage(volts);
    }

    @Override
    public void setRightHopperVoltage(double volts) {
        hopperRightMotor.setVoltage(volts);
    }
}
