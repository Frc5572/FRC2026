package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class IntakeReal implements IntakeIO {
    private TalonFX hopperRightMotor = new TalonFX(Constants.IntakeConstants.hopperRightID);
    private TalonFX hopperLeftMotor = new TalonFX(Constants.IntakeConstants.hopperLeftID);
    private SparkFlex intakeMotor = new SparkFlex(0, MotorType.kBrushless);
    private TalonFXConfiguration rightConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration leftConfiguration = new TalonFXConfiguration();
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    public IntakeReal() {
        configure();
    }

    public void configure() {
        rightConfiguration.Slot0.kP = Constants.IntakeConstants.KP;
        rightConfiguration.Slot0.kI = Constants.IntakeConstants.KI;
        rightConfiguration.Slot0.kD = Constants.IntakeConstants.KD;
        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfiguration.Slot0.kP = Constants.IntakeConstants.KP;
        leftConfiguration.Slot0.kI = Constants.IntakeConstants.KI;
        leftConfiguration.Slot0.kD = Constants.IntakeConstants.KD;
        leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperLeftMotor.getConfigurator().apply(leftConfiguration);
        hopperRightMotor.getConfigurator().apply(rightConfiguration);

    }


    @Override
    public void runIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.hopperPositionMeters = hopperRightMotor.getPosition().getValueAsDouble()
            / Constants.IntakeConstants.distanceToRotations;

    }

    @Override
    public void setEncoderPosition(double position) {
        hopperRightMotor.getConfigurator().setPosition(position);
        hopperLeftMotor.getConfigurator().setPosition(position);
    }

    @Override
    public void runHopperMotor(double setPoint) {
        hopperRightMotor.setControl(
            positionVoltage.withPosition(setPoint * Constants.IntakeConstants.distanceToRotations));
        hopperLeftMotor.setControl(
            positionVoltage.withPosition(setPoint * Constants.IntakeConstants.distanceToRotations));
    }


}
