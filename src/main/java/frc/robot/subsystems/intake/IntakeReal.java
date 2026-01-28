package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

public class IntakeReal implements IntakeIO {
    private TalonFX hopper = new TalonFX(Integer.MIN_VALUE);
    private SparkFlex intakeMotor = new SparkFlex(0, MotorType.kBrushless);
    private Slot0Configs configs = new Slot0Configs();
    private TalonFXConfiguration configuration = new TalonFXConfiguration();
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    public IntakeReal() {
        configure();
    }

    public void configure() {
        configuration.Slot0.kP = Constants.IntakeConstants.KP;
        configuration.Slot0.kI = Constants.IntakeConstants.KI;
        configuration.Slot0.kD = Constants.IntakeConstants.KD;
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hopper.getConfigurator().apply(configs);

    }


    @Override
    public void runIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.hopperPositionMeters =
            hopper.getPosition().getValueAsDouble() / Constants.IntakeConstants.distanceToRotations;
    }

    @Override
    public void setEncoderPosition(double position) {
        hopper.getConfigurator().setPosition(position);
    }

    @Override
    public void runHopperMotor(double setPoint) {
        hopper.setControl(
            positionVoltage.withPosition(setPoint * Constants.IntakeConstants.distanceToRotations));
    }


}
