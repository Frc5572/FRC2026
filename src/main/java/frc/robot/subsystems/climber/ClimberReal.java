package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberReal implements ClimberIO {
    private TalonFX angleMotor = new TalonFX(0);
    private TalonFX rightMotor = new TalonFX(0);
    private TalonFX leftMotor = new TalonFX(0);
    private TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.angleMotorPosition = angleMotor.getPosition().getValueAsDouble();
        inputs.leftMotorPosition = leftMotor.getPosition().getValueAsDouble();
        inputs.rightMotorPosition = rightMotor.getPosition().getValueAsDouble();
    }

    public void configure() {
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        angleConfig.Slot0.kP = 0;

    }

    @Override
    public void runAngleMotor(double setPoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runAngleMotor'");
    }

    @Override
    public void runLeftMotor(double setPoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runLeftMotor'");
    }

    @Override
    public void runRightMotor(double setPoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runRightMotor'");
    }

    @Override
    public void setAngleMotorEncoderPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException(
            "Unimplemented method 'setAngleMotorEncoderPosition'");
    }

    @Override
    public void setLeftMotorEncoderPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException(
            "Unimplemented method 'setLeftMotorEncoderPosition'");
    }

    @Override
    public void setRightMotorEncoderPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException(
            "Unimplemented method 'setRightMotorEncoderPosition'");
    }



}
