package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeReal implements IntakeIO {
    private TalonFX arm = new TalonFX(0);
    private SparkFlex intakeMotor = new SparkFlex(0, MotorType.kBrushless);
    private TalonFXConfiguration config = new TalonFXConfiguration();

    public IntakeReal() {

    }

    public void configure() {
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
    }

    public @Override public void updateInputs(IntakeIOInputs inputs) {
        inputs.armAngle = arm.getPosition();
    }

    @Override
    public void runIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

}
