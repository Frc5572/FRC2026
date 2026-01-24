package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Angle;

public class IntakeReal implements IntakeIO {
    private TalonFX arm = new TalonFX(0);
    private SparkFlex intakeMotor = new SparkFlex(0, MotorType.kBrushless);
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private StatusSignal<Angle> armPosition = arm.getPosition();

    public IntakeReal() {

    }

    public void configure() {
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        arm.getConfigurator().apply(config);

    }


    @Override
    public void runIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.armAngle = armPosition.getValue().times(360);
    }

    @Override
    public void setEncoderPosition(double position) {
        arm.getConfigurator().setPosition(position);
    }

    @Override
    public void runArmMotor(double speed) {
        arm.set(speed);
    }


}
