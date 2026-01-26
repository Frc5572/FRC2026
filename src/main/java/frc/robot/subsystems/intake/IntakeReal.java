package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeReal implements IntakeIO {
    private SparkFlex arm = new SparkFlex(0, null);
    private SparkFlex intakeMotor = new SparkFlex(0, MotorType.kBrushless);
    private RelativeEncoder armEncoder = arm.getEncoder();
    private SparkBaseConfig armConfig;

    public IntakeReal() {

    }

    public void configure() {
        armConfig.idleMode(IdleMode.kBrake);
        arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    @Override
    public void runIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.armAngle = armEncoder.getPosition() * 360;
    }

    @Override
    public void setEncoderPosition(double position) {
        arm.getEncoder().setPosition(position);
    }

    @Override
    public void runArmMotor(double speed) {
        arm.set(speed);
    }


}
