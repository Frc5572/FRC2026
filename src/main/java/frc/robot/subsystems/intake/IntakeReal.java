package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeReal implements IntakeIO {
    private SparkFlex hopper = new SparkFlex(0, null);
    private SparkFlex intakeMotor = new SparkFlex(0, MotorType.kBrushless);
    private RelativeEncoder hopperEncoder = hopper.getEncoder();
    private SparkBaseConfig hopperConfig;
    private SparkClosedLoopController pid = hopper.getClosedLoopController();

    public IntakeReal() {

    }

    public void configure() {
        hopperConfig.idleMode(IdleMode.kBrake);
        hopperConfig.closedLoop.p(0).i(0).d(0);
        hopper.configure(hopperConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

    }


    @Override
    public void runIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.hopperPosition = hopperEncoder.getPosition();
    }

    @Override
    public void setEncoderPosition(double position) {
        hopper.getEncoder().setPosition(position);
    }

    @Override
    public void runHopperMotor(double setPoint) {
        pid.setSetpoint(setPoint, ControlType.kPosition);
    }


}
