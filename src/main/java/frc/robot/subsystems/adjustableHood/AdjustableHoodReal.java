package frc.robot.subsystems.adjustableHood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class AdjustableHoodReal implements AdjustableHoodIO {

    private final TalonFX hoodMotor = new TalonFX(Constants.AdjustableHood.HoodMotorID);
    private TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    private CANcoder hoodCANcoder = new CANcoder(Constants.AdjustableHood.HoodCANCoderID);
    private CANcoderConfiguration hoodCANcoderConfig = new CANcoderConfiguration();

    private StatusSignal<Angle> hoodAngle = hoodMotor.getPosition();
    private StatusSignal<Voltage> hoodVoltage = hoodMotor.getMotorVoltage();
    private StatusSignal<Current> hoodCurrent = hoodMotor.getStatorCurrent();
    private StatusSignal<Angle> hoodCANcoderAngle = hoodCANcoder.getPosition();

    public AdjustableHoodReal() {
        configAdjustableHood();

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        BaseStatusSignal.setUpdateFrequencyForAll(50, hoodAngle, hoodVoltage, hoodCurrent,
            hoodCANcoderAngle);
    }

    private void configAdjustableHood() {

        // PID and feedforward

        hoodConfig.Slot0.kP = Constants.AdjustableHood.KP;
        hoodConfig.Slot0.kI = Constants.AdjustableHood.KI;
        hoodConfig.Slot0.kD = Constants.AdjustableHood.KD;
        hoodConfig.Slot0.kS = Constants.AdjustableHood.KS;
        hoodConfig.Slot0.kV = Constants.AdjustableHood.KV;
        hoodConfig.Slot0.kA = Constants.AdjustableHood.KA;
        hoodConfig.Slot0.kG = Constants.AdjustableHood.KG;


    }

    public void setAdjustableHoodVoltage(Voltage volts) {

    }

    public void updateInputs(AdjustableHoodInputs inputs) {

    }

    public void setTargetAngle(Angle angle) {}

}
