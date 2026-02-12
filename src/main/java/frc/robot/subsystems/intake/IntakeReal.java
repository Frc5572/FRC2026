package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * intake real implementation
 */
public class IntakeReal implements IntakeIO {
    private TalonFX hopperRightMotor = new TalonFX(Constants.IntakeConstants.hopperRightID);
    private TalonFX hopperLeftMotor = new TalonFX(Constants.IntakeConstants.hopperLeftID);
    private SparkFlex intakeMotor;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private DigitalInput limitSwitchMin = new DigitalInput(Constants.IntakeConstants.limitSwitchID);
    private final StatusSignal<Angle> rightMotorPosition = hopperRightMotor.getPosition();

    private boolean intakeConnected = false;

    /** Real Intake Implementation */
    public IntakeReal() {
        try {
            intakeMotor = new SparkFlex(Constants.IntakeConstants.intakeID, MotorType.kBrushless);
            if (intakeMotor.getFirmwareVersion() == 0) {
                throw new Exception("Motor not found");
            }
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
        hopperRightMotor.getConfigurator().apply(config);
        hopperLeftMotor
            .setControl(new Follower(hopperRightMotor.getDeviceID(), MotorAlignmentValue.Opposed)); // check
    }


    @Override
    public void runIntakeMotor(double speed) {
        if (intakeConnected && intakeMotor != null) {
            intakeMotor.set(speed);
        }
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        BaseStatusSignal.refreshAll(rightMotorPosition);
        inputs.hopperPosition = Meters.of(rightMotorPosition.getValue().in(Rotations));
        inputs.limitSwitch = limitSwitchMin.get();

        inputs.intakeMotorConnected = intakeConnected;
        if (intakeConnected && intakeMotor != null) {
            inputs.intakeDutyCycle = intakeMotor.get();
        } else {
            inputs.intakeDutyCycle = 0.0;
        }

    }

    @Override
    public void setEncoderPosition(double position) {
        hopperRightMotor.getConfigurator().setPosition(position);
        hopperLeftMotor.getConfigurator().setPosition(position);
    }

    @Override
    public void runHopperMotor(double setPoint) {
        hopperRightMotor.setControl(positionVoltage.withPosition(setPoint));
    }


}
