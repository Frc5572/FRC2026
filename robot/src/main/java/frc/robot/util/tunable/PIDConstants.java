package frc.robot.util.tunable;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.util.typestate.InitField;
import frc.robot.util.typestate.OptionalField;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Tunable constants for PID controllers */
@frc.robot.util.Tunable
public class PIDConstants {

    public double kP;
    public double kD;
    public double kI;
    public double kV;
    public double kS;
    public double kG;
    public double kA;
    public GravityTypeValue gravityType;

    /** Create new PID Constants */
    @TypeStateBuilder("PIDConstantsBuilder")
    public PIDConstants(@InitField GravityTypeValue gravityType, @RequiredField double kP,
        @OptionalField("0.0") double kI, @OptionalField("0.0") double kD,
        @OptionalField("0.0") double kV, @OptionalField("0.0") double kS,
        @OptionalField("0.0") double kG, @OptionalField("0.0") double kA) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kV = kV;
        this.kS = kS;
        this.kG = kG;
        this.kA = kA;
        this.gravityType = gravityType;
    }

    /** Write PID constants to TalonFX */
    public void apply(Slot0Configs config) {
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kV;
        config.kG = kG;
        config.kS = kS;
        config.kA = kA;
        config.GravityType = gravityType;
    }

    /** Write PID constants to TalonFX */
    public void apply(Slot1Configs config) {
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kV;
        config.kG = kG;
        config.kS = kS;
        config.kA = kA;
        config.GravityType = gravityType;
    }

}
