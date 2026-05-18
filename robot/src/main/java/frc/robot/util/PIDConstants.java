package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.util.typestate.InitField;
import frc.robot.util.typestate.OptionalField;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

/** Tunable constants for PID controllers */
public class PIDConstants {

    public double kP;
    public double kD;
    public double kI;
    public double kV;
    public double kS;
    public double kG;
    public double kA;
    public boolean isArm;
    private String name;

    /** Create new PID Constants */
    @TypeStateBuilder("PIDConstantsBuilder")
    public PIDConstants(@InitField String name, @InitField GravityTypeValue gravityType,
        @RequiredField double kP, @OptionalField("0.0") double kI, @OptionalField("0.0") double kD,
        @OptionalField("0.0") double kV, @OptionalField("0.0") double kS,
        @OptionalField("0.0") double kG, @OptionalField("0.0") double kA) {
        this.name = name;
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kV = kV;
        this.kS = kS;
        this.kG = kG;
        this.kA = kA;
        this.isArm = gravityType.equals(GravityTypeValue.Arm_Cosine);
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
        config.GravityType = isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
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
        config.GravityType = isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
    }

}
