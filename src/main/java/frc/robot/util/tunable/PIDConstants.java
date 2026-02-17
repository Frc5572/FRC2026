package frc.robot.util.tunable;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.util.typestate.InitField;
import frc.robot.util.typestate.OptionalField;
import frc.robot.util.typestate.RequiredField;
import frc.robot.util.typestate.TypeStateBuilder;

public class PIDConstants implements LoggableInputs, Cloneable, Tunable {

    public double kP;
    public double kD;
    public double kI;
    public double kV;
    public double kS;
    public double kG;
    public double kA;
    public GravityTypeValue gravityType;
    private boolean isDirty;
    private String name;

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
        this.gravityType = gravityType;
        this.isDirty = false;

        Tunable.setupTunable("/" + name, this, PIDConstants.class, () -> {
            this.isDirty = true;
        });
    }

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

    public void ifDirty(Runnable runnable) {
        Logger.processInputs(this.name, this);
        if (this.isDirty) {
            runnable.run();
        }
        this.isDirty = false;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("kP", this.kP);
        table.put("kI", this.kI);
        table.put("kD", this.kD);
        table.put("kV", this.kV);
        table.put("kG", this.kG);
        table.put("kS", this.kS);
        table.put("kA", this.kA);
        table.put("isArm", this.gravityType.equals(GravityTypeValue.Arm_Cosine));
    }

    @Override
    public void fromLog(LogTable table) {
        this.kP = table.get("kP", this.kP);
        this.kI = table.get("kI", this.kI);
        this.kD = table.get("kD", this.kD);
        this.kV = table.get("kV", this.kV);
        this.kG = table.get("kG", this.kG);
        this.kS = table.get("kS", this.kS);
        this.kS = table.get("kA", this.kA);
        this.gravityType = table.get("isArm", this.gravityType.equals(GravityTypeValue.Arm_Cosine))
            ? GravityTypeValue.Arm_Cosine
            : GravityTypeValue.Elevator_Static;
    }

    @Override
    public PIDConstants clone() {
        PIDConstants copy = new PIDConstants(name, this.gravityType, kP, kD, kI, kV, kS, kG, kA);
        return copy;
    }

}
