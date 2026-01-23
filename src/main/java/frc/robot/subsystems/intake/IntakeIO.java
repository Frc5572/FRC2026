package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        StatusSignal<Angle> armAngle;
        double intakeVelocity;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void runIntakeMotor(double speed);

}
