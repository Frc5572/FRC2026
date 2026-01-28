package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        double hopperPosition;
        double intakeVelocity;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void runIntakeMotor(double speed);

    public void setEncoderPosition(double position);

    public void runHopperMotor(double setPoint);

    public static class Empty implements IntakeIO {

        @Override
        public void updateInputs(IntakeIOInputs inputs) {}

        @Override
        public void runIntakeMotor(double speed) {}

        @Override
        public void setEncoderPosition(double position) {}

        @Override
        public void runHopperMotor(double speed) {}

    }

}
