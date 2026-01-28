package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double angleMotorPosition;
        public double rightMotorPosition;
        public double leftMotorPosition;
    }

    public void updateInputs(ClimberIOInputs inputs);

    public void runAngleMotor(double setPoint);

    public void runLeftMotor(double setPoint);

    public void runRightMotor(double setPoint);

    public void setAngleMotorEncoderPosition(double position);

    public void setLeftMotorEncoderPosition(double position);

    public void setRightMotorEncoderPosition(double position);


}
