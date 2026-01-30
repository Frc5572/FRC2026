package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveSim;

/** Simulated state of the robot */
public class SimulatedRobotState {

    /** Swerve state */
    public final SwerveSim swerveDrive;
    // Turret, intake, and hood state would go here too.

    /** Create new robot simulation */
    public SimulatedRobotState(Pose2d initialPose) {
        this.swerveDrive = new SwerveSim(initialPose);
    }

}
