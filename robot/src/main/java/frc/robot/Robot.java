// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Runs tasks on Roborio in this file.
 */
public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;

    /**
     * Robnot Run type
     */
    public static enum RobotRunType {
        /** Real Robot. */
        kReal,
        /** Simulation runtime. */
        kSimulation,
        /** Replay runtime. */
        kReplay;
    }

    public RobotRunType robotRunType = RobotRunType.kReal;
    private Timer gcTimer = new Timer();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot(boolean isReplay) {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
            Logger.addDataReceiver(new NT4Publisher());
            setUseTiming(true);
            robotRunType = RobotRunType.kReal;
        } else {
            Logger.addDataReceiver(new NT4Publisher());
            setUseTiming(true);
            robotRunType = RobotRunType.kSimulation;
        }
        Logger.start();

        robotContainer = new RobotContainer(robotRunType);

        gcTimer.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (gcTimer.advanceIfElapsed(5)) {
            System.gc();
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
