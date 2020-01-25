/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Robot sets up the Robot.
 */
public class Robot extends TimedRobot {
    /* The actual robot logic. */
    private RobotContainer m_robotContainer;

    /* A scheduler for the commands issue by the robot. */
    private CommandScheduler m_scheduler;

    /* The currently selected autonomous command. */
    private Command m_autonomousCommand;

    /* The currently selected teleop command. */
    private Command m_teleopCommand;

    /**
     * Initializes the robot.
     */
    @Override
    public void robotInit() {
        // Initialize a container for the robot logic
        m_robotContainer = new RobotContainer();

        // Load up a command scheduler that we can use for scheduling commands, duh.
        this.m_scheduler = CommandScheduler.getInstance();
    }

    /**
     * Polls the robot, allocating resources to the event loop as necessary.
     */
    @Override
    public void robotPeriodic() {
        // Poll the scheduler. This must be done every frame, since it's really just an
        // event loop with a fancy name.
        this.m_scheduler.run();
    }

    @Override
    public void autonomousInit() {
        // If the teleop command is already running, stop it.
        if (this.m_teleopCommand != null) {
            this.m_teleopCommand.cancel();
        }

        // Set up the autonomous command for the robot
        this.m_autonomousCommand = this.m_robotContainer.getAutonomousCommand();

        // Start the autonomous command
        this.m_scheduler.schedule(this.m_autonomousCommand);
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // Set up the teleop command for the robot
        this.m_teleopCommand = this.m_robotContainer.getTeleopCommand();

        // Start the teleop command
        this.m_scheduler.schedule(this.m_teleopCommand);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }
}