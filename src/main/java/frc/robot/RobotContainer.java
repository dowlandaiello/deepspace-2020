/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.RhinoDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.MotorControllerConfiguration;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * RobotContainer holds all of the robot's subsystems.
 * 
 * @author Dowland Aiello
 */
public class RobotContainer {
    /* The robot's drivetrain. */
    private final DriveSubsystem m_drivetrain;

    /* The robot's settings. */
    private final Preferences m_preferences;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Make a motor controller config for the drivetraini
        MotorControllerConfiguration motorCfg = new MotorControllerConfiguration(Constants.FRONTLEFTMOTOR,
                Constants.FRONTRIGHTMOTOR, Constants.BACKLEFTMOTOR, Constants.BACKRIGHTMOTOR);

        // Initialize each of the subsystems
        this.m_preferences = Preferences.getInstance();
        this.m_drivetrain = new DriveSubsystem(motorCfg);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }
}
