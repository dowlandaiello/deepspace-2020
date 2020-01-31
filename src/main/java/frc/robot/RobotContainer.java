/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.commands.DifferentialDriveCommand;
import frc.robot.commands.MoveToReflectiveTargetCommand;
import frc.robot.commands.RhinoDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem.MotorControllerConfiguration;
import frc.robot.subsystems.VisionSubsystem.LimelightConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * RobotContainer holds all of the robot's subsystems.
 * 
 * @author Dowland Aiello
 */
public class RobotContainer {
        /* BEGIN SUBSYSTEMS */

        /* The robot's drivetrain. */
        private final DriveSubsystem m_drivetrain;

        /* The robot's vision subsystem. */
        private final VisionSubsystem m_vision;

        /* END SUBSYSTEMS */

        /* The robot's settings. */
        private final Preferences m_preferences;

        /* The driver's joystick. */
        private final Joystick m_leftDriverJoystick, m_rightDriverJoystick;

        /* The button box. */
        private final Joystick m_buttonbox;

        /* The button on the button box used to activate the autonomous command. */
        private final JoystickButton m_activateAutonomousButton;

        /* BEGIN COMMANDS */

        /* The current teleOp command for the robot. */
        private final RhinoDriveCommand teleopCommand;

        /* A fallback teleOp command for the robot (arcade drive). */
        private final DifferentialDriveCommand fallbackTeleopCommand;

        /* The current autonomous command for the robot. */
        private final MoveToReflectiveTargetCommand autonomousCommand;

        /* END COMMANDS */

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Make a motor controller config for the drivetraini
                MotorControllerConfiguration motorCfg = new MotorControllerConfiguration(Constants.FRONTLEFTMOTOR,
                                Constants.FRONTRIGHTMOTOR, Constants.BACKLEFTMOTOR, Constants.BACKRIGHTMOTOR);

                // Keep the light on the limelight on at all times
                LimelightConfiguration visionCfg = new LimelightConfiguration(VisionSubsystem.LEDMode.ON);

                // Get a reference to the limelight network tables connector
                NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

                // Initialize each of the subsystems
                this.m_preferences = Preferences.getInstance();
                this.m_drivetrain = new DriveSubsystem(motorCfg);
                this.m_vision = new VisionSubsystem(limelightTable, visionCfg);

                // Set up the controllers for the teleop command
                this.m_leftDriverJoystick = new Joystick(0);
                this.m_rightDriverJoystick = new Joystick(1);

                // Set up the buttons for the autonomous command
                this.m_buttonbox = new Joystick(2);
                this.m_activateAutonomousButton = new JoystickButton(this.m_buttonbox, 1);

                // Set up the actual teleop command
                this.teleopCommand = new RhinoDriveCommand(this.m_drivetrain,
                                () -> this.m_leftDriverJoystick.getRawAxis(1),
                                () -> this.m_rightDriverJoystick.getRawAxis(1));

                // Set up an alternative teleop command that uses arcade drive; use just one
                // joystick
                this.fallbackTeleopCommand = new DifferentialDriveCommand(this.m_drivetrain,
                                () -> this.m_leftDriverJoystick.getRawAxis(1),
                                () -> this.m_leftDriverJoystick.getRawAxis(2));

                // Set up the autonomous command
                this.autonomousCommand = new MoveToReflectiveTargetCommand(this.m_drivetrain, this.m_vision,
                                new MoveToReflectiveTargetCommand.Configuration(
                                                () -> this.m_preferences.getDouble("vision::kP",
                                                                Constants.DEFAULT_VISION_KP),
                                                () -> this.m_preferences.getDouble("vision::kChange",
                                                                Constants.DEFAULT_VISION_KCHANGE),
                                                () -> this.m_preferences.getDouble("vision::errorTolerance",
                                                                Constants.DEFAULT_VISION_ERROR_TOLERANCE),
                                                () -> this.m_preferences.getDouble("vision::maximumSpeed",
                                                                Constants.DEFAULT_VISION_MAX_SPEED),
                                                () -> this.m_preferences.getDouble("vision::maximumTargetOffset",
                                                                Constants.DEFAULT_VISION_BOUNDS)));

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Activates bindings to the autonomous command from the button box.
         */
        private void configureButtonBindings() {
                // When the activate autonomous button is pushed, turn on the autonomous command
                this.m_activateAutonomousButton.toggleWhenPressed(this.autonomousCommand);
        }

        /**
         * Gets the currently selected teleop command for the robot.
         * 
         * @return the teleop command for the robot
         */
        public Command getTeleopCommand() {
                // Check if we should be using rhino drive or arcade drivie
                if (this.m_preferences.getBoolean("drive::useRhino", true)) {
                        return this.teleopCommand; // Use a standard rhino command
                }

                // Return the robot's teleop command
                return this.fallbackTeleopCommand;
        }
}
