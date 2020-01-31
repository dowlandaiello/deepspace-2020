/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Type;

/**
 * An example command that uses an example subsystem.
 */
public class RhinoDriveCommand extends CommandBase {
    /* The drivetrain that we'll use to run the Rhino drive command. */
    private final DriveSubsystem m_drivetrain;

    /* The right and left joystick inputs. */
    private final DoubleSupplier leftJoystickInput, rightJoystickInput;

    /**
     * Creates a new RhinoDriveCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public RhinoDriveCommand(DriveSubsystem drivetrain, DoubleSupplier leftJoystickInput,
            DoubleSupplier rightJoystickInput) {
        // Use the provided drive train
        this.m_drivetrain = drivetrain;

        // Set the joystick inputs of the RhinoDriveCommand
        this.leftJoystickInput = leftJoystickInput;
        this.rightJoystickInput = rightJoystickInput;

        // The RhinoDriveCommand doesn't work without a drivetrain
        addRequirements(drivetrain);
    }

    /**
     * Executes the command.
     */
    @Override
    public void execute() {
        // Drive
        this.m_drivetrain.drive(Type.RHINO,
                new double[] { this.leftJoystickInput.getAsDouble(), this.rightJoystickInput.getAsDouble() });
    }

    /**
     * Returns whether or not the command has finished executing.
     * 
     * @return whether or not the command has completed
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
