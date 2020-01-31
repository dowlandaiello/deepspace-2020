package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Type;

/**
 * Implements a differential drive driving command for the standard Drive
 * Subsystem.
 * 
 * @author Dowland Aiello
 */
public class DifferentialDriveCommand extends CommandBase {
    /* The drivetrain that we'll use to run the Differential Drive command. */
    private final DriveSubsystem m_drivetrain;

    /* The x and y axis inputs from the joystick. */
    private final DoubleSupplier xInput, yInput, zInput;

    /**
     * Crates a new DifferentialDriveCommand.
     * 
     * @param drivetrain the subsystem used by this command
     * @param xInput     input from the x axis of the input joystick
     * @param zInput     rotational input from the z axis of the input joystick
     */
    public DifferentialDriveCommand(DriveSubsystem drivetrain, DoubleSupplier xInput, DoubleSupplier yInput,
            DoubleSupplier zInput) {
        // Use the provided drivetrain
        this.m_drivetrain = drivetrain;

        // Use the axis inputs that the user provided
        this.xInput = xInput;
        this.yInput = yInput;
        this.zInput = zInput;
    }

    /**
     * Executes the command.
     */
    @Override
    public void execute() {
        // Drive the drivetrain with a differential drive config
        this.m_drivetrain.drive(Type.DIFFERENTIAL,
                new double[] { this.xInput.getAsDouble(), this.yInput.getAsDouble(), this.zInput.getAsDouble() });
    }

    /**
     * Returns whether or not the command has finished executing.
     * 
     * @return whether or not the command has completed
     */
    public boolean isFinished() {
        return false;
    }
}