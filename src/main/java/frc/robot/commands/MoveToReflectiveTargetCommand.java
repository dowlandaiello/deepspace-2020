package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Moves the robot to a reflective target, within a specified distance from the
 * target.
 *
 * @author Dowland Aiello
 **/
public class MoveToReflectiveTargetCommand extends CommandBase {
    /* The drivetrain that we'll use to run the MoveToReflectiiveTarget command. */
    private final DriveSubsystem m_drivetrain;

    /* The vision subsystem that we'll use for targeting. */
    private final VisionSubsystem m_vision;

    /* A configuration for this command. */
    private final Configuration cfg;

    /**
     * Configuration represents a configuration for the MoveToReflectiveTarget
     * command.
     */
    public static class Configuration {
        /*
         * The proportional value for the command's PID loop. This is the amount that
         * the input error value is multiplied by on each call.
         */
        DoubleSupplier kP;

        /*
         * The tolerance to error in this command's PID loop. This is the amount that
         * the error vlaue can deviate from the target value.
         */
        DoubleSupplier errorTolerance;

        /**
         * Initializes a new Configuration for the MoveToReflectiveTarget command with
         * the given parameters.
         * 
         * @param kP             the proportional value for this command's PID loop
         * @param errorTolerance the acceptable range that the output of this command
         *                       will be from the target value
         */
        public Configuration(DoubleSupplier kP, DoubleSupplier errorTolerance) {
            // Set up the configuration using the given constraints
            this.kP = kP;
            this.errorTolerance = errorTolerance;
        }

        /**
         * Gets the kP of the configuration.
         * 
         * @return the kP of the configuration
         */
        public double getKp() {
            // Return the configuration's current kP
            return this.kP.getAsDouble();
        }

        /**
         * Gets the error tolerance of the configuration.
         * 
         * @return the error tolerance of the configuration
         */
        public double getErrorTolerance() {
            // Return the configuration's current error tolerance variable
            return this.errorTolerance.getAsDouble();
        }
    }

    public MoveToReflectiveTargetCommand(DriveSubsystem drivetrain, VisionSubsystem vision, Configuration cfg) {
        // Use the provided drive train, vision subsystem, and configuration classes
        this.m_drivetrain = drivetrain;
        this.m_vision = vision;
        this.cfg = cfg;

        // Drivetrain and vision subsystems need to be established in order for this
        // command to work
        addRequirements(drivetrain, vision);
    }

    /**
     * Executes the command.
     */
    @Override
    public void execute() {
        // If there isn't a target in our field of view, we don't need to do any more
        // work
        if (!this.m_vision.hasTarget()) {
            return;
        }

        // Get the offset by which we need to move
        double offsetX = this.m_vision.tx();
        double offsetY = this.m_vision.ty();

        // Collect each of the required values (kP and errorTolerance) from the
        // dashboard
        double kP = this.cfg.kP.getAsDouble();
        double errorTolerance = this.cfg.errorTolerance.getAsDouble();

        // Calculate the rotational gain we need to drive with
        double rotationalGain = kP * offsetX;

        // Calculate the amount we need to move forward
        // double forwardGain = kP * offsetY;

        // Check if we need to correct for X at all
        if (Math.abs(offsetX) > errorTolerance) {
            // Drive to correct for the X
            this.m_drivetrain.drive(new double[] { rotationalGain, -rotationalGain });
        } else if (Math.abs(offsetY) > errorTolerance) {
            // this.m_drivetrain.drive(new double[] { forwardGain, forwardGain });
        }
    }
}