package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem.Type;

/**
 * Moves the robot to a reflective target, within a specified distance from the
 * target.
 *
 * @author Dowland Aiello
 */
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
         * A multiplier for the final output value of the command's PID loop. Applied as
         * output^kChange.
         */
        DoubleSupplier kChange;

        /*
         * The tolerance to error in this command's PID loop. This is the amount that
         * the error vlaue can deviate from the target value.
         */
        DoubleSupplier errorTolerance;

        /*
         * The maximum speed that the robot will run during the execution of this
         * command.
         */
        DoubleSupplier maximumSpeed;

        /*
         * The maximum number of degrees that a target may be offset from the center of
         * the limelight view.
         */
        DoubleSupplier maximumTargetOffset;

        /**
         * Initializes a new Configuration for the MoveToReflectiveTarget command with
         * the given parameters.
         * 
         * @param kP                  the proportional value for this command's PID loop
         * @param kChange             a multiplier for the final output value of the
         *                            command's PID loop
         * @param errorTolerance      the acceptable range that the output of this
         *                            command will be from the target value
         * @param maximumSpeed        the maximum speed that the robot will run during
         *                            the execution of this command
         * @param maximumTargetOffset the maximum number of degrees that a target may be
         *                            offset from the center of the limelight view
         */
        public Configuration(DoubleSupplier kP, DoubleSupplier kChange, DoubleSupplier errorTolerance,
                DoubleSupplier maximumSpeed, DoubleSupplier maximumTargetOffset) {
            // Set up the configuration using the given constraints
            this.kP = kP;
            this.kChange = kChange;
            this.errorTolerance = errorTolerance;
            this.maximumSpeed = maximumSpeed;
            this.maximumTargetOffset = maximumTargetOffset;
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

        /**
         * Gets the maximum speed that this command will run at.
         * 
         * @return the maximum speed at which the robot will run during the execution of
         *         this command
         */
        public double getMaximumSpeed() {
            // Return the configuration's maximum speed variable
            return this.maximumSpeed.getAsDouble();
        }

        /**
         * Gets the maximum number of degrees that a target may be offset from the
         * center of the limelight view.
         * 
         * @return the maximum number of degrees that a target may be offset from the
         *         center of the limelight view
         */
        public double getMaximumTargetOffset() {
            // Return the configuration's maximum offset variable
            return this.maximumTargetOffset.getAsDouble();
        }

        /**
         * Gets the kChange value for this command's config.
         * 
         * @return the kChange for the command
         */
        public double getkChange() {
            // Return the kChange
            return this.kChange.getAsDouble();
        }
    }

    /**
     * Initializes a new MoveToReflectiveTargetCommand with the given constraints.
     */
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
     * Calculates a target offset, considering a given maximum offset from the
     * center of the limelight view, in conjunction with consideration to the
     * value's status as a negative or positive value.
     */
    private double normalizeOffset(double offset) {
        // Get the maximum number of degrees that the offset may be from the center.
        // This represents the percentage, rather than the pure number of degrees, that
        // the offset is from the center
        // of the limelight view.
        double maxTargetOffset = this.cfg.getMaximumTargetOffset();

        // If the offset is negative, compare it against the negative max number of
        // degrees. Otherwise, compare it against
        // the positive version.
        return offset < 0 ? -Math.pow(offset / -maxTargetOffset, this.cfg.getkChange())
                : Math.pow(offset / maxTargetOffset, this.cfg.getkChange());
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
        double offsetX = this.normalizeOffset(this.m_vision.tx());
        double offsetY = this.normalizeOffset(this.m_vision.ty());

        // Collect each of the required values (kP and errorTolerance) from the
        // dashboard
        double kP = this.cfg.kP.getAsDouble();
        double errorTolerance = this.cfg.errorTolerance.getAsDouble();

        // Calculate the rotational gain we need to drive with
        double rotationalGain = kP * offsetX * this.cfg.getMaximumSpeed();

        // Calculate the amount we need to move forward
        double forwardGain = kP * offsetY * this.cfg.getMaximumSpeed();

        // Check if we need to correct for X at all
        if (Math.abs(offsetX) > errorTolerance) {
            // Drive to correct for the X
            this.m_drivetrain.drive(Type.RHINO, new double[] { rotationalGain, -rotationalGain });
        } else if (Math.abs(offsetY) > errorTolerance) {
            // Drive to correct for the Y
            this.m_drivetrain.drive(Type.RHINO, new double[] { forwardGain, forwardGain });
        }
    }
}