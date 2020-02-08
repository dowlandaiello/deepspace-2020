package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Collections;
import java.lang.Comparable;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

    /* The state of the command. This includes all historical data. */
    private final State state;

    /**
     * Axis represents a generic limelight vision axis.
     **/
    public static enum Axis {
        X, Y, Z
    }

    /**
     * State represents the current state of the command.
     *
     * @author Dowland Aiello
     **/
    private static class State {
        /* The offsets on each axis from the target. */
        ArrayList<Entry<Double>> targetOffsets;

        /* Whether or not a target is present in the camera's field of view. */
        Entry<Boolean> targetPresence;

        /* The number of frames until the state will restart. */
        private int maxFrames;

        /* The current frame number. */
        private int currentFrame;

        /**
         * Creates a new state with the given data sources.
         *
         * @param maxFrames the maximum number of frames until the state will be reset
         **/
        public State(int maxFrames) {
            // Make a new array to store historical offset data in
            this.targetOffsets = new ArrayList<Entry<Double>>() {
                {
                    add(new Entry<Double>(Entry.Type.AVERAGE_ROTATIONAL_OFFSET));
                    add(new Entry<Double>(Entry.Type.AVERAGE_Y_OFFSET));
                    add(new Entry<Double>(Entry.Type.AVERAGE_ZED_DISTANCE_OFFSET));
                }
            };

            this.targetPresence = new Entry<Boolean>(Entry.Type.AVERAGE_HAS_TARGET);
            this.maxFrames = maxFrames;
            this.currentFrame = 0;
        }

        /**
         * Inserts the given offsets into the state's history.
         *
         * @param offsets the offset values to use
         **/
        public void putValues(double[] offsets, boolean hasTarget) {
            // If we're past the max number of frames, reset the state
            if (this.currentFrame > this.maxFrames) {
                this.reset();
            }

            // Increment the current frame
            this.currentFrame++;

            // Iterate through each of the historical target offset values
            for (int i = 0; i < this.targetOffsets.size() && i < offsets.length; i++) {
                // Insert each of the offset values
                this.targetOffsets.get(i).push(offsets[i]);
            }

            this.targetPresence.push(hasTarget);
        }

        /**
         * Resets the state.
         **/
        private void reset() {
            // Make a new array to store historical offset data in
            this.targetOffsets = new ArrayList<Entry<Double>>() {
                {
                    add(new Entry<Double>(Entry.Type.AVERAGE_ROTATIONAL_OFFSET));
                    add(new Entry<Double>(Entry.Type.AVERAGE_Y_OFFSET));
                    add(new Entry<Double>(Entry.Type.AVERAGE_ZED_DISTANCE_OFFSET));
                }
            };

            this.targetPresence = new Entry<Boolean>(Entry.Type.AVERAGE_HAS_TARGET);
            this.currentFrame = 0;
        }

        /**
         * Gets the offset values contained inside the state.
         *
         * @return the offset values contained inside the state
         **/
        public double[] getOffsets() {
            return new double[] { this.targetOffsets.get(0).getAverage(), this.targetOffsets.get(1).getAverage(),
                    this.targetOffsets.get(2).getAverage() };
        }

        /**
         * Checks whether or not a target is present in the camera's field of view.
         *
         * @return whether or not a target exists in the camera's field of view
         **/
        public boolean hasTarget() {
            return this.targetPresence.getMode();
        }

        /**
         * Checks whether or not the target has been acquired.
         **/
        public boolean hasFinished(double errorTolerance) {
            return this.currentFrame > this.maxFrames / 2 && !(this.needsCorrectionOnAxis(Axis.X, errorTolerance)
                    || this.needsCorrectionOnAxis(Axis.Y, errorTolerance)
                    || this.needsCorrectionOnAxis(Axis.Z, errorTolerance));
        }

        /**
         * Checks whether or not correction is necessary on the given axis.
         *
         * @return whether or not correction is necessary for a particular axis
         **/
        public boolean needsCorrectionOnAxis(Axis axis, double errorTolerance) {
            switch (axis) {
            case X:
                return Math.abs(this.targetOffsets.get(0).getAverage())
                        / Constants.DEFAULT_VISION_BOUNDS[0] > errorTolerance;
            case Y:
                return Math.abs(this.targetOffsets.get(1).getAverage())
                        / Constants.DEFAULT_VISION_BOUNDS[1] > errorTolerance;
            default:
                return Constants.DEFAULT_VISION_BOUNDS[2] - this.targetOffsets.get(2).getAverage() > errorTolerance;
            }
        }

        /**
         * Entry represents a generic state entry for the command. This used to
         * represent historical values for the command.
         **/
        private static class Entry<T extends Comparable<T>> {
            /* The type of the entry. */
            private Type entryType;

            /* Individual entries inside the entry at each frame. */
            private ArrayList<T> historicalValues;

            /* The number of times that each value occurs in the Entry. */
            private HashMap<T, Integer> frequencies;

            /* The most often occurring value in the historical values of the entry. */
            private T mode;

            /**
             * Initializes a new Entry.
             *
             * @param type the type of entry
             **/
            public Entry(Type type) {
                this.entryType = type;
                this.historicalValues = new ArrayList<>();
                this.frequencies = new HashMap<>();
                this.mode = null;
            }

            /**
             * Entry represents a generic type of entry stored in the command's state.
             **/
            static enum Type {
                /* Whether or not a target has been recorded in more than half of the frames. */
                AVERAGE_HAS_TARGET,

                /* The average offset along the X axis over the specified number of frames. */
                AVERAGE_ROTATIONAL_OFFSET,

                /* The average offset along the Z axis over the specified number of frames. */
                AVERAGE_ZED_DISTANCE_OFFSET,

                /* The average offset along the Y axis over the specified number of frames. */
                AVERAGE_Y_OFFSET,
            }

            /**
             * Gets the mode of the historical values stored in the entry.
             *
             * @return the most often occurring value stored in the entry
             **/
            private T getMode() {
                return this.mode;
            }

            /**
             * Gets the median of the historical values stored in the entry.
             *
             * @return the median value stored in the entry
             **/
            private T getMedian() {
                // Sort each of the values contained in the set of historical values
                Collections.sort(this.historicalValues);

                // Return the middle value from the sorted historical values list
                return this.historicalValues.get(this.historicalValues.size() / 2);
            }

            /**
             * Gets the average of the historical values stored in the entry through the
             * respective average mode.
             *
             * @return the mean of the historical values stored in the entry
             **/
            public T getAverage() {
                switch (this.entryType) {
                case AVERAGE_HAS_TARGET:
                    return this.getMode();
                default:
                    return this.getMedian();
                }
            }

            /**
             * Gets the type of the entry.
             *
             * @return the type of the entry
             **/
            public Type getEntryType() {
                return this.entryType;
            }

            /**
             * Pushes a value onto the state entry.
             *
             * @param value the value that will be pushed onto the entry
             **/
            public void push(T value) {
                // Increment the frequency for the value provided
                this.frequencies.put(value, this.frequencies.getOrDefault(value, 0) + 1);

                // Add the value into the set of historical values contained in the state entry
                this.historicalValues.add(value);

                // Recalculate the mode for the entry
                if (this.mode == null || this.frequencies.get(value) > this.frequencies.getOrDefault(mode, 0)) {
                    this.mode = value;
                }
            }
        }
    }

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

        /**
         * Initializes a new Configuration for the MoveToReflectiveTarget command with
         * the given parameters.
         *
         * @param kP             the proportional value for this command's PID loop
         * @param kChange        a multiplier for the final output value of the
         *                       command's PID loop
         * @param errorTolerance the acceptable range that the output of this command
         *                       will be from the target value
         * @param maximumSpeed   the maximum speed that the robot will run during the
         *                       execution of this command
         */
        public Configuration(DoubleSupplier kP, DoubleSupplier kChange, DoubleSupplier errorTolerance,
                DoubleSupplier maximumSpeed) {
            // Set up the configuration using the given constraints
            this.kP = kP;
            this.kChange = kChange;
            this.errorTolerance = errorTolerance;
            this.maximumSpeed = maximumSpeed;
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
        this.state = new State(Constants.TARGETLESS_FRAMES_TO_VISION_STOP);

        // Drivetrain and vision subsystems need to be established in order for this
        // command to work
        addRequirements(drivetrain, vision);
    }

    /**
     * Calculates the percentage value of each offset along an assumed X, Y, and Z
     * axis.
     * 
     * @param offsets each of the X, Y, and Z offset values
     * @return the corrected offset values expressed as percentages of the maximum
     *         boundary
     */
    private double[] normalizeOffsets(double[] offsets) {
        Axis[] axes = { Axis.X, Axis.Y, Axis.Z };

        // Normalize each of the provided offsets
        for (int i = 0; i < offsets.length && i < axes.length; i++) {
            // Normalize the offset
            offsets[i] = this.normalizeOffset(offsets[i], axes[i]);
        }

        return offsets;
    }

    /**
     * Calculates a target offset, considering a given maximum offset from the
     * center of the limelight view, in conjunction with consideration to the
     * value's status as a negative or positive value.
     */
    private double normalizeOffset(double offset, Axis axis) {
        // The original offset value
        double original = offset;

        // Convert the raw offset to a percentage of the boundary definition
        switch (axis) {
        case X:
            offset = Math.abs(offset) / Constants.DEFAULT_VISION_BOUNDS[0];

            break;
        case Y:
            offset = Math.abs(offset) / Constants.DEFAULT_VISION_BOUNDS[1];

            break;
        case Z:
            offset = Constants.DEFAULT_VISION_BOUNDS[2] - offset;

            break;
        }

        // If the offset is negative, compare it against the negative max number of
        // degrees. Otherwise, compare it against
        // the positive version.
        return original < 0 ? -Math.pow(offset, this.cfg.getkChange()) : Math.pow(offset, this.cfg.getkChange());
    }

    /**
     * Executes the command.
     */
    @Override
    public void execute() {
        // Update the state of the command
        this.state.putValues(new double[] { this.m_vision.tx(), this.m_vision.ty(), this.m_vision.ta() },
                this.m_vision.hasTarget());

        // Get each of the offset values from the limelight
        double[] offsets = this.normalizeOffsets(this.state.getOffsets());

        // Check if we have a target
        boolean hasTarget = this.state.hasTarget();

        // If we don't have a target to lock on to, we can stop execution
        if (!hasTarget) {
            return;
        }

        // Check that we need to correct for X axis error
        if (this.state.needsCorrectionOnAxis(Axis.X, this.cfg.getErrorTolerance())) {
            // Calculate the gain with the x offset
            double gain = this.cfg.getKp() * offsets[0] * this.cfg.getMaximumSpeed();

            // Spin in one spot using the provided gain variable
            this.m_drivetrain.drive(Type.RHINO, new double[] { -gain, gain });
        } else if (this.state.needsCorrectionOnAxis(Axis.Z, this.cfg.getErrorTolerance())) {
            // Calculate the gain with the z offset
            double gain = this.cfg.getKp() * offsets[2] * this.cfg.getMaximumSpeed();

            // Move forward and back using the gain variable
            this.m_drivetrain.drive(Type.RHINO, new double[] { gain, gain });
        } else if (this.state.needsCorrectionOnAxis(Axis.Y, this.cfg.getErrorTolerance())) {
            // Calculate the gain with the y offset
            double gain = this.cfg.getKp() * offsets[1] * this.cfg.getMaximumSpeed();

            // Move forward and back using the gain variable
            this.m_drivetrain.drive(Type.RHINO, new double[] { gain, gain });
        }
    }

    /**
     * Returns whether or not the command has finished executing.
     *
     * @return whether or not the command has completed
     */
    @Override
    public boolean isFinished() {
        // If there aren't any targets, we're done. Or, if we've moved to the target,
        // we're done.
        return this.state.hasFinished(this.cfg.getErrorTolerance()) || !this.state.hasTarget();
    }
}
