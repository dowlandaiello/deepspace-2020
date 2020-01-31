package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * DriveSubsystem is a subsystem that handles control of the drivetrain.
 *
 * @author Dowland Aiello
 */
public class DriveSubsystem extends SubsystemBase {
    /**
     * Specifies a type of drive for the subsystem. Either differential (arcade) or
     * rhino.
     */
    public static enum Type {
        DIFFERENTIAL, RHINO
    }

    public static class MotorControllerConfiguration {
        /* The front left motor controller */
        Talon frontLeftController, frontRightController, backLeftController, backRightController;

        /* Controller groups for the left and right sides of the robot */
        SpeedControllerGroup right, left;

        /* A DifferentialDrive instance for the motor controller config */
        private DifferentialDrive diffDrive;

        /**
         * Initializes a new MotorControllerConfiguration with the given ports.
         *
         * @param frontLeftControllerPort  the port of the front left motor controller
         * @param frontRightControllerPort the port of the front right motor controller
         * @param backLeftControllerPort   the port of the back left motor controller
         * @param backRightControllerPort  the port of the back right motor controller
         */
        public MotorControllerConfiguration(int frontLeftControllerPort, int frontRightControllerPort,
                int backLeftControllerPort, int backRightControllerPort) {
            // Initialize each of the talons
            this.frontLeftController = new Talon(frontLeftControllerPort);
            this.frontRightController = new Talon(frontRightControllerPort);
            this.backLeftController = new Talon(backLeftControllerPort);
            this.backRightController = new Talon(backRightControllerPort);

            this.left = new SpeedControllerGroup(this.frontLeftController, this.backLeftController);
            this.right = new SpeedControllerGroup(this.frontRightController, this.backRightController);

            // Use the left and right side of the motor for the differential drive command
            // diffDrive
            this.diffDrive = new DifferentialDrive(this.left, this.right);
            this.diffDrive.setRightSideInverted(false);
        }

        /**
         * Drives the robot according to a left and right percentage speed.
         * 
         * @param leftPercentageSpeed  the desired speed of the left motor controllers
         * @param rightPercentageSpeed the desired speed of the right motor controllers
         */
        void drive(double leftPercentageSpeed, double rightPercentageSpeed) {
            // Set the left and riight side of the robot to move the desired speeds
            this.left.set(-leftPercentageSpeed);
            this.right.set(-rightPercentageSpeed);

            return;
        }

        /**
         * Drives the robot differentially.
         * 
         * @param xInput
         * @param yInput
         * @param zInput
         */
        void drive(double xInput, double yInput, double zInput) {
            this.diffDrive.setMaxOutput(((zInput - 1) / 2) * (-1));
            this.diffDrive.arcadeDrive(-yInput, -xInput);
        }
    }

    /* The motor controllers that will be used in the drive subsystem. */
    private final MotorControllerConfiguration motorControllers;

    /**
     * Initializes a new DriveSubsystem.
     */
    public DriveSubsystem(MotorControllerConfiguration motorConfig) {
        // Use the motor controller configuration provided to us by the calling command
        this.motorControllers = motorConfig;
    }

    /**
     * Drives the robot with the given percentage speed values.
     *
     * @param driveType        the manner in which the robot should drive
     * @param percentageSpeeds the percentage speed values to drive with
     */
    public void drive(Type driveType, double[] percentageSpeeds) {
        if (driveType.equals(Type.RHINO)) {
            // Drive the robot
            this.motorControllers.drive(percentageSpeeds[0], percentageSpeeds[1]);

            return;
        }

        // Use differential drive to drive the robot
        this.motorControllers.drive(percentageSpeeds[0], percentageSpeeds[1], percentageSpeeds[2]);
    }
}
