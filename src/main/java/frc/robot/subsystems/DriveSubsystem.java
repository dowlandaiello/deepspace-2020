package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * DriveSubsystem is a subsystem that handles control of the drivetrain.
 *
 * @author Dowland Aiello
 */
public class DriveSubsystem extends SubsystemBase {
    public static class MotorControllerConfiguration {
        /* The front left motor controller */
        final TalonSRX frontLeftController;

        /* The front right motor controller */
        final TalonSRX frontRightController;

        /* The back left motor controller */
        final TalonSRX backLeftController;

        /* The back right motor controller */
        final TalonSRX backRightController;

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
            this.frontLeftController = new TalonSRX(frontLeftControllerPort);
            this.frontRightController = new TalonSRX(frontRightControllerPort);
            this.backLeftController = new TalonSRX(backLeftControllerPort);
            this.backRightController = new TalonSRX(backRightControllerPort);
        }

        /**
         * Drives the robot according to a left and right percentage speed.
         * 
         * @param leftPercentageSpeed  the desired speed of the left motor controllers
         * @param rightPercentageSpeed the desired speed of the right motor controllers
         */
        void drive(double leftPercentageSpeed, double rightPercentageSpeed) {
            // Set the two right motors to use the given percentage speed
            this.frontLeftController.set(ControlMode.PercentOutput, leftPercentageSpeed);
            this.frontRightController.set(ControlMode.PercentOutput, rightPercentageSpeed);

            // Set the back two controllers to use the same percentage output as the front
            // two controllers
            this.backLeftController.follow(this.frontLeftController);
            this.backRightController.follow(this.frontRightController);
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
     * @param percentageSpeeds the percentage speed values to drive with
     */
    public void drive(double[] percentageSpeeds) {
        // Drive the robot
        this.motorControllers.drive(percentageSpeeds[0], percentageSpeeds[1]);
    }
}
