package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
        WPI_TalonSRX frontLeftController, frontRightController, backLeftController, backRightController;

        /* Controller groups for the left and right sides of the robot */
        SpeedControllerGroup right, left;

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
            this.frontLeftController = new WPI_TalonSRX(frontLeftControllerPort);
            this.frontRightController = new WPI_TalonSRX(frontRightControllerPort);
            this.backLeftController = new WPI_TalonSRX(backLeftControllerPort);
            this.backRightController = new WPI_TalonSRX(backRightControllerPort);
        }

        /**
         * Drives the robot according to a left and right percentage speed.
         *
         * @param driveType            the type of drive to use
         * @param leftPercentageSpeed  the desired speed of the left motor controllers
         * @param rightPercentageSpeed the desired speed of the right motor controllers
         */
        void drive(Type driveType, double leftPercentageSpeed, double rightPercentageSpeed) {
            if (driveType.equals(Type.DIFFERENTIAL)) {
                // Treat the rightPercentageSpeed as a rotational parameter
                this.frontLeftController.set(ControlMode.PercentOutput, leftPercentageSpeed,
                        DemandType.ArbitraryFeedForward, rightPercentageSpeed);
                this.backLeftController.follow(this.frontLeftController);

                this.frontRightController.set(ControlMode.PercentOutput, leftPercentageSpeed,
                        DemandType.ArbitraryFeedForward, -rightPercentageSpeed);
                this.backRightController.follow(this.frontRightController);
            }

            this.frontLeftController.set(-leftPercentageSpeed);
            this.backLeftController.follow(this.frontLeftController);

            this.frontRightController.set(rightPercentageSpeed);
            this.backRightController.follow(this.frontRightController);

            return;
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
            this.motorControllers.drive(Type.RHINO, percentageSpeeds[0], percentageSpeeds[1]);

            return;
        }

        // Use differential drive to drive the robot
        this.motorControllers.drive(Type.DIFFERENTIAL, percentageSpeeds[0], percentageSpeeds[1]);
    }
}
