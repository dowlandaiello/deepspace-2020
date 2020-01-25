package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;

/**
 * VisionSubsystem implements a connector to the limelight over a given
 * transport, namely NetworkTables.
 * 
 * @author Dowland Aiello
 */
public class VisionSubsystem extends SubsystemBase {
    /**
     * LEDMode represents one of 4 LED modes available to the limelight.
     */
    public static enum LEDMode {
        DEFAULT(1), BLINK(2), ON(3), OFF(4);

        /* The value corresponding to the LEDMode. */
        private final int intVal;

        /**
         * Initializies a new LEDMode with the given corresponding integer value.
         * 
         * @param value the value corresponding to the LEDMode
         */
        private LEDMode(int value) {
            this.intVal = value;
        }

        /**
         * Converts the LEDMode to an integer.
         * 
         * @return the value of the LEDMode
         */
        public int value() {
            // Return the value of this mode
            return this.intVal;
        }
    }

    /**
     * LimelightConfiguration defines a standard configuration for the connected
     * limelight.
     */
    public static class LimelightConfiguration {
        /* The mode that the limelight's LED will use. */
        LEDMode ledMode;

        /**
         * Initializes a new LimelightConfiguration with the given paramters.
         * 
         * @param ledMode the mode that the limelight's LED should operate in
         */
        public LimelightConfiguration(LEDMode ledMode) {
            // Set the limelight's ledMode
            this.ledMode = ledMode;
        }

        /**
         * Applies the Limelight configuration to the limelight through the given
         * network tables connector.
         * 
         * @param limelightTable the NetworkTable connector to the limelight
         */
        public void applySettings(NetworkTable limelightTable) {
            // Apply the ledMode to the limelight
            limelightTable.getEntry("ledMode").setNumber(this.ledMode.value());
        }
    }

    /* A NetworkTables tabel for the limelight. */
    private NetworkTable limelightTable;

    /* A configuration for the LimelightConfiguratioin. */
    private LimelightConfiguration limelightConfiguratiion;

    /**
     * Initializes the VisionSubsystem with the given network tables configuration.
     * 
     * @param limelightTable the connector to the limelight over the NetworkTables
     *                       API
     */
    public VisionSubsystem(NetworkTable limelightTable, LimelightConfiguration limelightConfiguration) {
        // Set up the limelight tables API
        this.limelightTable = limelightTable;

        // Use the user-provided limelight configuration class
        this.limelightConfiguratiion = limelightConfiguration;

        // Apply all of our settinigs
        this.limelightConfiguratiion.applySettings(this.limelightTable);
    }

    /**
     * Whether or not the limelight has a vision target.
     * 
     * @return
     */
    public boolean hasTarget() {
        return this.tv() > 0;
    }

    /**
     * Gets the volume variable from the limelight.
     */
    public double tv() {
        return this.limelightTable.getEntry("tv").getDouble(0.0);
    }

    /**
     * Gets the x offset from the limelight.
     * 
     * @return the offset of the crosshair from the target regiion
     */
    public double tx() {
        return this.limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Gets the y offset from the limelight.
     * 
     * @return the x offset of the crosshair from the target region
     */
    public double ty() {
        return this.limelightTable.getEntry("ty").getDouble(0.0);
    }
}