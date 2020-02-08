/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Constants holds a list of robot-wide constant definitions.
 */
public final class Constants {
    /*
     * The default multiplier for the distance moved in the
     * MoveToReflectiveTargetCommand.
     */
    public static double DEFAULT_VISION_KP = 1.0;

    /*
     * A secondary multiplier for the distance moved in the
     * MoveToReflectiveTargetCommand. This multiplier is applied in a logarithmic
     * fashion. For example, a value of 0.4 causes the robot to slow down as it
     * reaches the target output.
     */
    public static double DEFAULT_VISION_KCHANGE = 0.4;

    /*
     * The default number of degrees that a target may be from the center of the
     * vision camera's view.
     */
    public static double DEFAULT_VISION_ERROR_TOLERANCE = 0.25;

    /* The maximum speed of the vision command. */
    public static double DEFAULT_VISION_MAX_SPEED = 0.75;

    /* The degree radius of the vision camera. */
    public static double[] DEFAULT_VISION_BOUNDS = new double[] { 27.0, 20.5, 2.0 };

    /*
     * The number of calls to the vision command in which there must not be a target
     * for the command to be stopped.
     */
    public static int TARGETLESS_FRAMES_TO_VISION_STOP = 10;

    /* Definitions for different motor device numbers */
    public static int FRONTLEFTMOTOR = 3;
    public static int BACKLEFTMOTOR = 2;
    public static int FRONTRIGHTMOTOR = 1;
    public static int BACKRIGHTMOTOR = 0;
    public static int ARTICULATIONMOTOR = 13;
    public static int ROLLERMOTOR = 12;
    public static int ROLLERMOTORSLAVE = 9;
    public static int WRISTTALON = 7;
    public static int LIFTMOTOR = 4;
    public static int LIFTMOTORSLAVE = 5;
    public static int LEGSMOTOR = 8;

    public int HLVL1 = 18; // NOT IN USE
    public int CSC1 = 38; // good
    public int RSH2 = 40; // good
    public int RSH3 = 65; // good
    public int RSC1 = 27;
    public int RSC2 = 50; // good
    public int RSC3 = 67; // good
    public int FSH1 = 19; // good (pneu)
    public int ZERO = 0; // good
}
