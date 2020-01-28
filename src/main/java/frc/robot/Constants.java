/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.MoveToReflectiveTargetCommand;

/**
 * Constants holds a list of robot-wide constant definitions.
 */
public final class Constants {
    /* Default values for the MoveToReflectiveTargetCommand configuration. */
    public static MoveToReflectiveTargetCommand.Configuration DEFAULTMOVETOREFLECTIVETARGETCOMMANDCONFIGURATION = new MoveToReflectiveTargetCommand.Configuration(
            () -> 1.0, () -> 0.4, () -> 0.25, () -> 0.75, () -> 30.0);

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
