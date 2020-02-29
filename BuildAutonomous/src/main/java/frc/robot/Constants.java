/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

public static int leftMasterPort = 2;
public static int rightMasterPort = 3;

public static int leftSlavePort = 1;
public static int rightSlavePort = 4;

public static int JoystickPort = 1;

public static final int SR_port = 0;
public static final int SF_port = 1;

public static double maxOutput = 0.5;

public final static double yDeadband = 0.1;
public final static double zDeadband = 0.1;

public static int wheelDiamter = 6;
public static double wheelCircumference = Math.PI * 6;

public static final double kDistancePerRevolution = Math.PI * 0.1524; // 6 inches is radius  
public static final double kPulsesPerRevolution = 360;     
public static final double kDistancePerPulse = kDistancePerRevolution / kPulsesPerRevolution;

public static final double kP_Autonomous = 0.2;
public static final double kI_Autonomous = 0.05;
public static final double kD_Autonomous = 0.05;

public static final double kS = 0.2;
public static final double kA = 0.2;
public static final double kV = 1.8;

public static final double wheelSpacing = 10; // inches 


}

