/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static int leftFrontMotor = 3; 
  public static int rightFrontMotor = 1;
  public static int leftBackMotor = 5;
  public static int rightBackMotor = 2;
  public static int shooterMotor = 4;
  public static int intakeArm = 0;
  public static int frontBelt = 7;
  public static int backBelt = 6;
  public static int horizontalBelt = 8;
  public static int climber = 9;
  public static int wheelOfFortune = 10;
 
  public static int buttonIDA = 1;
  public static int buttonIDB = 2;
  public static int buttonIDY = 4;
  public static int buttonIDX = 3;
  public static int buttonIDRightBumper = 6;
  public static int buttonIDLeftBumper = 5;
  public static int leftStickY = 1;
  public static int buttonIDStart = 8;
  public static int buttonIDBack = 7;

  public static int backUltraSonicEcho = 9;
  public static int backUltraSonicTrigger = 8;
  public static int frontUltraSonicEcho = 7;
  public static int frontUltraSonicTrigger = 6;

  public static int leftIntakeServo = 0;
  public static int rightIntakeServo = 1;
  public static int colourArm = 3;

}

