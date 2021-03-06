/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FlipDirection;
import frc.robot.commands.IntakeArm;
import frc.robot.commands.IntakeRelease;
import frc.robot.commands.LowerColourSensor;
import frc.robot.commands.Shooter;
import frc.robot.commands.WheelOfFortune;
import frc.robot.commands.WheelToColour;
import frc.robot.commands.WheelToRotations;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  private XboxController controller = new XboxController(0);
  private JoystickButton a;
  private JoystickButton b;
  private JoystickButton y;
  private JoystickButton x;
  private JoystickButton rightBumper;
  private JoystickButton startButton;
  private JoystickButton backButton;
  private JoystickButton leftBumper;
  
  public OI() {
    a = new JoystickButton(controller, RobotMap.buttonIDA);
    b = new JoystickButton(controller, RobotMap.buttonIDB);
    y = new JoystickButton(controller, RobotMap.buttonIDY);
    x = new JoystickButton(controller, RobotMap.buttonIDX);
    startButton = new JoystickButton(controller, RobotMap.buttonIDStart);
    rightBumper = new JoystickButton(controller, RobotMap.buttonIDRightBumper);
    backButton = new JoystickButton(controller, RobotMap.buttonIDBack);
    leftBumper = new JoystickButton(controller, RobotMap.buttonIDLeftBumper);
    a.whenPressed(new Shooter(a));
    b.whenPressed(new IntakeArm(b));
    y.whenPressed(new FlipDirection());
    x.whenPressed(new LowerColourSensor(x));
    startButton.whenPressed(new IntakeRelease());
    leftBumper.whenPressed(new WheelToRotations());
    rightBumper.whenPressed(new WheelToColour());
    backButton.whenPressed(new WheelOfFortune(backButton));
    
  } 

  public XboxController getController() {
    return controller;
  }
}
