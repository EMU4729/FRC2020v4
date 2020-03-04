/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class WheelOfFortune extends CommandBase {
  JoystickButton button;

  public WheelOfFortune(JoystickButton button) {
    this.button = button;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    Robot.wheelOfFortuneSubsystem.start();
  }

  // Make this return true when this Command  no longer needs to run execute()
  public boolean isFinished() {
    if (!button.get()) {
      Robot.wheelOfFortuneSubsystem.stop();
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  public void end() {
    Robot.wheelOfFortuneSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  public void interrupted() {
    Robot.wheelOfFortuneSubsystem.stop();
  }
}
