/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class IntakeRelease extends CommandBase {
  /**
   * Creates a new IntakeRelease.
   */
  public IntakeRelease() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    System.out.println("release");
    Robot.intakeSubsystem.releaseIntake();
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return true;
  }
}
