/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class IntakeSubsystem implements Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX intake;
  private VictorSPX belt;

  public IntakeSubsystem() {
    intake = new VictorSPX(RobotMap.intakeArm);
    belt = new VictorSPX(RobotMap.horizontalBelt);
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void stop() {
    intake.set(ControlMode.PercentOutput, 0);
    belt.set(ControlMode.PercentOutput, 0);
  }

  public void start(){
    // Swap back sensor for front sensor when it is wired
    double backDistance = Robot.ultraSonicSubsystem.getBackDistance();
    intake.set(ControlMode.PercentOutput, 0.5);
    if (backDistance > 70) {
      belt.set(ControlMode.PercentOutput, 0.5);
    } else {
      belt.set(ControlMode.PercentOutput, 0);
    }
  }
}
