/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class WheelOfFortuneSubsystem implements Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX wheel;
  public WheelOfFortuneSubsystem() {
    wheel = new TalonSRX(RobotMap.wheelOfFortune);
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void stop() {
    wheel.set(ControlMode.PercentOutput, 0);
  }
  public void start() {
    wheel.set(ControlMode.PercentOutput, 1);
  }
}
