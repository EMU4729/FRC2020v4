/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.RobotDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public class DriveSubsystem implements Subsystem {
  private TalonSRX leftMotor;
  private TalonSRX rightMotor;

  public DriveSubsystem() {
    leftMotor = new TalonSRX(RobotMap.leftMotorPort);
    rightMotor = new TalonSRX(RobotMap.rightMotorPort);
    rightMotor.setInverted(true); 
  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop() {
    setMotors(0, 0);
  }

  public void drive(double forwards, double turning) {
    double left = forwards+turning;
    double right = forwards-turning;
    if (Math.abs(left) > 1 || Math.abs(right) > 1) {
      if (Math.abs(left) > Math.abs(right)) {
        right *= 1/Math.abs(left);
        left *= 1/Math.abs(left);
      }
      else {
        right *= 1/Math.abs(right);
        left *= 1/Math.abs(right);
      }
    }
    setMotors(left, right);
  }

  public void setMotors(double left, double right) {
    leftMotor.set(ControlMode.PercentOutput, left);
    rightMotor.set(ControlMode.PercentOutput, right);
  }
}
