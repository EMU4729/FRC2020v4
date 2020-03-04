/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public class DriveSubsystem implements Subsystem {
  private TalonSRX leftFrontMotor;
  private TalonSRX rightFrontMotor;
  private TalonSRX leftBackMotor;
  private TalonSRX rightBackMotor;
  private double direction; //1 for shooter side, -1 for intake side

  public DriveSubsystem() {
    leftFrontMotor = new TalonSRX(RobotMap.leftFrontMotor);
    rightFrontMotor = new TalonSRX(RobotMap.rightFrontMotor);
    leftBackMotor = new TalonSRX(RobotMap.leftBackMotor);
    rightBackMotor = new TalonSRX(RobotMap.rightBackMotor);
    // rightMotor.setInverted(true); 
    direction = 0.75;
    SmartDashboard.putString("Direction", "Shooter side");
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

  public static final double deadzone = 0.1;
  public void drive(double forwards, double turning) {
    // turning *= 0.8;
    double forwardsSign = Math.signum(forwards);
    forwards = (Math.abs(forwards) - deadzone) / (1 - deadzone);
    if (forwards < 0) forwards = 0;
    forwards *= forwardsSign;

    double turningSign = Math.signum(turning);
    turning = (Math.abs(turning) - deadzone) / (1 - deadzone);
    if (turning < 0) turning = 0;
    turning *= turningSign;

    double left = forwards + direction*turning;
    double right = forwards - direction*turning;
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
    // left *= 0.5;
    // right *= 0.5;
    setMotors(left, right);
  }

  public void toggleDirection() {
    direction *= -1;
    if (direction > 0) {
      SmartDashboard.putString("Direction", "Shooter side");
    } else {
      SmartDashboard.putString("Direction", "Intake side");
    }
  }

  public void setMotors(double left, double right) {
    leftFrontMotor.set(ControlMode.PercentOutput, left);
    leftBackMotor.set(ControlMode.PercentOutput, left);
    rightFrontMotor.set(ControlMode.PercentOutput, right);
    rightBackMotor.set(ControlMode.PercentOutput, right);
  }
}
