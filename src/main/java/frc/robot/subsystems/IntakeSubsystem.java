/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Servo;
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
  public boolean isOn;
  public boolean isActive;
  public Servo rightServo;
  public Servo leftServo;

  public IntakeSubsystem() {
    intake = new VictorSPX(RobotMap.intakeArm);
    belt = new VictorSPX(RobotMap.horizontalBelt);
    isOn = false;
    isActive = false;
    leftServo = new Servo(RobotMap.leftIntakeServo);
    rightServo = new Servo(RobotMap.rightIntakeServo);
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void stop() {
    intake.set(ControlMode.PercentOutput, 0);
    belt.set(ControlMode.PercentOutput, 0);
    isOn = false;
  }

  public void start(){
    double frontDistance = Robot.ultraSonicSubsystem.getFrontDistance();
    frontDistance = 100;
    intake.set(ControlMode.PercentOutput, -0.5);
    if (Robot.beltSubsystem.beltTimer.get() < 0.1 && Robot.beltSubsystem.isOn) {
      belt.set(ControlMode.PercentOutput, 0);
      isOn = false;
    } else if (frontDistance > 70) {
      belt.set(ControlMode.PercentOutput, -0.8);
      isOn = true;
    } else if (Robot.beltSubsystem.ballCount == 4) {
      Robot.beltSubsystem.ballCount += 1;
      belt.set(ControlMode.PercentOutput, 0);
      isOn = false;
    }
  }
  public void startBelt() {
    belt.set(ControlMode.PercentOutput,- 0.8);
  }
  public void stopBelt() {
    belt.set(ControlMode.PercentOutput, 0);
  }
  public void releaseIntake() {
    rightServo.set(0.6);
    leftServo.set(0.6);
  }
}

