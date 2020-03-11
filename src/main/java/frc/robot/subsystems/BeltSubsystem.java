/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class BeltSubsystem implements Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX frontBelt;
  private VictorSPX backBelt;
  public Timer beltTimer;
  private Timer downTimer;
  public boolean isOn;
  private XboxController controller;
  public double ballCount;

  public BeltSubsystem() {
    frontBelt = new VictorSPX(RobotMap.frontBelt);
    backBelt = new VictorSPX(RobotMap.backBelt);
    beltTimer = new Timer();
    downTimer = new Timer();
    downTimer.start();
    controller = new XboxController(0);
    isOn = false;
    ballCount = 0;
    SmartDashboard.putNumber("Ball Count", ballCount);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop() {
    frontBelt.set(ControlMode.PercentOutput, 0);
    backBelt.set(ControlMode.PercentOutput, 0);
    isOn = false;
  }

  public void start(){
    frontBelt.set(ControlMode.PercentOutput, 0.7);
    backBelt.set(ControlMode.PercentOutput, -0.7);
  }
  public void startFast() {
    isOn = true;
    frontBelt.set(ControlMode.PercentOutput, 1);
    backBelt.set(ControlMode.PercentOutput, -1);
  }

  public void checkBelts() {
    double backDistance = Robot.ultraSonicSubsystem.getBackDistance();
    double frontDistance = Robot.ultraSonicSubsystem.getFrontDistance();
    double onTime = 0.5;
    //test with controller
    ballCount = SmartDashboard.getNumber("Ball Count", 0);
    if ((backDistance > 200 | backDistance < 100) && !isOn && (downTimer.get() > 0.5 | downTimer.get() == 0)) {
      start();
      Robot.intakeSubsystem.startBelt();
      beltTimer.reset();
      beltTimer.start();
      isOn = true;
      ballCount += 1;
      SmartDashboard.putNumber("Ball Count", ballCount);
      // System.out.print("started motors due to ultrasonic input");
    } else if (beltTimer.get() > onTime  && !Robot.shooterSubsystem.isOn) {
      stop();
      if (!Robot.intakeSubsystem.isOn) {
        Robot.intakeSubsystem.stopBelt();
      }
      
      isOn = false;
      beltTimer.stop();
      beltTimer.reset();
      downTimer.start();
      downTimer.reset();
      // System.out.print("stopped motors% due to ultrasonic input");
    }
    if (downTimer.get() > 5 && downTimer.get() < 5.2 && !isOn && false) {
      frontBelt.set(ControlMode.PercentOutput, -0.5);
      backBelt.set(ControlMode.PercentOutput, 0.5);
      
    } else if (!isOn) {
      stop();
    }
  }
}
