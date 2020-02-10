/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
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
  private Timer beltTimer;
  private boolean isOn;

  public BeltSubsystem() {
    frontBelt = new VictorSPX(RobotMap.frontBelt);
    backBelt = new VictorSPX(RobotMap.backBelt);
    beltTimer = new Timer();
    isOn = false;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop() {
    frontBelt.set(ControlMode.PercentOutput, 0);
    backBelt.set(ControlMode.PercentOutput, 0);
  }

  public void start(){
    frontBelt.set(ControlMode.PercentOutput, 0.5);
    backBelt.set(ControlMode.PercentOutput, 0.5);
  }

  public void checkBelts() {
    double backDistance = Robot.ultraSonicSubsystem.getBackDistance();
    double frontDistance = Robot.ultraSonicSubsystem.getFrontDistance();
    int onTime = 1;

    if (backDistance < 70 && !isOn) {
      start();
      beltTimer.start();
      isOn = true;
      // System.out.print("started motors due to ultrasonic input");
    } else if (beltTimer.get() > onTime) {
      stop();
      isOn = false;
      beltTimer.stop();
      beltTimer.reset();
      // System.out.print("stopped motors due to ultrasonic input");
    }
  }
}
