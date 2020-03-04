/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * Add your docs here.
 */
public class ShooterSubsystem implements Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX shooter;
  public boolean isOn;
  private Timer beltTimer;

  public ShooterSubsystem(){
    shooter = new TalonSRX(RobotMap.shooterMotor);
    isOn = false;
    beltTimer = new Timer();
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop() {
    shooter.set(ControlMode.PercentOutput, 0);
    isOn = false;
    beltTimer.stop();
    beltTimer.reset();
    Robot.beltSubsystem.stop();

  }

  public void shoot(){
    if (beltTimer.get() == 0) {
      beltTimer.start();
    }

    if (beltTimer.get() > 0.4) {
      Robot.beltSubsystem.startFast();
    }
    shooter.set(ControlMode.PercentOutput, -1);
    isOn = true;
    Robot.beltSubsystem.ballCount = 0;
    SmartDashboard.putNumber("Ball Count", Robot.beltSubsystem.ballCount);
  }
}
