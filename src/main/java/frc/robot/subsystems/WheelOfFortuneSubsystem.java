/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class WheelOfFortuneSubsystem implements Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private String gameData;
  private TalonSRX wheel;
  private int turnCount;
  private String startColour;
  private boolean rotationsOn;
  private boolean onStartColour;

  public WheelOfFortuneSubsystem() {
    wheel = new TalonSRX(RobotMap.wheelOfFortune);
    turnCount = 0;
    startColour = "R";
    rotationsOn = false;
    onStartColour = false;
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

  public boolean toColour() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    gameData = "R"; //remove in final version
    String colours [] = {"B", "G", "R", "Y"};
    int actualColour = (Robot.colourSubsystem.getColour() + 2) % 4;
    String colourLetter = colours[actualColour];
    boolean out = false;
    if(gameData.length() > 0) {
      if (colourLetter == gameData) {
        stop();
        out = true;
      } else {
        start();
      }
    } else {
      //Code for no data received yet
    }
    return out;
  }

  public boolean toRotations() {
    boolean out = false;
    String colours [] = {"B", "G", "R", "Y"};
    if (!rotationsOn) {
      startColour = colours[Robot.colourSubsystem.getColour()];
      turnCount = 0;
      onStartColour = true;
      rotationsOn = true;
      start();
    } else {
      if (!onStartColour) {
        String colour = colours[Robot.colourSubsystem.getColour()];
        if (colour == startColour) {
          onStartColour = true;
          turnCount += 1;
          if (turnCount > 2) { //Change 4 to 8 in final version
            out = true;
            stop();
            rotationsOn = false;
          }
        }
      } else {
        String colour = colours[Robot.colourSubsystem.getColour()];
        if (colour != startColour) {
          onStartColour = false;
        }
      }
    }
    return out;
  }

}
