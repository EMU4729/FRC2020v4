/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public class ColourSubsystem implements Subsystem {
  private ColorSensorV3 colourSensor;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  public ColourSubsystem() {
      colourSensor = new ColorSensorV3(i2cPort);
  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public void getColour() {
      System.out.println(colourSensor.getRed() + ", " + colourSensor.getGreen() + ", " + colourSensor.getBlue());
    //   return colourSensor.getColour();
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop() {
  }
}
