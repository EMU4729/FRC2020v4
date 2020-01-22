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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * Add your docs here.
 */
public class ColourSubsystem implements Subsystem {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


  public void getColour() {
    Color detectedColor = m_colorSensor.getColor();

    String colour = "";
    double red = detectedColor.red;
    double green = detectedColor.green;
    double blue = detectedColor.blue;


    if (Math.max(Math.max(red,green),blue) == blue) {
      colour = "blue";
    } else if (Math.max(Math.max(red,green),blue) == red) {
      colour = "red";
    } else if (Math.max(Math.max(red,green),blue) == green) {
      if (red > 0.3) {
        colour = "yellow";
      } else {
        colour = "green";
      }
    }

    SmartDashboard.putString("Detected Color", colour);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop() {
  }
}
