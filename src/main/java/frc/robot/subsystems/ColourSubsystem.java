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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public class ColourSubsystem implements Subsystem {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


  public String getColour() {
    Color detectedColor = m_colorSensor.getColor();

    int colour = 0;
    double red = detectedColor.red;
    double green = detectedColor.green;
    double blue = detectedColor.blue;
    String colours [] = {"blue","green","red","yellow"};    
    int shift = 0; 

    if (Math.max(Math.max(red,green),blue) == blue) {
      colour = 0;
    } else if (Math.max(Math.max(red,green),blue) == red) {
      colour = 2;
    } else if (Math.max(Math.max(red,green),blue) == green) {
      if (red > 0.3) {
        colour = 3;
      } else {
        colour = 1;
      }
    }
    colour += shift;
    colour %= 4;


    SmartDashboard.putString("Detected Color", colours[colour]);
    return colours[colour];
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void stop() {
  }
}
