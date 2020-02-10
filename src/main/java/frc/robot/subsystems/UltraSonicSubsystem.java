/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class UltraSonicSubsystem implements Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DigitalOutput frontTrigger = new DigitalOutput(RobotMap.frontUltraSonicTrigger);
  private DigitalInput frontEcho = new DigitalInput(RobotMap.frontUltraSonicEcho);
  private Ultrasonic frontSensor = new Ultrasonic(frontTrigger,frontEcho);
  private DigitalOutput backTrigger = new DigitalOutput(RobotMap.backUltraSonicTrigger);
  private DigitalInput backEcho = new DigitalInput(RobotMap.backUltraSonicEcho);
  private Ultrasonic backSensor = new Ultrasonic(backTrigger,backEcho);
  
  public UltraSonicSubsystem() {
    backSensor.setAutomaticMode(true);
    frontSensor.setAutomaticMode(true);
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public double getBackDistance() {
    double distance = backSensor.getRangeMM();

    SmartDashboard.putNumber("Back Sensor Distance", distance);
    return distance;
  }

  public double getFrontDistance() {
    double distance = frontSensor.getRangeMM();

    SmartDashboard.putNumber("Front Sensor Distance", distance);
    return distance;
  }

}
