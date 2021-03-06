/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory %%of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Drive;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static ColourSubsystem colourSubsystem = new ColourSubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static UltraSonicSubsystem ultraSonicSubsystem = new UltraSonicSubsystem();
  // public static CameraSubsystem cameraSubsystem = new CameraSubsystem();
  public static VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static BeltSubsystem beltSubsystem = new BeltSubsystem();
  public static CameraSubsystem cameraSubsystem = new CameraSubsystem();
  public static WheelOfFortuneSubsystem wheelOfFortuneSubsystem = new WheelOfFortuneSubsystem();
public static Object AutonomousCommand;
  public Drive drive;
  public Shooter shooter;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  public void robotInit() {
    oi = new OI();
    visionSubsystem.startCamera();
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  
  public void disabledInit() {
  }

  
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.execute();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    drive = new Drive(oi.getController());
    // drive.start();
    drive.initialize();
  }

  /**
   * This function is called periodically during operator control.
   */
  
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    drive.execute();
    colourSubsystem.getColour();
    ultraSonicSubsystem.getBackDistance();
    visionSubsystem.getAdjustmentAngle();
    beltSubsystem.checkBelts();
  }

  /**
   * This function is called periodically during test mode.
   */
  
  public void testPeriodic() {
  }
}
