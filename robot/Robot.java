// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.commands.CommandGroup;
//import frc.robot.commands.DriveCommand;
import frc.robot.subsystem.Drivetrain;
import frc.robot.commands.Auton1;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

import java.sql.Time;
import java.util.logging.LogManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static Drivetrain drivetrain;


  public Robot() {}


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    drivetrain = new Drivetrain();
    

    try {
      LogManager.getLogManager().reset();
    } catch (Exception exception) {
      System.out.println("Security Exception");
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}


  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you refer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    System.out.println("--------------------AUTONOMOUS--------------------");
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    drivetrain.resetGyro();
    drivetrain.zeroSensors();

    switch (m_autoSelected) {
      case kDefaultAuto:
        // CommandScheduler.getInstance().schedule(new Auton1()); 
        break;
      default:
        // CommandScheduler.getInstance().schedule(new Auton1()); 
        break;
    }
  }

  /** This function is called periodically during autonomous. */
  double startTimestamp = 0;
  boolean turned, driven = false;
  @Override
  public void autonomousPeriodic() {
   
    if (startTimestamp == 0) {
      startTimestamp = Timer.getFPGATimestamp();
    }

    //System.out.println("--------------------AUTONOMOUS--------------------");
    //Intake.getInstance().rollIn(0.5);
    // Intake.getInstance().rollOut(0.75);
    // drivetrain.TurnCommand(48);
    // // drivetrain.DriveCommand(75);
    
    // If it doesnt work, just comment these following lines out // Just drive out
    Intake.getInstance().rollOut(0.5);

    if (Timer.getFPGATimestamp() - startTimestamp > 5 && turned) {
      drivetrain.DriveCommand(85);
      if (!driven){
        Arm.getInstance().armMotor.set(-0.025);
        driven = true;
      }
    } else if (Timer.getFPGATimestamp() - startTimestamp > 2 && !turned) {
        drivetrain.TurnCommand(42);
        turned = true;
    }

    // Arm.getInstance().armMotor.set(-0.025);
    // drivetrain.DriveCommand(75);


    drivetrain.periodic();
    CommandScheduler.getInstance().run();

    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("--------------------TELEOP--------------------");
    drivetrain.resetGyro();
    Intake.getInstance().stop();     // We might not need this line
    Arm.getInstance().stop();        // We might not need this line
    Arm.getInstance().armEncoder.setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivetrain.periodic();
    drivetrain.teleopPeriodic();
    Arm.getInstance().armPeriodic();
    Intake.getInstance().intakePeriodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    drivetrain.stop();
    Arm.getInstance().stop();
    Intake.getInstance().stop();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}