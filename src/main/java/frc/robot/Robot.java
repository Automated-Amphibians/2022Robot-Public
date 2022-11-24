// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.OI;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;

import java.util.logging.LogManager;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  
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
    
    CameraServer.startAutomaticCapture();

    Climber.getInstance().resetEncoder();

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
  public void robotPeriodic() {
    Climber.getInstance().climberDemand = -OI.getInstance().armController.getRawAxis(1);
    //System.out.println("Climber Demand: " + Climber.getInstance().climberDemand);
    Climber.getInstance().climberDemand = Math.abs(Climber.getInstance().climberDemand) < 0.01 ? 0 : Climber.getInstance().climberDemand;
    SmartDashboard.putNumber("Climber Demand", Climber.getInstance().climberDemand);
  }

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

    drivetrain.resetGyro();
    drivetrain.zeroSensors();
    drivetrain.leftMaster.setNeutralMode(NeutralMode.Brake);
    drivetrain.leftFollower.setNeutralMode(NeutralMode.Brake);
    drivetrain.rightMaster.setNeutralMode(NeutralMode.Brake);
    drivetrain.rightFollower.setNeutralMode(NeutralMode.Brake);
    Arm.getInstance().armMotor.setIdleMode(IdleMode.kBrake);
    Climber.getInstance().rotationMotor.setIdleMode(IdleMode.kBrake);
    Climber.getInstance().resetEncoder();
    Climber.getInstance().extensionMotor.setNeutralMode(NeutralMode.Brake);

    
  }

  /** This function is called periodically during autonomous. */


  double startTimestamp = 0;
  // <<<---------------------- A U T O N 1 ---------------------->>>
  boolean turned1, armDown, driven1, pickedUpBall, turned2, armUp, driven2, isFinish = false;
  // <<<---------------------- A U T O N 1 ---------------------->>>

  // Drops thet ball into the lower hub, turn around, drive out of the tarmac and lower the arm
  public void auton1() {
    if (Timer.getFPGATimestamp() - startTimestamp > 5 && turned1) {
      drivetrain.driveCommand(120);
      if (!driven1){
        Arm.getInstance().armMotor.set(-0.025);
        driven1 = true;
      }
    } else if (Timer.getFPGATimestamp() - startTimestamp > 2 && !turned1) {
        drivetrain.turnCommand(40);
        Intake.getInstance().stop();
        turned1 = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp < 2 && !turned1) {
      Intake.getInstance().rollOut(1);
    }
  }

  // Drop the cargo, turn around 180 degrees, move forward, pick up another cargo, turn back 180 degrees, move forward and score
  public void auton2() {
    if (Timer.getFPGATimestamp() - startTimestamp > 13.5 && turned1 && armDown && driven1 && pickedUpBall && turned2 && armUp && driven2) {
      Intake.getInstance().rollOut(0.8);
      Arm.getInstance().armMotor.set(0);
    } else if (Timer.getFPGATimestamp() - startTimestamp > 10.5 && turned1 && armDown && driven1 && pickedUpBall && turned2 && armUp && !driven2) {
      drivetrain.driveCommand(80);
      driven2 = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 8.75 && turned1 && armDown && driven1 && pickedUpBall && turned2 && !armUp && !driven2) {
      Arm.getInstance().armMotor.set(0.4);
      armUp = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 6 && turned1 && armDown && driven1 && pickedUpBall && !turned2 && !armUp && !driven2) {
      drivetrain.turnCommand(-40);
      Intake.getInstance().stop();
      turned2 = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 5 && turned1 && armDown && driven1 && !pickedUpBall && !turned2 && !armUp && !driven2) {
      Intake.getInstance().rollIn(0.7);
      pickedUpBall = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 4 && turned1 && armDown && !driven1 && !pickedUpBall && !turned2 && !armUp && !driven2) {
      drivetrain.driveCommand(120);
      Arm.getInstance().stop();
      driven1 = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 1.5 && !turned1 && !armDown && !driven1 && !pickedUpBall && !turned2 && !armUp && !driven2) {
        drivetrain.turnCommand(36);

        if (!armDown){
          Arm.getInstance().armMotor.set(-0.05);
          armDown = true;
        }

        Intake.getInstance().stop();;
        turned1 = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp < 1 && !turned1 && !armDown && !driven1 && !pickedUpBall && !turned2 && !armUp && !driven2) {
      Intake.getInstance().rollOut(1);
    } else {
      System.out.println("ELSE");
    }
  }

  @Override
  public void autonomousPeriodic() {
    if (startTimestamp == 0) {
      startTimestamp = Timer.getFPGATimestamp();
    }
    // auton1();
    // auton2();    
    if (Timer.getFPGATimestamp() - startTimestamp > 11 && armDown && driven1 && pickedUpBall && turned1 && armUp && driven2 && isFinish) {
      Intake.getInstance().stop();
      drivetrain.stop();
    } else if (Timer.getFPGATimestamp() - startTimestamp > 10 && armDown && driven1 && pickedUpBall && turned1 && armUp && driven2 && !isFinish) {
      Intake.getInstance().rollOut(1);
      Arm.getInstance().armMotor.set(0);
      isFinish = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 8 && armDown && driven1 && pickedUpBall && turned1 && armUp && !driven2 && !isFinish) {
      drivetrain.driveCommand(120);
      driven2 = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 7 && armDown && driven1 && pickedUpBall && turned1 && !armUp && !driven2 && !isFinish) {
      Arm.getInstance().armMotor.set(0.4);
      armUp = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 5 && armDown && driven1 && pickedUpBall && !turned1 && !armUp && !driven2 && !isFinish) {
      drivetrain.turnForInches(43.5);
      turned1 = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 4 && armDown && driven1 && pickedUpBall && !turned1 && !armUp && !driven2 && !isFinish) {
      drivetrain.stop();
      Intake.getInstance().stop();
    } else if (Timer.getFPGATimestamp() - startTimestamp > 2.5 && armDown && driven1 && !pickedUpBall && !turned1 && !armUp && !driven2 && !isFinish) {
      Intake.getInstance().rollIn(0.85);
      pickedUpBall = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp > 2 && armDown && !driven1 && !pickedUpBall && !turned1 && !armUp && !driven2 && !isFinish) {
      drivetrain.driveCommand(80);
      Arm.getInstance().armMotor.set(0);
      driven1 = true;
    } else if (Timer.getFPGATimestamp() - startTimestamp < 2 && !armDown && !driven1 && !pickedUpBall && !turned1 && !armUp && !driven2 && !isFinish) {
      Arm.getInstance().armMotor.set(-0.05);
      armDown = true;
    }

    drivetrain.periodic();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("--------------------TELEOP--------------------");
    drivetrain.resetGyro();
    drivetrain.resetDrivetrainEncoder();
    Intake.getInstance().stop();     // We might not need this line
    Arm.getInstance().stop();        // We might not need this line
    Arm.getInstance().resetEncoderValues();
    Climber.getInstance().resetEncoder();

    // have the robot hold position 
    
    drivetrain.leftMaster.setNeutralMode(NeutralMode.Brake);
    drivetrain.leftFollower.setNeutralMode(NeutralMode.Brake);
    drivetrain.rightMaster.setNeutralMode(NeutralMode.Brake);
    drivetrain.rightFollower.setNeutralMode(NeutralMode.Brake);    
    Arm.getInstance().armMotor.setIdleMode(IdleMode.kBrake);
    Climber.getInstance().rotationMotor.setIdleMode(IdleMode.kBrake);
    Climber.getInstance().extensionMotor.setNeutralMode(NeutralMode.Brake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivetrain.periodic();
    drivetrain.teleopPeriodic();
    Arm.getInstance().armPeriodic();
    Intake.getInstance().intakePeriodic();
    Climber.getInstance().climberPeriodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // stop the robot, lower the arm, turn off the intake
    drivetrain.stop();
    Arm.getInstance().stop();
    Intake.getInstance().stop();

    // you must turn on the coast mode when the robot is disabled, otherwise it won't be movable.
    drivetrain.leftMaster.setNeutralMode(NeutralMode.Coast);
    drivetrain.leftFollower.setNeutralMode(NeutralMode.Coast);
    drivetrain.rightMaster.setNeutralMode(NeutralMode.Coast);
    drivetrain.rightFollower.setNeutralMode(NeutralMode.Coast);
    Arm.getInstance().armMotor.setIdleMode(IdleMode.kCoast);
    Climber.getInstance().rotationMotor.setIdleMode(IdleMode.kCoast);
    Climber.getInstance().extensionMotor.setNeutralMode(NeutralMode.Coast);
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