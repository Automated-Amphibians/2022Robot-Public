package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    public TalonSRX leftMaster;
    public TalonSRX leftFollower;
    public TalonSRX rightMaster;
    public TalonSRX rightFollower;   
    double distance; 

    AHRS navX;
    private boolean calibration_complete = false;

    boolean turnSpeedWasZero = false;
    double stabilizationSetPoint = 0;
    boolean isTurning = false;

    private final double clicksPerInch = 4096 / (6 * Math.PI);
    // double targetRangeRight = 100;
    // double targetRangeLeft = 100;
    double targetPosition = 0;


    public Drivetrain() {
        leftMaster = new TalonSRX(3); 
        leftFollower = new TalonSRX(4);
        rightMaster = new TalonSRX(2);
        rightFollower = new TalonSRX(1);

        // Motors on the right are running in the opposite direction of the direction that they are suppose to go
        rightMaster.setInverted(true);
        rightFollower.setInverted(true);
        

        // rightMaster and leftMaster are motor controllers that are connected to the encoders
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        rightMaster.setSensorPhase(true);
        leftMaster.setSensorPhase(true);

        rightMaster.config_kP(0, 0.25);
        leftMaster.config_kP(0, 0.25);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        zeroSensors();
    }

    public void zeroSensors() {
        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
        stabilizationSetPoint = 0.0;
        
        try {
            navX = new AHRS(SPI.Port.kMXP);
          } catch (RuntimeException ex) {
            System.out.println("Error instantiating navX MXP:  " + ex.getMessage());
          }
      
          /* NavX calibration completed ~15 seconds after it is powered on */
          /* as long as the robot is still. Hold off using Yaw value until */
          /* calibration is complete */
      
          while (!calibration_complete) {
            calibration_complete = !navX.isCalibrating();
            if (!calibration_complete) {
              SmartDashboard.putString("NavX", "Calibration in Progress");
            } else {
              SmartDashboard.putString("NavX", "Calibration Completed!");
              // force reset of yaw value
              navX.zeroYaw();
            }
        }
    }


    public void resetDrivetrainEncoder() {
        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
        stabilizationSetPoint = 0.0;
    }

    public void arcadeDrive(double velocity, double turnSpeed) {
        if (turnSpeed == 0) {
            if (!turnSpeedWasZero) {
                stabilizationSetPoint = navX.getYaw();
            } else if (velocity != 0) {
                double error = (navX.getYaw() - stabilizationSetPoint);
                error = Math.abs(error) < 1 ? 0 : error;

                // -0.05 has to stay negative
                turnSpeed = -0.05 * error;
            }

            turnSpeedWasZero = true;
        } else {
            turnSpeedWasZero = false;
        }

        double left = velocity + turnSpeed;
        double right = velocity - turnSpeed;
        double ratio = 1;

        if (left > 1) {
            ratio = 1/left;
        } else if (right > 1) {
            ratio = 1/right;
        }

        left *= ratio;
        right *= ratio;

        leftMaster.set(ControlMode.PercentOutput, left);
        leftFollower.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
        rightFollower.set(ControlMode.PercentOutput, right);    
    }
    
    public void DriveCommand(double inches) {
        distance = inches;
        autonDrive(0.4, 0, distance);
    }


    public void TurnCommand (double inches) {
        distance = inches;
        zeroSensors();
        turnForInches(distance);
    }

    public void autonDrive(double velocity, double turnSpeed, double inches) {
        targetPosition = inches * clicksPerInch;
        double left;
        double right;
        System.out.println("Target Position: " + targetPosition);

        // get yaw center
        stabilizationSetPoint = navX.getYaw();


        if (!isAtTarget()) {
            // turn speed is 0 (not turning)
            if (!isTurning) {
                // find error
                double error = (navX.getYaw() - stabilizationSetPoint);
                error = Math.abs(error) < 1 ? 0 : error;
                // set adjustment factor
                turnSpeed = -0.05 * error;
            }
            

            left = velocity + turnSpeed;
            right = velocity - turnSpeed;
            double ratio = 1;

            if (left > 1) {
                ratio = 1/left;
            } else if (right > 1) {
                ratio = 1/right;
            }
                

            left *= ratio;
            right *= ratio;

            leftMaster.set(ControlMode.PercentOutput, left);
            rightMaster.set(ControlMode.PercentOutput, right); 

            leftFollower.set(ControlMode.PercentOutput, left);
            rightFollower.set(ControlMode.PercentOutput, right);

            SmartDashboard.putNumber("Left Velocity", left);
            SmartDashboard.putNumber("Right Velocity", right);
        }

        if (isAtTarget()) {
            stop();
        }
    }

    public void turnForInches(double inches) {
        targetPosition = inches * clicksPerInch;
        leftMaster.set(ControlMode.Position, targetPosition);
        rightMaster.set(ControlMode.Position, -targetPosition);
    }

    public boolean isAtTarget() {
        return (Math.abs(leftMaster.getSelectedSensorPosition(0)) >= Math.abs(targetPosition)) || (Math.abs(rightMaster.getSelectedSensorPosition(0)) >= Math.abs(targetPosition));
    }

    public void teleopPeriodic() {
        OI.getInstance().updateInputs();
        OI.getInstance().updateSpeedLimits();
        arcadeDrive(OI.getInstance().robotVelocity, OI.getInstance().robotTurnSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftEncoder", leftMaster.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("rightEncoder", rightMaster.getSelectedSensorPosition(0));
        SmartDashboard.putBoolean("isAtTarget", isAtTarget());
        SmartDashboard.putNumber("Gyro Angle", navX.getYaw());
        SmartDashboard.putNumber("Target Position", targetPosition);

        SmartDashboard.putNumber("Robot Velocity", OI.getInstance().robotVelocity);
        SmartDashboard.putNumber("Robot Turn Speed", OI.getInstance().robotTurnSpeed);
        SmartDashboard.putNumber("Drive Speed Limit", OI.getInstance().driveSpeedLimit);
        SmartDashboard.putNumber("Turn Speed Limit", OI.getInstance().turnSpeedLimit);

        SmartDashboard.putNumber("Arm Encoder Position", Arm.getInstance().armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Encoder Max Value", Arm.getInstance().maxArmEncoderValue);

        SmartDashboard.putNumber("Climber Rotation Encoder", Climber.getInstance().rMEncoder.getPosition());
    }
    
    public void stop() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        leftFollower.set(ControlMode.PercentOutput, 0);

        rightMaster.set(ControlMode.PercentOutput, 0);
        rightFollower.set(ControlMode.PercentOutput, 0);
    }

    public void resetGyro() {
        navX.reset();
    }
}