package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private TalonSRX leftMaster;
    private TalonSRX leftFollower;
    private TalonSRX rightMaster;
    private TalonSRX rightFollower;    

    AHRS navX;
    private boolean calibration_complete = false;

    boolean turnSpeedWasZero = false;
    double stabilizationSetPoint = 0;

    private final double clicksPerInch = 4096 / (6 * Math.PI);
    double targetRangeRight = 20;
    double targetRangeLeft = 30;

    public Drivetrain() {
        leftMaster = new TalonSRX(4);
        leftFollower = new TalonSRX(2);
        rightMaster = new TalonSRX(1);
        rightFollower = new TalonSRX(3);

        // Motors on the right are running in the opposite direction of the direction that they are suppose to go
        rightMaster.setInverted(true);
        rightFollower.setInverted(true);

        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);

        // rightMaster and leftMaster are motor controllers that are connected to the encoders
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        rightMaster.setSensorPhase(true);
        leftMaster.setSensorPhase(true);

        rightMaster.config_kP(0, 0.18);
        leftMaster.config_kP(0, 0.18);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        zeroSensors();

        try {
            navX = new AHRS(SPI.Port.kMXP);
          } catch (RuntimeException ex) {
            System.out.println("Error instantiating navX MXP:  " + ex.getMessage());
            // FrogLogger.logMsg("ERROR GETTING GYRO" ,kSource,true);
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

    public void zeroSensors() {
        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
    }

    public void arcadeDrive(double velocity, double turnSpeed) {
        
        
        if (turnSpeed == 0) {
            if (!turnSpeedWasZero) {
                stabilizationSetPoint = navX.getYaw();
            } else if (velocity != 0) {
                double error = (navX.getYaw() - stabilizationSetPoint);
                error = Math.abs(error) < 2 ? 0 : error;

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
        rightMaster.set(ControlMode.PercentOutput, right);

        
    }

    public void driveForInches(double inches) {
        double targetPosition = inches * clicksPerInch;
        SmartDashboard.putNumber("Target Position", targetPosition);
        leftMaster.set(ControlMode.Position, targetPosition);
        rightMaster.set(ControlMode.Position, targetPosition);
    }

    public void turnForInches(double inches) {
        double targetPosition = inches * clicksPerInch;
        SmartDashboard.putNumber("Target Position", targetPosition);
        leftMaster.set(ControlMode.Position, targetPosition);
        rightMaster.set(ControlMode.Position, -targetPosition);
    }

    public boolean isAtTarget() {
        return (Math.abs(leftMaster.getClosedLoopError()) <= targetRangeLeft) && (Math.abs(rightMaster.getClosedLoopError()) <= targetRangeRight);

    }
    public void teleopPeriodic() {
        OI.getInstance().updateInputs();
        arcadeDrive(OI.getInstance().robotVelocity, OI.getInstance().robotTurnSpeed);
        OI.getInstance().printSpeedLimits();
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftEncoder", leftMaster.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("rightEncoder", rightMaster.getSelectedSensorPosition(0));
        SmartDashboard.putBoolean("isAtTarget", isAtTarget());
        SmartDashboard.putNumber("Gyro Angle", navX.getYaw());
    }
    
    public void stop() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        leftFollower.set(ControlMode.PercentOutput, 0);
    }

    public void resetGyro() {
        navX.reset();
    }
}
