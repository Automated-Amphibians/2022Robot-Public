package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private TalonSRX leftMaster;
    private TalonSRX leftFollower;
    private TalonSRX rightMaster;
    private TalonSRX rightFollower;    

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
    }

    public void zeroSensors() {
        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
    }

    public void arcadeDrive(double velocity, double turnSpeed) {
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
    }
    
    public void stop() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        leftFollower.set(ControlMode.PercentOutput, 0);
    }
}
