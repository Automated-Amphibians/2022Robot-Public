package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private TalonSRX leftMaster;
    private TalonSRX leftFollower;
    private TalonSRX rightMaster;
    private TalonSRX rightFollower;

    public Drivetrain() {
        // Still need to find the correct motor IDs
        leftMaster = new TalonSRX(0);
        leftFollower = new TalonSRX(1);
        rightMaster = new TalonSRX(2);
        rightFollower = new TalonSRX(3);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

    }

    public void arcadeDrive(double velocity, double turnSpeed) {
        double left = velocity - turnSpeed;
        double right = velocity + turnSpeed;
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

    @Override
    public void periodic() {
        // robotVelocity = drivetrainController.getRawAxis(1) * -1;
        // robotTurnSpeed = drivetrainController.getRawAxis(2);

        // arcadeDrive(robotVelocity, robotTurnSpeed);
    }
    
}
