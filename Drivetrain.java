package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import frc.robot.subsystem.OI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private TalonSRX leftMaster;
    private TalonSRX leftFollower;
    private TalonSRX rightMaster;
    private TalonSRX rightFollower;

    OI controller = new OI();
    

    public Drivetrain() {
        leftMaster = new TalonSRX(2);
        leftFollower = new TalonSRX(4);
        rightMaster = new TalonSRX(1);
        rightFollower = new TalonSRX(3);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

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

    @Override
    public void periodic() {
        controller.updateInputs();
        arcadeDrive(controller.robotVelocity, controller.robotTurnSpeed);
    }
    
}
