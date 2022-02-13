package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveCommand extends CommandBase {
    double distance;
    
    
    public DriveCommand(double inches) {
        distance = inches;
    }

    @Override
    public void initialize() {
        System.out.println(Robot.drivetrain.isAtTarget());
        Robot.drivetrain.zeroSensors();
        //Robot.drivetrain.driveForInches(distance);
        Robot.drivetrain.autonDrive(0.8, 0, distance);
    }

    

    @Override
    public boolean isFinished() {
        return Robot.drivetrain.isAtTarget();
        // return false;
    }
}
