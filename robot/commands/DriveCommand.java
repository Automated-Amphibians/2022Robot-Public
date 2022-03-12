package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveCommand extends CommandBase {
    double distance;
    
    
    public DriveCommand(double inches) {
        distance = inches;
        Robot.drivetrain.autonDrive(0.8, 0, distance);
        System.out.println("HELLOOOOOOOOO");
    }


    @Override
    public void initialize() {
        
    }

    
    @Override
    public boolean isFinished() {
        return Robot.drivetrain.isAtTarget();
    }
}