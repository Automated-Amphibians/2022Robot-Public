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
        Robot.drivetrain.zeroSensors();
        Robot.drivetrain.driveForInches(distance);
    }

    @Override
    public boolean isFinished() {
        return Robot.drivetrain.isAtTarget();
    }
}
