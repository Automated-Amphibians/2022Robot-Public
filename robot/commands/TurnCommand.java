package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TurnCommand extends CommandBase {
    double distance;
    
    public TurnCommand(double inches) {
        distance = inches;
        
    }

    @Override
    public void initialize() {
        Robot.drivetrain.zeroSensors();
        Robot.drivetrain.turnForInches(distance);
    }

    @Override
    public boolean isFinished() {
        return Robot.drivetrain.isAtTarget();
    }
}
