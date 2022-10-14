package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystem.Intake;

public class IntakeRollOutCommand extends CommandBase {
    double power;
    double timeToDo = 2;
    double initTime;

    public IntakeRollOutCommand(double powerToPutIn) {
        power = powerToPutIn;

    }


    @Override
    public void initialize() {
        
        initTime = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - initTime <= timeToDo){
            Intake.getInstance().rollOut(power);
        }
    }


    @Override
    public boolean isFinished() {
        return Robot.drivetrain.isAtTarget();
    }
}