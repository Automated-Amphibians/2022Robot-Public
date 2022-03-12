package frc.robot.commands;

 

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Intake;

public class Auton1 extends SequentialCommandGroup{
    public Auton1() {
        addCommands(
            new IntakeRollOutCommand(0.75),
            new WaitCommand(1.75),
            new TurnCommand(48),
            new WaitCommand(1.75),
            new DriveCommand(75)
        );
    }
}