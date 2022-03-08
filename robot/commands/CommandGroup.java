package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandGroup extends SequentialCommandGroup{
    public CommandGroup() {
        addCommands(
            new DriveCommand(30),
            new TurnCommand(24),
            new DriveCommand(30)
        );
    }
}