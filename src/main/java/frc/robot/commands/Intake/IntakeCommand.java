package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command
{
    private final IntakeSubsystem intake;

    
        public IntakeCommand(IntakeSubsystem intake)
        {
            this.intake = intake;
            addRequirements(intake);
        }
    
}