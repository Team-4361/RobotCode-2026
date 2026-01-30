package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command
{
    private final IntakeSubsystem intake;
    private final double speed;

    
        public IntakeCommand(IntakeSubsystem intake, double speed)
        {
            this.intake = intake;
            addRequirements(intake);
            this.speed = speed;
        }
    
        @Override
        public void initialize() 
        {
            intake.SetMotorSpeed(speed);
        }

        @Override
        public void execute()
        {
            intake.SetMotorSpeed(speed);
        }

        @Override
        public void end(boolean interrupted) 
        {
            intake.stop();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
}