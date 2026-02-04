package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class WinchUp extends Command {
    public final ClimberSubsystem winch;
    public WinchUp(ClimberSubsystem subsystem) {
        this.winch = subsystem;

        addRequirements(winch); 
    }

 @Override
         //Prepares to move the winch up
    public void initialize()
    {

        winch.winchMoveUp();
    }
        //Moves the winch up
    public void execute()
    {
        winch.winchMoveUp();
    }

    public void end() 
    {
        winch.stopWinch();
    }

    @Override
    public void end(boolean interrupted) {
        winch.stopWinch();
    }


    @Override
    public boolean isFinished()
    {
        return false; //Checks if it is at its target position
    }
}