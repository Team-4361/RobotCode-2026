package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class WinchDown extends Command {
    public final ClimberSubsystem winch;
    public WinchDown(ClimberSubsystem subsystem) {
        this.winch = subsystem;

        addRequirements(winch);
    }

 @Override
     //Prepares to move the winch down
    public void initialize()
    {
        winch.winchMoveDown();
    }
    
    //Moves the winch down
    public void execute()
    {
        winch.winchMoveDown();
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