package frc.robot.commands.Hopper;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;



public class HopperCommand extends Command {
    private final HopperSubsystem hopperSubsystem;

    public HopperCommand(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
          hopperSubsystem.feed(0.5);
    }

    @Override
    public void execute() {
       
        hopperSubsystem.feed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        
        hopperSubsystem.stopHopper();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}