import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

package frc.robot.commands.Hopper;


public class HopperCommand extends Command {
    private final HopperSubsystem hopperSubsystem;

    public HopperCommand(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
          hopperSubsystem.feed();
    }

    @Override
    public void execute() {
       
        hopperSubsystem.feed();
    }

    @Override
    public void end(boolean interrupted) {
        
        hopperSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}