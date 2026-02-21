package frc.robot.commands.Hopper;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class HopperCommand extends Command {
    private final HopperSubsystem hopperSubsystem;

    public HopperCommand(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
          hopperSubsystem.feed(Constants.HopperConstants.FEED_SPEED);
    }

    @Override
    public void execute() {
       
        hopperSubsystem.feed(Constants.HopperConstants.FEED_SPEED);
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