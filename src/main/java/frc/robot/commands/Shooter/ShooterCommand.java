package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
public class ShooterCommand extends Command {
private final ShooterSubsystem shooter;
private final double indexSpeed;
private final double shooterSpeed;

    public ShooterCommand(ShooterSubsystem shooter, double indexSpeed, double shooterSpeed) {
        this.shooter = shooter;
        
        this.indexSpeed = indexSpeed;
        this.shooterSpeed = shooterSpeed;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(shooter);
    }

    @Override
    public void initialize()
    {   shooter.indexVortex(indexSpeed);
        shooter.shooterKraken(shooterSpeed);
    }
    @Override
    public void execute()
    {
        shooter.indexVortex(indexSpeed); //sets the indaexer at the given speed 
        shooter.shooterKraken(shooterSpeed); //set the shooter at the given speeed
    }

    @Override
    public void end(boolean interrupted)
    {
        shooter.stopShooter(); //if interupted, stops all
    }

    @Override
    public boolean isFinished()
    {
        return false; //idr how to implement this 
    }
}