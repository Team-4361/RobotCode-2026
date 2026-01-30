package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
public class ShooterComamand extends Command {
private final ShooterSubsystem shooter;
private final double indexSpeed;
private final double shooterSpeed;

    public ShooterComamand(ShooterSubsystem shooter, double indexSpeed, double shooterSpeed) {
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
        shooter.shooterKraken(shooterSpeed); //moves the elevator up
    }

    @Override
    public void end(boolean interrupted)
    {
        shooter.stopShooter(); //if interrupted, stops the elevator as an emergency
    }

    @Override
    public boolean isFinished()
    {
        return false; //idr how to implement this 
    }
}