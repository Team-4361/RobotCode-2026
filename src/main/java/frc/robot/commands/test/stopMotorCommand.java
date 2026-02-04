package frc.robot.commands.test;
import frc.robot.subsystems.swerveDrive.MotorSubsystem;
import frc.robot.commands.test.MotorCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class stopMotorCommand extends Command {
private final MotorSubsystem motor;
private final MotorCommand motor2;
private double currentAngle;
private double targetAngle;



    public stopMotorCommand(MotorSubsystem subsystem) {
        this.motor = subsystem;
        this.motor2 = new MotorCommand(subsystem);

        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(motor);
    }

    @Override
    public void initialize()
    {
    }
    @Override
    public void execute() 
    {
        
        motor.stopMotor(); //rotates the motor forwards
        CommandScheduler.getInstance().cancel(motor2);

    }

    @Override
    public void end(boolean interrupted)
    {
        motor.stopMotor(); //stops if interrupted
    }

    @Override
    public boolean isFinished()
    {
        return motor.atTarget();
    }

}