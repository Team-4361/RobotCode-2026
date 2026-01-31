package frc.robot.commands.test;
import frc.robot.subsystems.swerveDrive.MotorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
public class MotorCommand extends Command {
private final MotorSubsystem motor;
private double currentAngle;
private double targetAngle;


    public MotorCommand(MotorSubsystem subsystem) {
        this.motor = subsystem;

        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(motor);
    }

    @Override
    public void initialize()
    {
            currentAngle = motor.getCurrentAngle(); //gets the current angle
            motor.forwardMotorAngle();
    }
    @Override
    public void execute() 
    {
        motor.forwardMotorAngle(); //rotates the motor forwards

    }

    @Override
    public void end(boolean interrupted)
    {
        motor.stopMotor(); //stops if interrupted
    }

    @Override
    public boolean isFinished()
    {
        return currentAngle == motor.getTargetAngle(); //checks to see if it reached its position
    }

}