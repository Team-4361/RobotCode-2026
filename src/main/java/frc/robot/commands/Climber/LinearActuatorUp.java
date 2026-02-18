package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class LinearActuatorUp extends Command{
    public final ClimberSubsystem linearActuator;
    public LinearActuatorUp(ClimberSubsystem subsystem) {
        this.linearActuator = subsystem;

        addRequirements(linearActuator);
    }

@Override
    public void initialize(){
        linearActuator.moveLinearActuatorUp();;
    }


    public void execute(){
        linearActuator.moveLinearActuatorUp();
    }

    public void end() {
        linearActuator.stopLinearActuator();
    }


    public boolean isFinished()
    {
        return true; 
    }
}