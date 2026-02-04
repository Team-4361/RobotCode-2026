package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class LinearActuatorDown extends Command{
    public final ClimberSubsystem linearActuator;
    public LinearActuatorDown(ClimberSubsystem subsystem) {
        this.linearActuator = subsystem;

        addRequirements(linearActuator);
    }

@Override
    public void initialize(){
        linearActuator.moveLinearActuatorDown();
    }


    public void execute(){
        linearActuator.moveLinearActuatorDown();
    }

    public void end() {
        linearActuator.stopLinearActuator();
    }


    public boolean isFinished()
    {
        return true; 
    }
}