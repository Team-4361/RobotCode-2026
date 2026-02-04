package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class LinearActuatorUpID1 extends Command{
    public final ClimberSubsystem linearActuator;
    public LinearActuatorUpID1(ClimberSubsystem subsystem) {
        this.linearActuator = subsystem;

    }

@Override
    public void initialize(){
        linearActuator.moveDown();;
    }


    public void execute(){
        linearActuator.moveDown();
    }

    public void end() {
        linearActuator.stopLinearActuator();
    }


    public boolean isFinished()
    {
        return true; 
    }
}