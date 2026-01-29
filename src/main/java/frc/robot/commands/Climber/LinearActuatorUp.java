package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
public class LinearActuatorUp extends Command {
        public final ClimberSubsystem linearActuator;
    public LinearActuatorUp(ClimberSubsystem subsystem) {
        this.linearActuator = subsystem;
    }


    public void initialize(){
        linearActuator.moveUp();;
    }


    public void execute(){
        linearActuator.moveUp();
    }

    public void end() {
        linearActuator.stopLinearActuator();
    }

    @Override
    public void end(boolean interrupted) {
        linearActuator.stopLinearActuator();
    }


    public boolean isFinished()
    {
        return true; 
    }
}