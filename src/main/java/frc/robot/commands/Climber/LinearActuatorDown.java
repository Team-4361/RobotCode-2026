package frc.robot.commands.Climber;

import frc.robot.subsystems.ClimberSubsystem;

public class LinearActuatorDown {
    public final ClimberSubsystem linearActuator;
    public LinearActuatorDown(ClimberSubsystem subsystem) {
        this.linearActuator = subsystem;

    }


    public void initialize(){
        linearActuator.moveDown();;
    }


    public void execute(){
        linearActuator.moveDown();
    }

    public void end() {
        linearActuator.stopLinearActuator();
    }

    public void end(boolean interrupted) {
        linearActuator.stopLinearActuator();
    }


    public boolean isFinished()
    {
        return true; 
    }
}