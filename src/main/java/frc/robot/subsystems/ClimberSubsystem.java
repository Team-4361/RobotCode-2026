/*
package frc.robot.subsystems;

Imports */ /*
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;

public class ClimberSubsystem extends Command {
    private SparkMax linearActuator1;
    private SparkMax linearActuator2;
    private RelativeEncoder linearActuatorPos;
    private RelativeEncoder linearActuatorPos2;
    private RelativeEncoder linearActuatorEncoder1;
    private RelativeEncoder linearActuatorEncoder2;

    public ClimberSubsystem() 
    {
        /* Declares Sparkmax and Position
        linearActuator1 = new SparkMax(Constants.climberConstants.SPARKMAX_ID, MotorType.kBrushless);
        linearActuatorEncoder1 = linearActuator1.getEncoder();
        linearActuatorEncoder2 = linearActuator2.getEncoder();
        linearActuatorPos.setPosition(Constants.climberConstants.climberZero1); //Sets the zero
        linearActuatorPos.setPosition(Constants.climberConstants.climberZero2); //Sets the zero
    }

    public void periodic() {}

    //Moves the linear actuator up (negative is down)
    public void moveDown1() {
        linearActuator1.set(-Constants.climberConstants.climberSpeed1);
    }

    public void moveDown2() {
        linearActuator2.set(-Constants.climberConstants.climberSpeed2);
    }

    //Moves the linear actuator up (positive is up)
    public void moveUp1() {
        linearActuator1.set(Constants.climberConstants.climberSpeed1);
    }

    public void moveUp2() {
        linearActuator2.set(Constants.climberConstants.climberSpeed2);
    }

    //Stops the Linear Actuator
    public void stopLinearActuator1() { 
        linearActuator1.set(0);
    }
    public void stopLinearActuator2() { 
        linearActuator1.set(0);
    }
}
*/ //idk if this is linked to my code just playing it safe - Jordan