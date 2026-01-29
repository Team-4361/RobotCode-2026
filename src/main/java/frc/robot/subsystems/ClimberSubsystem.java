package frc.robot.subsystems;

/* Imports */
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;

public class ClimberSubsystem extends Command {
    private SparkMax linearActuator;
    private RelativeEncoder linearActuatorPos;
    private RelativeEncoder linearActuatorEncoder;

    public ClimberSubsystem() 
    {
        /* Declares Sparkmax and Position */
        linearActuator = new SparkMax(Constants.climberConstants.SPARKMAX_ID, MotorType.kBrushless);
        //linearActuatorEncoder = linearActuatorMotor.getEncoder();
        linearActuatorPos.setPosition(Constants.climberConstants.climberZero); //Sets the zero
    }

    public void periodic() {}

    //Moves the linear actuator up (negative is down)
    public void moveDown() {
        linearActuator.set(-Constants.climberConstants.climberSpeed);
    }

    //Moves the linear actuator up (positive is up)
    public void moveUp() {
        linearActuator.set(Constants.climberConstants.climberSpeed);
    }

    //Stops the Linear Actuator
    public void stopLinearActuator() { 
        linearActuator.set(0);
    }
}