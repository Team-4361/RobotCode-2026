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
    private RelativeEncoder linearActuatorPosID1;
    private RelativeEncoder linearActuatorPosID2;
    private RelativeEncoder linearActuatorEncoderID1;
    private RelativeEncoder linearActuatorEncoderID2;

    public ClimberSubsystem() 
    {
        /* Declares Sparkmax and Position */
        linearActuator = new SparkMax(Constants.climberConstants.RSPARKMAX_ID, MotorType.kBrushless);
        linearActuator = new SparkMax(Constants.climberConstants.LSPARKMAX_ID, MotorType.kBrushless);
        linearActuatorEncoderID1 = linearActuatorEncoderID1.getEncoder(); //needs to be fixed a bit
        linearActuatorEncoderID2 = linearActuatorEncoderID2.getEncoder(); //needs to be fixed a bit
        linearActuatorPosID1.setPosition(Constants.climberConstants.climberZero); //Sets the zero
        linearActuatorPosID2.setPosition(Constants.climberConstants.climberZero);
    }

    public void periodic() {}
    /* TODO: Test to make up and down is correct */

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