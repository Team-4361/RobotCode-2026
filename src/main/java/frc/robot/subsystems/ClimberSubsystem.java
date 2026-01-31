package frc.robot.subsystems;

/* Imports */
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
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
    private PIDController linearActuatorPID;

    private double targetPosition = 0.0;

    public ClimberSubsystem() 
    {
        /* Declares Sparkmax and Position */
        linearActuator = new SparkMax(Constants.climberConstants.RSPARKMAX_ID, MotorType.kBrushless);
        linearActuator = new SparkMax(Constants.climberConstants.LSPARKMAX_ID, MotorType.kBrushless);
        //linearActuatorEncoderID1 = linearActuatorEncoderID1.getEncoder(); //getEncoder doesn't exist yet, will be implemented later.
        //linearActuatorEncoderID2 = linearActuatorEncoderID2.getEncoder(); 
        linearActuatorPosID1.setPosition(Constants.climberConstants.climberZero); //Sets the zero
        linearActuatorPosID2.setPosition(Constants.climberConstants.climberZero);

        double currentPos = linearActuatorPosID1.getPosition(); 
        double pidOutput = linearActuatorPID.calculate(currentPos, targetPosition);
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        linearActuator.set(pidOutput);
        linearActuatorPID = new PIDController(Constants.climberConstants.kP, Constants.climberConstants.kI, Constants.climberConstants.kD);   
    }

    public void periodic() {}

    //Moves the linear actuator up (postitive is down)
    public void moveDown() {
        linearActuator.set(Constants.climberConstants.climberSpeed);
    }

    //Moves the linear actuator up (negative is up)
    public void moveUp() {
        linearActuator.set(-Constants.climberConstants.climberSpeed); 
    }

    //Stops the Linear Actuator
    public void stopLinearActuator() { 
        linearActuator.set(0);
    }
}