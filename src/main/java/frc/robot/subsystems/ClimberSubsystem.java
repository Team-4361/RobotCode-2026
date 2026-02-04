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

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax linearActuator;
    private RelativeEncoder linearActuatorPos;
    private RelativeEncoder linearActuatorEncoder;
    private RelativeEncoder winchEncoder;
    private PIDController linearActuatorPID;
    private SparkMax winchMotor;

    private double targetPosition = 0.0;

    public ClimberSubsystem() 
    {
        /* Declares Sparkmax and Position */
        winchEncoder = winchMotor.getEncoder();
        linearActuator = new SparkMax(Constants.climberConstants.RSPARKMAX_ID, MotorType.kBrushless); 
        linearActuatorEncoder = linearActuator.getEncoder();
        //linearActuatorPos = linearActuator.getRelativeEncoder(); //Maybe later if this is implemented
        linearActuatorPos.setPosition(Constants.climberConstants.climberZero); //Sets the zero
        winchEncoder.setPosition(0);

        double currentPos = linearActuatorPos.getPosition(); 
        double pidOutput = linearActuatorPID.calculate(currentPos, targetPosition);
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        linearActuator.set(pidOutput);
        linearActuatorPID = new PIDController(Constants.climberConstants.kP, Constants.climberConstants.kI, Constants.climberConstants.kD);   
    }

    public void periodic() {}

    //Moves the linear actuator up (postitive is down)
    public void moveLinearActuatorDown() {
        linearActuator.set(Constants.climberConstants.climberSpeed);
    }

    //Moves the linear actuator up (negative is up)
    public void moveLinearActuatorUp() {
        linearActuator.set(-Constants.climberConstants.climberSpeed); 
    }

    //Stops the Linear Actuator
    public void stopLinearActuator() { 
        linearActuator.set(0);
    }

     public void winchMoveDown() {
        winchMotor.set(-Constants.climberConstants.winchSpeed);
    }


    public void winchMoveUp() {
        winchMotor.set(Constants.climberConstants.winchSpeed);
    }


    public void stopWinch() {
        winchMotor.set(0);
    }
}