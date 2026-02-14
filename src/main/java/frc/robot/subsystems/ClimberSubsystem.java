package frc.robot.subsystems;

/* Imports */
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double integral = 0.0;
    private double previousError = 0.0;
    private double pidOutput = 0.0;

    public ClimberSubsystem() 
    {
        /* Declares Sparkmax and Position */
        winchEncoder = winchMotor.getEncoder();
        linearActuator = new SparkMax(Constants.climberConstants.RSPARKMAX_ID, MotorType.kBrushless); 
        linearActuatorEncoder = linearActuator.getEncoder();
        //linearActuatorPos = linearActuator.getRelativeEncoder(); //Maybe later if this is implemented
        linearActuatorPos.setPosition(0); //Sets the zero
        winchEncoder.setPosition(0);

        //PID
        //Declares the Controller
         PIDController pid = new PIDController(Constants.climberConstants.kP, Constants.climberConstants.kI, Constants.climberConstants.kD);


        double currentPos = linearActuatorPos.getPosition(); 
        pidOutput = linearActuatorPID.calculate(currentPos, targetPosition);
        linearActuator.set(pidOutput);
        linearActuatorPID = new PIDController(Constants.climberConstants.kP, Constants.climberConstants.kI, Constants.climberConstants.kD);   

        double currentPosition = linearActuatorPos.getPosition();
        double error = targetPosition - currentPosition;

        // Calculate the integral and derivative
        integral += error * 0.02; // Assuming teleopPeriodic runs at ~50 Hz (20 ms loop time)
        double derivative = (error - previousError) / 0.02;

        // Calculate PID output
        pidOutput = (Constants.climberConstants.kP * error) + (Constants.climberConstants.kI * integral) + (Constants.climberConstants.kD * derivative);
    
        // Limit the PID output to the motor speed range
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput)); // Limit between -1.0 and 1.0

        // Update the previous error
        previousError = error;

    }

    public void periodic() {}

    //Moves the linear actuator up (postitive is down)
    public void moveLinearActuatorDown() {
        linearActuator.set(pidOutput);
    }
    /*
     * Timer.delay(targetPosition) is intended to reach the distance in cm as fast as possible
     * It needs some testing to reach the correct position multiply the amount of time it takes to reach the spot
     * Unsure if this subsystem will work.
     */

    //Moves the linear actuator up (negative is up)
    public void moveLinearActuatorUp() {
        linearActuator.set(pidOutput);
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