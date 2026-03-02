// package frc.robot.subsystems;

// /* Imports */


// import frc.robot.Constants;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;

// public class ClimberSubsystem extends SubsystemBase {
//     private SparkMax rightLinearActuator;
//     private SparkMax leftLinearActuator;
//     private RelativeEncoder rightLAencoder;
//     private RelativeEncoder leftLAencoder;
//     private SparkMax winchMotor;
    
//     // PID variables
//     private double integral = 0.0;
//     private double previousError = 0.0;
//     private double targetPosition = 0.0;
//     private double rightCurrentPos = leftLAencoder.getPosition();
//     private double leftCurrentPos = rightLAencoder.getPosition();
//     private double aveCurretPos = (rightCurrentPos + leftCurrentPos) / 2; 
//     private double error = targetPosition - aveCurretPos;
//     private double pidOutput;



//    public ClimberSubsystem() 
//      {
//         /* Declares Sparkmax and Position */ 
//         winchMotor = new SparkMax(0, null);
//         rightLinearActuator = new SparkMax(Constants.climberConstants.RSPARKMAX_ID, MotorType.kBrushless); 
//         leftLinearActuator = new SparkMax(Constants.climberConstants.LSPARKMAX_ID, MotorType.kBrushless); 
//         leftLAencoder = leftLinearActuator.getEncoder();
//         rightLAencoder = rightLinearActuator.getEncoder();
        
//         //linearActuatorPos.setPosition(Constants.climberConstants.climberZero); idk if we need this?  
//     }

//     public void periodic() 
//     {
//         // Calculate the integral and derivative
//         integral += error * 0.02; // Assuming teleopPeriodic runs at ~50 Hz (20 ms loop time)
//         double derivative = (error - previousError) / 0.02;

//         // Calculate PID output
//         pidOutput = (Constants.climberConstants.kP * error) + (Constants.climberConstants.kI * integral) + (Constants.climberConstants.kD * derivative);
    
//         // Limit the PID output to the motor speed range
//         pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput)); // Limit between -1.0 and 1.0

//         //Allows you to see pid Output and the other values
//         SmartDashboard.putNumber("Target Position: ", targetPosition);
//         SmartDashboard.putNumber("Current Position: ", aveCurretPos); 
//         SmartDashboard.putNumber("PID Output", pidOutput); 
//         SmartDashboard.putNumber("kP", Constants.climberConstants.kP); 
//         SmartDashboard.putNumber("kI", Constants.climberConstants.kI);
//         SmartDashboard.putNumber("kD", Constants.climberConstants.kD);


//     }


//     //Moves the linear actuator up (postitive is down)
//     public void moveLinearActuatorDown() {
//         rightLinearActuator.set(Constants.climberConstants.climberSpeed);
//         leftLinearActuator.set(Constants.climberConstants.climberSpeed);
//     }

//     //Moves the linear actuator up (negative is up)
//     public void moveLinearActuatorUp() {
//         rightLinearActuator.set(-Constants.climberConstants.climberSpeed); 
//         leftLinearActuator.set(-Constants.climberConstants.climberSpeed); 
//     }

//     //Stops the Linear Actuator
//     public void stopLinearActuator() { 
//         rightLinearActuator.set(0);
//         leftLinearActuator.set(0);
//     }

//      public void winchMoveDown() {
//         winchMotor.set(-Constants.climberConstants.winchSpeed);
//     }


//     public void winchMoveUp() {
//         winchMotor.set(Constants.climberConstants.winchSpeed);
//     }


//     public void stopWinch() 
//     {
//         winchMotor.set(0);
//     }




    
//     public void moveLinearActuatorPos(double targetPosition)
//     {
//         // Set the motor output
//         if (Math.abs(error) > Constants.climberConstants.POSITION_TOLERANCE) {
//             rightLinearActuator.set(pidOutput);
//             leftLinearActuator.set(pidOutput);
//         } else {
//             stopLinearActuator(); // Stop the motor if within tolerance
//         }
//         // Update the previous error
//         previousError = error;
    
//     }

// }