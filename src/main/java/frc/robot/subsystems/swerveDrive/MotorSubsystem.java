package frc.robot.subsystems.swerveDrive; //for testing a motor

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

public class MotorSubsystem extends SubsystemBase {
private final SparkMax motor;  //create the sparkmax motor object
private final RelativeEncoder encoder; // create the relative encoder that is attached to the motor, could change to alt 
private final PIDController pidController; //create the pidcontroller object, needs 3 values

private static final int MOTOR_ID = 6; // the id of the sparkmax that the motor is attached  
private static final int CPR = 42; // Encoder counts per revolution, was 2048 with the talon lea 42 for neo550
private static final double KP = 0.0666; // the P value of the PID controller, porportional 
private static final double KI = 0.00002; // the I value of the PID controller, intergral
private static final double KD = 0.0010; // the D value of te PID controller, derivative
private static final double MAX_POWER = 0.2; // the max power that the pidcontroller can set the motor to
//private final DCMotor bGearbox;
//private final Encoder e;

private double targetAngle1 = 0.0; // the desired encoder units that the motor needs to move to

public MotorSubsystem() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless); // initialtize the the motor with the id and with the brushless type 
    encoder = motor.getEncoder(); // initialtize the encoder object by using the getEncoder() method to interface with the physical encoder
    //if gearbox is used
    /*e.setDistancePerPulse(CPR);
    bGearbox = new DCMotor(MOTOR_GEAR_RATIO, MAX_POWER, KP, KI, KD, MOTOR_ID);*/

    //encoder.setDistancePerPulse(360.0 / (CPR * MOTOR_GEAR_RATIO));
    pidController = new PIDController(KP, KI, KD); //initialtize the pid controller with the related P, I, and D values
    pidController.setTolerance(0.5); // the unit tolerance in which the pidcontroller will stop moving the motor to the setpoint
}
public void forwardMotorAngle()
{
    targetAngle1 += 50; //add 50 encoder units to the target angle variable
}

public boolean atTarget()
{
    //if the encoder is currently at the target angle 
    return encoder.getPosition()>=targetAngle1;

}

public void backwardsMotorAngle()
{
    targetAngle1 -= 50;
}

public void fMotor() {
    //targetAngle1 += 5.0;
    motor.set(MAX_POWER); //sets the power for going forward
}

public void bMotor() {
    //targetAngle1 -= 5.0;
    motor.set(-MAX_POWER); //sets the power for going backwards
}
/* 
public void resetMotor() {
    //encoder.reset();
    targetAngle1 = 0.0;
}

public void zeroMotor() {
    targetAngle1 = 0.0;
}
*/
public void stopMotor() {
    motor.stopMotor(); //stops the Motor
}
public double getCurrentAngle(){
    return encoder.getPosition(); //gets the position of the Motor
}
public double getTargetAngle(){
    return targetAngle1; //gets a target angle
}
@Override
public void periodic() {
    double currentAngle = encoder.getPosition(); //gets the position of the Motor
    double pidOutput = pidController.calculate(currentAngle, targetAngle1); //tells the Motor its PID value

    pidOutput = Math.max(-1, Math.min(1, pidOutput)); //PID stuff

    pidOutput *= MAX_POWER;

    if (!pidController.atSetpoint()) {
        motor.set(pidOutput);
    } else {
        motor.set(0);
    }
    //SmartDashboard.putNumber("current angle", currentAngle);
    //SmartDashboard.putNumber("target angle", targetAngle1);
   // SmartDashboard.putNumber("pid output", pidOutput);
}
}