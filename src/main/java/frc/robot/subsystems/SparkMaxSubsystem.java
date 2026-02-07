package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig;
    
    // Constructor - replace CAN_ID with your motor's actual CAN ID
    public SparkMaxSubsystem(int canId) {
        motor = new SparkMax(canId, MotorType.kBrushless);
        
        // Create configuration object
        motorConfig = new SparkMaxConfig();
        
        // Configure idle mode (brake or coast)
        motorConfig.idleMode(IdleMode.kBrake);
        
        // Apply configuration to the SPARK MAX
        // kResetSafeParameters resets to known state
        // kPersistParameters saves config through power cycles
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    // Method to set motor speed (-1.0 to 1.0)
    public void setSpeed(double speed) {
        motor.set(speed);
    }
    
    // Method to stop the motor
    public void stopMotor() {
        motor.stopMotor();
    }
    
    // Command to run the motor at a specified speed
    public Command runMotorCommand(double speed) {
        return this.runOnce(
            () -> setSpeed(speed));
    }
    
    // Command to stop the motor
    public Command stopMotorCommand() {
        return this.runOnce(() -> stopMotor());
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add any telemetry or monitoring here if needed
    }
}