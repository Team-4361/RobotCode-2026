package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase //hello Jack and Chloe I know you guys are watching me
{
  private final SparkMax hopperMotor;
  private final com.revrobotics.RelativeEncoder hopperEncoder;


 // constants
/*private static final int HOPPER_ID = 1;
private static final double FEED_SPEED = 0.5;*/


  public HopperSubsystem()
  {
    hopperMotor = new SparkMax(6, MotorType.kBrushless); // creates a new motor controller object for the hopper motor
    SparkMaxConfig config = new SparkMaxConfig();
    
    config.idleMode(IdleMode.kCoast); //
    hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   config.smartCurrentLimit(40);  // limit to protect motor
   hopperEncoder = hopperMotor.getEncoder();
    SmartDashboard.putNumber("Feed Speed:", Constants.HopperConstants.FEED_SPEED);
  }


  public void feed(double FEED_SPEED)
  {
    hopperMotor.set(FEED_SPEED);
  }

 // Reverses the hopper motor (for unjamming or something)
  public void reverseHopper(double FEED_SPEED)
  {
    hopperMotor.set(-FEED_SPEED);

  }

  // Stops 
  public void stopHopper()
  {
    hopperMotor.stopMotor();
  }


  public double getHopperSpeed()
  {
    return hopperMotor.get();
  }

  public double getMotorCurrent()
  {
    return hopperMotor.getOutputCurrent();
  }

    // Command to run the motor at a specified speed
    public Command runMotorCommand(double speed) {
        return this.runOnce(
            () -> feed(speed));
    }
    
    // Command to stop the motor
    public Command stopMotorCommand() {
        return this.runOnce(() -> stopHopper());
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add any telemetry or monitoring here if needed
                SmartDashboard.putNumber("Hopper RPM", hopperEncoder.getVelocity());
    }


}
