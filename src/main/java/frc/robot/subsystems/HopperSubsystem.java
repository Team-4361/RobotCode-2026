package frc.robot.commands.subsystems;

// I don't know if these are the right import
import frc.robot.Constants;
import com.revrobotics.SparkMax;
import com.revrobotics.SparkBase.IdleMode;
import com.revrobotics.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase //hello Jack and Chloe I know you guys are watching me
{
  private final SparkMax hopperMotor;

 // constants
private static final int HOPPER_ID = 1;
private static final double FEED_SPEED = 0.5;


  public HopperSubsystem()
  {
    hopperMotor = new SparkMax(HOPPER_ID, MotorType.kBrushless); // creates a new motor controller object for the hopper motor
    hopperMotor.setIdleMode(IdleMode.kCoast); //
   hopperMotor.setSmartCurrentLimit(40);  // limit to protect motor
  }


  public void feed()
  {
    hopperMotor.set(FEED_SPEED);
  }

 // Reverses the hopper motor (for unjamming or something)
  public void reverseHopper()
  {
    hopperMotor.set(-FEED_SPEED);

  }

  // Stops 
  public void stopHopper()
  {
    hopperMotor.set(0);
  }

  
  public void hopperSpeed(double speed)
  {
    hopperMotor.set(speed);
  }

  public double getHopperSpeed()
  {
    return hopperMotor.get();
  }

  public double getMotorCurrent()
  {
    return hopperMotor.getOutputCurrent();
  }

  @Override
  public void periodic()
     {

    }
}