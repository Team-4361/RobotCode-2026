package frc.robot.subsystems;

/* Imports */
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ClimberSubsystem extends Command {
    private SparkMax linearActuator;
    private RelativeEncoder linearActuatorPos;


public ClimberSubsystem() 
{
    /* Declares Sparkmax and Position */
    //linearActuator = new SparkMax();
    linearActuatorPos.setPosition(0);
}

public void periodic() {}

public void moveDown() {

}

public void moveUp() {
 
}

public void stopLinearActuator() {
 linearActuator.set(0);
}
}