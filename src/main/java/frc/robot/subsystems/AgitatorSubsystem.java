package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase
{
        private final SparkFlex sparkFlex;


    public AgitatorSubsystem()
    {
        sparkFlex = new SparkFlex(Constants.AgitatorConstants.agitatorNeoID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.smartCurrentLimit(40);
         SmartDashboard.putNumber("Agitator Speed", Constants.AgitatorConstants.vortexSpeed);
    }

    public void changeAgitatorSpeed (double vortexSpeed) {
        sparkFlex.set(vortexSpeed);
    }
    public void stopAgitator () {
        sparkFlex.set(0);
    }

        // Command to run the motor at a specified speed
    public Command runMotorCommand(double speed) {
        return this.runOnce(
            () -> changeAgitatorSpeed(speed));
    }
    
    // Command to stop the motor
    public Command stopMotorCommand() {
        return this.runOnce(() -> stopAgitator());
    }
    


}