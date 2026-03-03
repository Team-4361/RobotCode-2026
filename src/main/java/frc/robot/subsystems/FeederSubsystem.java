package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class FeederSubsystem extends SubsystemBase
{
    private final SparkFlex sparkFlex;
    private final RelativeEncoder encoder;

    public FeederSubsystem()
    {
       sparkFlex = new SparkFlex(17, MotorType.kBrushless);
         SparkFlexConfig config = new SparkFlexConfig();
            sparkFlex.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        config.idleMode(IdleMode.kCoast);
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = sparkFlex.getEncoder();
        
    }
    public void SetMotorSpeed(Double speed) 
    {
        sparkFlex.set(speed);
    }
    public void stop()
    {
        sparkFlex.stopMotor();
    }

    // Command to run the motor at a specified speed
    public Command runMotorCommand(double speed) {
        return this.runOnce(
            () -> SetMotorSpeed(speed));
    }
    
    // Command to stop the motor
    public Command stopMotorCommand() {
        return this.runOnce(() -> stop());
    }
    
    public Command runMotorButBetter(double speed) {
    return this.runEnd(
        () -> sparkFlex.set(speed),
        () -> sparkFlex.set(0)
    );
}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add any telemetry or monitoring here if needed
        SmartDashboard.putNumber("Feeder RPM", encoder.getVelocity());

    }



}