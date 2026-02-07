package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends SubsystemBase
{
    private final SparkFlex sparkFlex;
    private final RelativeEncoder encoder;
    private double targetRPM = 69.11;
    private final double kP = 0.1;
    private final double kI = 0.001;
    private final double kD = 0.00;



    public IntakeSubsystem()
    {
       sparkFlex = new SparkFlex(Constants.IntakeConstants.INTAKENEO, MotorType.kBrushless);
         SparkFlexConfig config = new SparkFlexConfig();
           /* .closedLoop.pid(0.01, 0, 0.001); */
            sparkFlex.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        config.idleMode(IdleMode.kBrake);
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = sparkFlex.getEncoder();
        SparkClosedLoopController closedLoopController = sparkFlex.getClosedLoopController();

    }
    public void SetMotorSpeed(Double speed) 
    {
        sparkFlex.set(speed);
    }
    public void stop()
    {
        sparkFlex.stopMotor();
    }
    public void set()
    {
        //closedLoopController.setSetpoint(10, ControlType.kVelocity); // 10 RPM
    }
    
}