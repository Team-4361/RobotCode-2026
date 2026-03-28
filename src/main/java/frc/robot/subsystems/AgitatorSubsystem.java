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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AgitatorSubsystem extends SubsystemBase
{
        private final SparkFlex sparkFlex;
        private final RelativeEncoder encoder;


    public AgitatorSubsystem()
    {
        sparkFlex = new SparkFlex(Constants.AgitatorConstants.agitatorNeoID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.smartCurrentLimit(40);
        encoder = sparkFlex.getEncoder();
        //SparkClosedLoopController closedLoopController = sparkFlex.getClosedLoopController();
         SmartDashboard.putNumber("Agitator Speed", Constants.AgitatorConstants.vortexSpeed);
    }

    public void changeAgitatorSpeed (double vortexSpeed) {
        sparkFlex.set(vortexSpeed);
    }
    public void stopAgitator () {
        sparkFlex.set(0);
    }

    // public void setCServoSpeed (double servoSpeed) //clockwise
    // {
    //     sparkMax.set(servoSpeed);
    // }

    // public void setCCServoSpeed (double servoSpeed) //counterclockwise
    // {
    //     sparkMax.set(-servoSpeed);
    // }

    // public void stopServo()
    // {
    //     sparkMax.set(0);
    // }
    
 }