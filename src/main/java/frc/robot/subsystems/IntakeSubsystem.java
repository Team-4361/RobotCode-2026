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
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class IntakeSubsystem extends SubsystemBase
{
    private final SparkFlex sparkFlex;
    private final RelativeEncoder encoder;

    public IntakeSubsystem()
    {
<<<<<<< HEAD
        sparkFlex = new SparkFlex(0, null);
=======
        sparkFlex = new SparkFlex(Constants.IntakeConstants.INTAKENEO, MotorType.kBrushless);
>>>>>>> 5a7a6b1d4fdb72dfdf2223388de4290d63705c3f
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = sparkFlex.getEncoder();
<<<<<<< HEAD
=======
    }
    public void SetMotorSpeed(Double speed) 
    {
        sparkFlex.set(speed);
>>>>>>> 5a7a6b1d4fdb72dfdf2223388de4290d63705c3f
    }
}