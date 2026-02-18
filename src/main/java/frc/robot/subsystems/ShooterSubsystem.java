package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase
{
    private final TalonFX shooterKraken;
    private final SparkFlex indexVortex;
    //private MotionMagicVelocityVoltage talonController;
        public ShooterSubsystem()
        {
            shooterKraken = new TalonFX(0); //add to constants  
            shooterKraken.getConfigurator().apply(new TalonFXConfiguration());
            var currentLimitsConfigs =   new CurrentLimitsConfigs();
            shooterKraken.setNeutralMode(NeutralModeValue.Coast);
            currentLimitsConfigs.StatorCurrentLimit = 40; // add value to constants 
            currentLimitsConfigs.StatorCurrentLimitEnable = true;
            shooterKraken.getConfigurator().refresh(currentLimitsConfigs);
            shooterKraken.getConfigurator().apply(currentLimitsConfigs);
            //talonController = new MotionMagicVelocityVoltage(0);
        indexVortex = new SparkFlex(0, MotorType.kBrushless); 
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        indexVortex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    } 

    @Override
    public void periodic()
    {
        //SmartDashboard.putNumber()
    }

    public void shooterKraken(double speed)
    {
        shooterKraken.set(speed);
    }
    public void indexVortex(double speed)
    {
        indexVortex.set(speed);
    }
    public void stopShooter()
   {
        shooterKraken.stopMotor();
        
   }
   public void stopIndexer()
   {
        indexVortex.stopMotor();
   }
   public void stopMotors()
   {
        shooterKraken.stopMotor();
        indexVortex.stopMotor();
   }
   public double getRPM()
   {
         return shooterKraken.getVelocity().getValueAsDouble();
   }
    
}