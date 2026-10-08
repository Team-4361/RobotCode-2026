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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

public class AgitatorSubsystem extends SubsystemBase
{
        private final SparkFlex sparkFlex;
        

        private double lastCommandedSpeed = 0;

    public AgitatorSubsystem()
    {
        sparkFlex = new SparkFlex(Constants.AgitatorConstants.agitatorNeoID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);  
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
         SmartDashboard.putNumber("Agitator Speed", Constants.AgitatorConstants.vortexSpeed);
    }

private boolean isStalled() {
    return lastCommandedSpeed != 0
        && sparkFlex.getOutputCurrent() > 35
        && Math.abs(sparkFlex.getEncoder().getVelocity()) < 300;
}

    public void changeAgitatorSpeed (double vortexSpeed) {
        sparkFlex.set(vortexSpeed);
        lastCommandedSpeed = vortexSpeed;
    }
    public void stopAgitator () {
        sparkFlex.set(0);
        lastCommandedSpeed = 0;
    }

    
    public Command agitateCommand(double speed) {
        Trigger stalled = new Trigger(this::isStalled).debounce(0.4); // debounce covers spin-up
        return Commands.repeatingSequence(
            this.run(() -> changeAgitatorSpeed(speed)).until(stalled),
            this.run(() -> changeAgitatorSpeed(-speed)).withTimeout(0.25)
        ).finallyDo(this::stopAgitator);
    }


    @Override
    public void periodic() {
        Logger.recordOutput("Agitator/CommandedSpeed", lastCommandedSpeed);
        Logger.recordOutput("Agitator/Running", lastCommandedSpeed != 0);
        Logger.recordOutput("Agitator/OutputCurrentA", sparkFlex.getOutputCurrent());
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