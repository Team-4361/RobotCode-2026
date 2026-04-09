package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {

    private static final int    DEVICE_ID     = 15;       
    private static final double INTAKE_SPEED  =  0.8;
    private static final double OUTTAKE_SPEED = -0.8;

    private final TalonFX      kraken;
    private final DutyCycleOut dutyCycleRequest;

    public IntakeRollerSubsystem() {
        kraken = new TalonFX(DEVICE_ID);
        dutyCycleRequest = new DutyCycleOut(0.0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // rollers coast on stop
        kraken.getConfigurator().apply(config);
    }


    public void intake() {
        kraken.setControl(dutyCycleRequest.withOutput(INTAKE_SPEED));
    }

    public void outtake() {
        kraken.setControl(dutyCycleRequest.withOutput(OUTTAKE_SPEED));
    }

    public void stop() {
        kraken.stopMotor();
    }


    /** Runs rollers inward while held, stops on release. */
    public Command intakeCommand() {
        return this.startEnd(this::intake, this::stop);
    }

    /** Runs rollers outward while held, stops on release. */
    public Command outtakeCommand() {
        return this.startEnd(this::outtake, this::stop);
    }

    /** Manual override at a custom speed. */
    public Command manualCommand(double speed) {
        return this.startEnd(
            () -> kraken.setControl(dutyCycleRequest.withOutput(speed)),
            this::stop
        );
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeRoller/Output",  kraken.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("IntakeRoller/Current", kraken.getSupplyCurrent().getValueAsDouble());
    }
}