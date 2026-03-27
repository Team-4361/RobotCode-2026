package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // ─── Hardware ────────────────────────────────────────────────────────────────

    /** Roller motor – Kraken X60 (TalonFX) */
    private final TalonFX rollerMotor;
    private final DutyCycleOut rollerOutput = new DutyCycleOut(0.0);

    /** Pivot motor – NEO (SparkMax, duty-cycle output only — no onboard PID) */
    private final SparkMax pivotMotor;

    /**
     * Absolute encoder plugged into the SparkMax ENCODER port.
     * We read it here and feed it into our own WPILib PIDController.
     * The SparkMax onboard PID is never used.
     */
    private final SparkAbsoluteEncoder absoluteEncoder;

    // ─── PID───────────────────────────────────────

    private final PIDController pivotPID;

    // ─── Setpoints (degrees) ─────────────────────────────────────────────────────

    /** Intake deployed – roller down, attached to the extendable hopper. */
    public static final double PIVOT_DOWN_DEGREES = 90.0; 

    /** Intake stowed. */
    public static final double PIVOT_UP_DEGREES   = 0.0;    

    // ─── Pivot output clamp ───────────────────────────────────────────────────────

    private static final double PIVOT_MAX_OUTPUT = 0.4;    

    // ─── Roller speed ────────────────────────────────────────────────────────────

    private static final double INTAKE_SPEED = 0.8;      

    // ─── Internal state ──────────────────────────────────────────────────────────

    private boolean pivotPIDEnabled = false;

    // ─── Constructor ─────────────────────────────────────────────────────────────

    /**
     * @param rollerMotorID  CAN ID of the Kraken (TalonFX) roller motor
     * @param pivotMotorID   CAN ID of the NEO pivot SparkMax
     */
    public IntakeSubsystem(int rollerMotorID, int pivotMotorID) {

        // Roller – Kraken X60
        rollerMotor = new TalonFX(rollerMotorID);

        // Pivot – NEO via SparkMax, brake mode so it holds when output is zero
        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Read the absolute encoder through the SparkMax API
        absoluteEncoder = pivotMotor.getAbsoluteEncoder();

        
        pivotPID = new PIDController(0.01, 0.0, 0.0);
        pivotPID.setTolerance(1.5); // degrees
    }

    // ─── Periodic ────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        if (pivotPIDEnabled) {
            double measurement = getAngleDegrees();
            double output      = pivotPID.calculate(measurement);

            // Clamp so we don't slam the mechanism
            output = Math.max(-PIVOT_MAX_OUTPUT, Math.min(PIVOT_MAX_OUTPUT, output));
            pivotMotor.set(output);
        }
    }

    // ─── Hardware helpers ────────────────────────────────────────────────────────

    /**
     * Returns the current pivot angle in degrees.
     * SparkAbsoluteEncoder returns [0, 1) turns by default — multiply by 360
     * to convert to degrees. If you've set a position conversion factor on the
     * SparkMax config, remove the * 360 and it will already be in degrees.
     */
    public double getAngleDegrees() {
        return absoluteEncoder.getPosition() * 360.0;
    }

    private void setPivotSetpoint(double degrees) {
        pivotPID.setSetpoint(degrees);
        pivotPIDEnabled = true;
    }

    private void stopPivot() {
        pivotPIDEnabled = false;
        pivotMotor.stopMotor();
    }

    private void runRoller(double speed) {
        rollerMotor.setControl(rollerOutput.withOutput(speed));
    }

    private void stopRoller() {
        rollerMotor.stopMotor();
    }

    // ─── Commands ────────────────────────────────────────────────────────────────

    /** Pivots the intake down and holds it there via PID. */
    public Command pivotDownCommand() {
        return this.runOnce(() -> setPivotSetpoint(PIVOT_DOWN_DEGREES));
    }

    /** Pivots the intake up (stow) and holds it there via PID. */
    public Command pivotUpCommand() {
        return this.runOnce(() -> setPivotSetpoint(PIVOT_UP_DEGREES));
    }

    /**
     * Runs the roller.
     * It keeps spinning until stopIntakeCommand() is called.
     */
    public Command runIntakeCommand() {
        return this.runOnce(() -> runRoller(INTAKE_SPEED));
    }

    /** Stops the roller immediately. */
    public Command stopIntakeCommand() {
        return this.runOnce(() -> stopRoller());
    }
}