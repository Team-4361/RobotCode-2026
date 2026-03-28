package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // ─── Hardware ────────────────────────────────────────────────────────────────

    /** Roller motor – Kraken X60 (TalonFX) */
    private final TalonFX rollerMotor;
    private final DutyCycleOut rollerOutput = new DutyCycleOut(0.0);

    /**
     * Servo that acts as a latch/pin mechanism.
     * When released, a pre-loaded spring deploys the intake down.
     * When engaged, the servo re-latches the intake in the stowed position.
     *
     * Wired to a PWM port on the RoboRIO.
     */
    private final Servo latchServo;

    // ─── Servo positions ─────────────────────────────────────────────────────────

    /**
     * Servo angle (degrees) that holds the latch engaged — intake stowed.
     * Adjust this value to match your physical latch geometry.
     */
    private static final double SERVO_LATCHED_ANGLE   = 0.0;

    /**
     * Servo angle (degrees) that releases the latch — spring deploys intake.
     * Adjust this value to match your physical latch geometry.
     */
    private static final double SERVO_RELEASED_ANGLE  = 90.0;

    // ─── Roller speed ────────────────────────────────────────────────────────────

    private static final double INTAKE_SPEED = 0.8;

    // ─── Internal state ──────────────────────────────────────────────────────────

    /** Tracks whether the latch is currently engaged (intake stowed). */
    private boolean isLatched = true;

    // ─── Constructor ─────────────────────────────────────────────────────────────

    /**
     * @param rollerMotorID  CAN ID of the Kraken (TalonFX) roller motor
     * @param servoPWMPort   PWM port on the RoboRIO the latch servo is plugged into
     */
    public IntakeSubsystem(int rollerMotorID, int servoPWMPort) {

        // Roller – Kraken X60
        rollerMotor = new TalonFX(rollerMotorID);

        // Latch servo – controls the spring-loaded intake pivot
        latchServo = new Servo(servoPWMPort);

        // Start in the stowed/latched position
        engageLatch();
    }

    // ─── Periodic ────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // No continuous control needed — the servo holds its position
        // and the spring handles the pivot motion passively.
    }

    // ─── Hardware helpers ────────────────────────────────────────────────────────

    /**
     * Moves the servo to the latched position, re-engaging the latch pin
     * to hold the intake in the stowed (up) position.
     * Call this only when the intake has been physically pushed back up
     * and is ready to be latched.
     */
    private void engageLatch() {
        latchServo.setAngle(SERVO_LATCHED_ANGLE);
        isLatched = true;
    }

    /**
     * Moves the servo to the released position, freeing the latch pin
     * so the spring can deploy the intake downward.
     */
    private void releaseLatch() {
        latchServo.setAngle(SERVO_RELEASED_ANGLE);
        isLatched = false;
    }

    private void runRoller(double speed) {
        rollerMotor.setControl(rollerOutput.withOutput(speed));
    }

    private void stopRoller() {
        rollerMotor.stopMotor();
    }

    // ─── State accessors ─────────────────────────────────────────────────────────

    /** Returns true if the latch servo is engaged and the intake is stowed. */
    public boolean isLatched() {
        return isLatched;
    }

    // ─── Commands ────────────────────────────────────────────────────────────────

    /**
     * Releases the latch servo so the spring deploys the intake downward.
     * The intake will stay down until stowIntakeCommand() is called
     * (which assumes your robot mechanically re-cocks the spring first).
     */
    public Command deployIntakeCommand() {
        return this.runOnce(this::releaseLatch);
    }

    /**
     * Re-engages the latch servo to lock the intake in the stowed position.
     * Only call this after the intake has been physically returned to the
     * stowed position (e.g., by a separate re-cocking mechanism or manually).
     */
    public Command stowIntakeCommand() {
        return this.runOnce(this::engageLatch);
    }

    /**
     * Runs the roller motor.
     * The roller keeps spinning until stopIntakeCommand() is called.
     */
    public Command runIntakeCommand() {
        return this.runOnce(() -> runRoller(INTAKE_SPEED));
    }

    /** Stops the roller motor immediately. */
    public Command stopIntakeCommand() {
        return this.runOnce(this::stopRoller);
    }

    /**
     * Convenience command: releases the latch and immediately starts the roller.
     * Useful for binding to a single button press.
     */
    public Command deployAndRunCommand() {
        return this.runOnce(() -> {
            releaseLatch();
            runRoller(INTAKE_SPEED);
        });
    }
}