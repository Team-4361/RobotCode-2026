package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterCurveFitter;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;

/**
 * Manual distance-based shooter calibration.
 *
 * Workflow per data point:
 *  1. Driver moves robot to a position.
 *  2. Operator enters the measured distance via SmartDashboard
 *     ("Calibration/Input Distance (m)").
 *  3. Operator nudges RPM with bumpers until the shot looks right.
 *  4. Press A → scored (logs the point).  Press B → missed (discard, retry).
 *  5. Repeat from any distance.  Press Back to finish and print the fit.
 *
 * Bind example:
 * <pre>
 *   ShooterCalibrationCommand cal = new ShooterCalibrationCommand(
 *       shooter,
 *       () -> operator.getAButton(),
 *       () -> operator.getBButton(),
 *       () -> operator.getLeftBumper(),   // RPM down
 *       () -> operator.getRightBumper(),  // RPM up
 *       () -> operator.getBackButton()    // finish & print fit
 *   );
 *   new JoystickButton(operator, XboxController.Button.kStart.value)
 *       .toggleOnTrue(cal);
 * </pre>
 */
public class ShooterCalibrationCommand extends Command {

    private enum State { WAITING_FOR_LOCK, NUDGING, DONE }

    // ── SmartDashboard keys ───────────────────────────────────────────────────
    private static final String KEY_STATE         = "Calibration/State";
    private static final String KEY_CURRENT_RPM   = "Calibration/Current RPM";
    private static final String KEY_INPUT_DIST     = "Calibration/Input Distance (m)";
    private static final String KEY_POINTS_LOGGED  = "Calibration/Points Logged";
    private static final String KEY_LAST_RESULT    = "Calibration/Last Result";
    private static final String KEY_FIT_OUTPUT     = "Calibration/Fit Output";

    private static final double RPM_NUDGE_AMOUNT   = 25.0;   // RPM per button press
    private static final double RPM_START          = 2000.0; // default starting RPM
    private static final double RPM_MIN            = 500.0;
    private static final double RPM_MAX            = 6000.0;

    // ── Hardware / suppliers ──────────────────────────────────────────────────
    private final ShooterSubsystem shooter;
    private final BooleanSupplier  scoredButton;   // A
    private final BooleanSupplier  missedButton;   // B
    private final BooleanSupplier  rpmDownButton;  // Left bumper
    private final BooleanSupplier  rpmUpButton;    // Right bumper
    private final BooleanSupplier  finishButton;   // Back

    // ── Per-session state ─────────────────────────────────────────────────────
    private State  state;
    private double currentRPM;
    private double lockedDistance;

    // Previous button states for edge detection
    private boolean prevScored, prevMissed, prevUp, prevDown, prevFinish;

    // ── Dataset ───────────────────────────────────────────────────────────────
    private final List<Double> distancePoints = new ArrayList<>();
    private final List<Double> rpmPoints      = new ArrayList<>();

    // ── Constructor ───────────────────────────────────────────────────────────
    public ShooterCalibrationCommand(
            ShooterSubsystem shooter,
            BooleanSupplier  scoredButton,
            BooleanSupplier  missedButton,
            BooleanSupplier  rpmDownButton,
            BooleanSupplier  rpmUpButton,
            BooleanSupplier  finishButton) {

        this.shooter       = shooter;
        this.scoredButton  = scoredButton;
        this.missedButton  = missedButton;
        this.rpmDownButton = rpmDownButton;
        this.rpmUpButton   = rpmUpButton;
        this.finishButton  = finishButton;
        addRequirements(shooter);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        state       = State.WAITING_FOR_LOCK;
        currentRPM  = RPM_START;
        distancePoints.clear();
        rpmPoints.clear();

        // Seed the SmartDashboard input field so operators see it immediately
        SmartDashboard.putNumber(KEY_INPUT_DIST,    0.0);
        SmartDashboard.putNumber(KEY_CURRENT_RPM,   currentRPM);
        SmartDashboard.putNumber(KEY_POINTS_LOGGED, 0);
        SmartDashboard.putString(KEY_STATE,
                "Enter distance on dashboard, then press START to lock");
        SmartDashboard.putString(KEY_LAST_RESULT,   "---");
        SmartDashboard.putString(KEY_FIT_OUTPUT,    "---");
    }

    @Override
    public void execute() {
        boolean scored  = scoredButton.getAsBoolean();
        boolean missed  = missedButton.getAsBoolean();
        boolean up      = rpmUpButton.getAsBoolean();
        boolean down    = rpmDownButton.getAsBoolean();
        boolean finish  = finishButton.getAsBoolean();

        switch (state) {

            // ── Step 1: operator sets distance on dashboard, presses Y to lock ──
            case WAITING_FOR_LOCK:
                // Reuse scoredButton (A) as the "lock this distance" trigger
                if (scored && !prevScored) {
                    lockedDistance = SmartDashboard.getNumber(KEY_INPUT_DIST, 0.0);
                    currentRPM     = RPM_START;
                    state          = State.NUDGING;
                    shooter.setVelocity(RPM.of(currentRPM)).schedule();
                    SmartDashboard.putString(KEY_STATE,
                            String.format("Locked %.2f m — Bumpers nudge RPM. A=Scored, B=Missed",
                                    lockedDistance));
                }
                break;

            // ── Step 2: bumpers nudge RPM until shot is good, A/B to confirm ────
            case NUDGING:
                // Nudge RPM on rising edge only (prevents holding = spam)
                if (up && !prevUp) {
                    currentRPM = Math.min(currentRPM + RPM_NUDGE_AMOUNT, RPM_MAX);
                    shooter.setVelocity(RPM.of(currentRPM)).schedule();
                }
                if (down && !prevDown) {
                    currentRPM = Math.max(currentRPM - RPM_NUDGE_AMOUNT, RPM_MIN);
                    shooter.setVelocity(RPM.of(currentRPM)).schedule();
                }
                SmartDashboard.putNumber(KEY_CURRENT_RPM, currentRPM);

                // A = scored → log and reset for next point
                if (scored && !prevScored) {
                    distancePoints.add(lockedDistance);
                    rpmPoints.add(currentRPM);
                    SmartDashboard.putNumber(KEY_POINTS_LOGGED, distancePoints.size());
                    SmartDashboard.putString(KEY_LAST_RESULT,
                            String.format("✓ LOGGED  %.2f m → %.0f RPM", lockedDistance, currentRPM));
                    // Go back to waiting for next distance lock
                    state = State.WAITING_FOR_LOCK;
                    SmartDashboard.putString(KEY_STATE,
                            "Update distance on dashboard, press A to lock next point");
                }

                // B = missed → stay in nudging, just note it
                if (missed && !prevMissed) {
                    SmartDashboard.putString(KEY_LAST_RESULT, "✗ MISSED — keep nudging");
                }
                break;

            case DONE:
                break;
        }

        // ── Finish: Back button → compute and display fit ─────────────────────
        if (finish && !prevFinish && distancePoints.size() >= 2) {
            String fitResult = ShooterCurveFitter.fitAndFormat(distancePoints, rpmPoints);
            SmartDashboard.putString(KEY_FIT_OUTPUT, fitResult);
            SmartDashboard.putString(KEY_STATE, "DONE — See Fit Output on dashboard");
            state = State.DONE;
        }

        // Save previous button states for next loop
        prevScored = scored;
        prevMissed = missed;
        prevUp     = up;
        prevDown   = down;
        prevFinish = finish;
    }

    @Override
    public boolean isFinished() { return state == State.DONE; }

    @Override
    public void end(boolean interrupted) {
        shooter.set(0).schedule();
        if (interrupted) SmartDashboard.putString(KEY_STATE, "INTERRUPTED");
    }
}