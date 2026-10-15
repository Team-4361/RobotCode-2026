package frc.robot.logics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Settings;
import frc.robot.subsystems.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

public class teleopController {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private CommandJoystick joystickL;
    private CommandJoystick joystickR;
    private final ShooterSubsystem shooter;

    // ── Filtered outputs ──────────────────────────────────────────────────────
    public double xV = 0;
    public double yV = 0;
    public double rV = 0;
    public boolean isOrbit = false;

    public SlewRateLimiter xfilter = new SlewRateLimiter(8);
    public SlewRateLimiter yfilter = new SlewRateLimiter(8);
    public SlewRateLimiter rfilter = new SlewRateLimiter(4);

    // ── Buttons ───────────────────────────────────────────────────────────────
    /** joystickL button that toggles hub-lock (snap + shoot-on-move) mode. */
    private static final int ORBIT_TOGGLE_BUTTON = 1;

    /** joystickR button (HELD) that X-locks the wheels. Change if it's taken. */
    private static final int WHEEL_LOCK_BUTTON = 4;

    // ── Hub-lock tuning ───────────────────────────────────────────────────────
    /** Max translation speed (m/s) while locked onto the hub. */
    private static final double LOCKED_MAX_SPEED_MPS = 2.0;

    private static final double HEADING_P = 6.0;
    private static final double HEADING_MAX_RAD_S = 7.0;

    /** Radial P gain / max speed used to drive to the typed distance in tune mode. */
    private static final double TUNE_SNAP_GAIN = 4.0;
    private static final double TUNE_SNAP_MAX_SPEED = 2.0;
    private static final double TUNE_AT_DISTANCE_TOL_M = 0.05;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean hubOrbitEnabled = false;
    private boolean orbitButtonWasPressed = false;
    private ShotCalculator.Solution lastSolution = null;

    /** Holds the shooter at the table RPM (or the tuned RPM in tune mode). */
    private final Command orbitRevShooterCommand;

    public teleopController(CommandJoystick joyL,
                            CommandJoystick joyR,
                            CommandXboxController xboxCommandJoystick,
                            ShooterSubsystem shooterSubsystem) {
        joystickL = joyL;
        joystickR = joyR;
        shooter   = shooterSubsystem;
        orbitRevShooterCommand = shooter
            .run(() -> shooter.setTargetRPM(ShotTuning.commandedRpm()))
            .withName("HubLockRev");
        ShotTuning.init();
    }

    public ShotCalculator.Solution getLastSolution() { return lastSolution; }

    // =========================================================================
    // Main drive update — call every teleop tick
    // =========================================================================
    public void drivePID() {

        // ── 1. Toggle hub-lock on rising edge ──────────────────────────────
        boolean orbitButtonNow = joystickL.button(ORBIT_TOGGLE_BUTTON).getAsBoolean();
        if (orbitButtonNow && !orbitButtonWasPressed) {
            hubOrbitEnabled = !hubOrbitEnabled;
        }
        orbitButtonWasPressed = orbitButtonNow;

        boolean tuning = ShotTuning.isEnabled();
        boolean lockActive = hubOrbitEnabled || tuning;
        isOrbit = lockActive;

        // Keep the shooter revving while locked; hand it back when not.
        // (If the shoot button grabbed the shooter and released it, the default
        // STOPP() resumes and we re-schedule the rev here.)
        if (lockActive) {
            if (!orbitRevShooterCommand.isScheduled()
                && shooter.getCurrentCommand() == shooter.getDefaultCommand()) {
                orbitRevShooterCommand.schedule();
            }
        } else {
            if (orbitRevShooterCommand.isScheduled()) orbitRevShooterCommand.cancel();
            lastSolution = null;
        }

        // ── 1b. Wheel lock (X-brake) while button held ─────────────────────
        if (joystickR.button(WHEEL_LOCK_BUTTON).getAsBoolean()) {
            xfilter.reset(0);
            yfilter.reset(0);
            rfilter.reset(0);
            xV = 0;
            yV = 0;
            rV = 0;
            RobotContainer.drivebase.lock();
            Logger.recordOutput("Teleop/WheelLock", true);
            return;
        }
        Logger.recordOutput("Teleop/WheelLock", false);

        // ── 2. Read & filter joystick translation axes ─────────────────────
        double xSpeedJoystick = -joystickL.getRawAxis(1);
        if (Math.abs(xSpeedJoystick) < Settings.joystickDeadband) xSpeedJoystick = 0;
        xSpeedJoystick = xfilter.calculate(xSpeedJoystick);

        double ySpeedJoystick = -joystickL.getRawAxis(0);
        if (Math.abs(ySpeedJoystick) < Settings.joystickDeadband) ySpeedJoystick = 0;
        ySpeedJoystick = yfilter.calculate(ySpeedJoystick);

        double rSpeedJoystick = -joystickR.getRawAxis(2);
        if (Math.abs(rSpeedJoystick) < Settings.joystickDeadband) rSpeedJoystick = 0;
        rSpeedJoystick = rfilter.calculate(rSpeedJoystick);

        // ── 3. Alliance flip (field-centric) ───────────────────────────────
        boolean isRed = HubAlignment.isRedAlliance();
        if (isRed) {
            xSpeedJoystick = -xSpeedJoystick;
            ySpeedJoystick = -ySpeedJoystick;
        }

        // ── 4. Square for smoother feel ────────────────────────────────────
        double xInput = Math.copySign(xSpeedJoystick * xSpeedJoystick, xSpeedJoystick);
        double yInput = Math.copySign(ySpeedJoystick * ySpeedJoystick, ySpeedJoystick);
        double rInput = Math.copySign(rSpeedJoystick * rSpeedJoystick, rSpeedJoystick);

        // ── 5. Scale to physical velocities ───────────────────────────────
        double maxV   = RobotContainer.drivebase.getMaximumVelocity();
        double maxOmg = RobotContainer.drivebase.getMaximumChassisAngularVelocity();

        xV = xInput * maxV;
        yV = yInput * maxV;
        rV = rInput * maxOmg;

        // ── 6. Hub-lock overlay ───────────────────────────────────────────
        if (tuning) {
            applyTuneLock(isRed);
        } else if (hubOrbitEnabled) {
            applyHubLock();
        }

        // ── 7. Send to drivebase ───────────────────────────────────────────
        RobotContainer.drivebase.drive(xV, yV, rV, true);
    }

    // =========================================================================
    // Normal hub-lock: speed limit + range-keeping + aim at the virtual target
    // =========================================================================
    private void applyHubLock() {
        ShotCalculator.Solution sol = ShotCalculator.solveNow();
        lastSolution = sol;

        // 1. Speed limit while locked.
        limitSpeed();

        // 2. Tangential motion is free; radial is clamped so the shot distance
        //    stays inside the table (and pulled back in if outside).
        double ux = sol.unitAway.getX();
        double uy = sol.unitAway.getY();
        double vAway = xV * ux + yV * uy;
        double tx = xV - vAway * ux;
        double ty = yV - vAway * uy;
        double vAwayClamped = sol.clampRadialVelocity(vAway);
        xV = tx + vAwayClamped * ux;
        yV = ty + vAwayClamped * uy;

        // 3. Face the virtual target (P + feedforward for the moving aim line).
        rV = MathUtil.clamp(
            HEADING_P * sol.headingErrorRad + sol.headingRateFF,
            -HEADING_MAX_RAD_S, HEADING_MAX_RAD_S);

        Logger.recordOutput("Shot/RPM", sol.rpm);
        Logger.recordOutput("Shot/TimeOfFlight", sol.tofSeconds);
        Logger.recordOutput("Shot/EffectiveDistance", sol.effectiveDistanceM);
        Logger.recordOutput("Shot/RealDistance", sol.realDistanceM);
        Logger.recordOutput("Shot/InRange", sol.inRange);
        Logger.recordOutput("Shot/HeadingErrorDeg", Math.toDegrees(sol.headingErrorRad));
        Logger.recordOutput("Shot/VirtualTarget", new Pose2d(sol.virtualTarget, sol.aimHeading));
    }

    // =========================================================================
    // Tune mode: hold the typed distance from the hub, face the REAL hub,
    // driver may only strafe around it. Ignores the table's min/max.
    // =========================================================================
    private void applyTuneLock(boolean isRed) {
        Translation2d hub = HubAlignment.allianceHub(isRed);
        HubAlignment.Snapshot snap =
            HubAlignment.compute(RobotContainer.drivebase.getPose(), hub, ShotTuning.getDistance());
        if (snap == null) return;

        limitSpeed();

        // Strip all radial driver input, keep tangential.
        double ux = snap.unitAway.getX();
        double uy = snap.unitAway.getY();
        double radial = xV * ux + yV * uy;
        xV -= radial * ux;
        yV -= radial * uy;

        // Drive to the typed distance (error > 0 = too far -> move toward hub).
        double snapSpeed = HubAlignment.radialSnapSpeed(snap.distanceError, TUNE_SNAP_GAIN, TUNE_SNAP_MAX_SPEED);
        xV -= snapSpeed * ux;
        yV -= snapSpeed * uy;

        rV = MathUtil.clamp(HEADING_P * snap.headingError, -HEADING_MAX_RAD_S, HEADING_MAX_RAD_S);

        SmartDashboard.putNumber("ShotTune/Current Distance (m)", snap.distance);
        SmartDashboard.putBoolean("ShotTune/At Distance",
            Math.abs(snap.distanceError) < TUNE_AT_DISTANCE_TOL_M);
        SmartDashboard.putBoolean("ShotTune/At RPM", shooter.isAtTargetSpeed(75));
    }

    private void limitSpeed() {
        double speed = Math.hypot(xV, yV);
        if (speed > LOCKED_MAX_SPEED_MPS) {
            double s = LOCKED_MAX_SPEED_MPS / speed;
            xV *= s;
            yV *= s;
        }
    }
}