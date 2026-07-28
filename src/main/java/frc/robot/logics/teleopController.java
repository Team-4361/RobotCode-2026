package frc.robot.logics;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Settings;
import frc.robot.subsystems.ShooterSubsystem;

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

    public SlewRateLimiter xfilter = new SlewRateLimiter(4);
    public SlewRateLimiter yfilter = new SlewRateLimiter(4);
    public SlewRateLimiter rfilter = new SlewRateLimiter(4);

    // ── Hub-orbit state ───────────────────────────────────────────────────────
    /**
     * When true the robot will:
     *   • Always face the alliance hub (front of robot pointed at hub).
     *   • Never approach closer than HUB_KEEP_DISTANCE_M.
     *     If the driver tries to drive into the hub, the radial component
     *     of their input is clamped / reversed so the robot stays ~1 m out.
     *   • Rev the shooter to RobotContainer.SHOOTER_REV_SPEED (see
     *     orbitRevShooterCommand below) — teleop-only, SnapToHubCommand
     *     (the autonomous version) never touches the shooter.
     *
     * Toggle with joystickL button 1 (change ORBIT_TOGGLE_BUTTON if needed).
     *
     * NOTE: this runs INLINE as an overlay at the end of drivePID() below —
     * it is not a separate command/loop, so it never competes with the main
     * drive update for a scheduler slot. The geometry itself is shared with
     * SnapToHubCommand via HubAlignment so it's only implemented once.
     */
    private boolean hubOrbitEnabled = false;
    private boolean orbitButtonWasPressed = false;   // for edge-detect toggle

    /** Button index on joystickL that toggles hub-orbit mode. */ 
    private static final int ORBIT_TOGGLE_BUTTON = 1;

    /**
     * The minimum distance (meters) the robot keeps from the hub centre
     * while orbit mode is active.
     */
    private static final double HUB_KEEP_DISTANCE_M = 2.502154;

    /** P-gain for snapping to HUB_KEEP_DISTANCE_M (m/s per metre of error). */
    private static final double HUB_SNAP_GAIN = 4.0;

    /** Maximum speed the snap controller will command toward/away from hub (m/s). */
    private static final double HUB_SNAP_MAX_SPEED = 2.5;

    /**
     * (rad/s per radian of error).
     */
    private static final double HEADING_P = 6.0;

    /** Maximum rotational speed the heading controller will command (rad/s). */
    private static final double HEADING_MAX_RAD_S = 7.0;

    /**
     * Command that revs the shooter while hub-orbit is active. Scheduling it
     * takes the shooter subsystem away from its default STOPP() command;
     * cancelling it hands the subsystem back to that default automatically.
     * Built once in the constructor so toggling doesn't allocate every press.
     */
    private final Command orbitRevShooterCommand;

    // ─────────────────────────────────────────────────────────────────────────

    public teleopController(CommandJoystick joyL,
                            CommandJoystick joyR,
                            CommandXboxController xboxCommandJoystick,
                            ShooterSubsystem shooterSubsystem) {
        joystickL    = joyL;
        joystickR    = joyR;
        shooter      = shooterSubsystem;
        orbitRevShooterCommand = shooter.setSPEED(RobotContainer.SHOOTER_REV_SPEED);
    }

    // =========================================================================
    // Main drive update — call every robot-periodic tick
    // =========================================================================
    public void drivePID() {

        // ── 1. Toggle hub-orbit on rising edge of button ───────────────────
        boolean orbitButtonNow = joystickL.button(ORBIT_TOGGLE_BUTTON).getAsBoolean();
        if (orbitButtonNow && !orbitButtonWasPressed) {
            hubOrbitEnabled = !hubOrbitEnabled;

            // Rev the shooter in step with orbit mode — teleop only.
            if (hubOrbitEnabled) {
                orbitRevShooterCommand.schedule();
            } else {
                orbitRevShooterCommand.cancel(); // shooter falls back to its default STOPP()
            }
        }
        orbitButtonWasPressed = orbitButtonNow;
        isOrbit = hubOrbitEnabled;

        // ── 2. Read & filter joystick translation axes ─────────────────────
        double xSpeedJoystick = -joystickL.getRawAxis(1); // forward/back (inverted)
        if (Math.abs(xSpeedJoystick) < Settings.joystickDeadband) xSpeedJoystick = 0;
        xSpeedJoystick = xfilter.calculate(xSpeedJoystick);

        double ySpeedJoystick = -joystickL.getRawAxis(0); // left/right (inverted)
        if (Math.abs(ySpeedJoystick) < Settings.joystickDeadband) ySpeedJoystick = 0;
        ySpeedJoystick = yfilter.calculate(ySpeedJoystick);

        double rSpeedJoystick = -joystickR.getRawAxis(2); // rotation
        if (Math.abs(rSpeedJoystick) < Settings.joystickDeadband) rSpeedJoystick = 0;
        rSpeedJoystick = rfilter.calculate(rSpeedJoystick);

        // ── 3. Alliance flip (field-centric) ───────────────────────────────
        boolean isRed = HubAlignment.isRedAlliance();
        if (isRed) {
            xSpeedJoystick = -xSpeedJoystick;
            ySpeedJoystick = -ySpeedJoystick;
        }

        // ── 4. Cube for smoother feel ──────────────────────────────────────
        double xInput = Math.pow(xSpeedJoystick, 3);
        double yInput = Math.pow(ySpeedJoystick, 3);
        double rInput = Math.pow(rSpeedJoystick, 3);

        // ── 5. Scale to physical velocities ───────────────────────────────
        double maxV   = RobotContainer.drivebase.getMaximumVelocity();
        double maxOmg = RobotContainer.drivebase.getMaximumChassisAngularVelocity();

        xV = xInput * maxV;
        yV = yInput * maxV;
        rV = rInput * maxOmg;

        // ── 6. Hub-orbit overlay (inline, not a separate command) ──────────
        if (hubOrbitEnabled) {
            applyHubOrbit(isRed);
        }

        // ── 7. Send to drivebase ───────────────────────────────────────────
        RobotContainer.drivebase.drive(xV, yV, rV, true);
    }

    // =========================================================================
    // Hub-orbit logic
    // =========================================================================

    /**
     * Modifies xV / yV / rV so that:
     *   • The robot never comes closer than HUB_KEEP_DISTANCE_M to the hub.
     *   • The robot's front always faces the hub (rV overridden by P controller).
     *
     * All maths is in field-centric coordinates (same frame as drivebase.drive).
     * Geometry itself comes from HubAlignment so it's computed once here and
     * shared (not re-derived) with SnapToHubCommand's autonomous version.
     */
    private void applyHubOrbit(boolean isRed) {

        Pose2d pose = RobotContainer.drivebase.getPose();
        Translation2d hub = HubAlignment.allianceHub(isRed);

        HubAlignment.Snapshot snap = HubAlignment.compute(pose, hub, HUB_KEEP_DISTANCE_M);
        if (snap == null) return; // robot is essentially on top of the hub — skip this tick

        // Strip ALL radial driver input — driver can only move tangentially.
        // The distance is controlled entirely by the snap correction below.
        double radialV = xV * snap.unitAway.getX() + yV * snap.unitAway.getY();
        xV -= radialV * snap.unitAway.getX();
        yV -= radialV * snap.unitAway.getY();

        // Snap correction: P-controller drives robot to exactly HUB_KEEP_DISTANCE_M.
        // distanceError > 0 (too far)  → pull inward  (subtract unitAway)
        // distanceError < 0 (too close) → push outward (add unitAway)
        double snapSpeed = HubAlignment.radialSnapSpeed(snap.distanceError, HUB_SNAP_GAIN, HUB_SNAP_MAX_SPEED);
        xV -= snapSpeed * snap.unitAway.getX();
        yV -= snapSpeed * snap.unitAway.getY();

        // ── Heading P-controller — front of robot faces hub ────────────
        rV = HubAlignment.headingSnapSpeed(snap.headingError, HEADING_P, HEADING_MAX_RAD_S);
    }
}