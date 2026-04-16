package frc.robot.logics;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Settings;

public class teleopController {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private CommandJoystick joystickL;
    private CommandJoystick joystickR;

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
     *
     * Toggle with joystickL button 1 (change ORBIT_TOGGLE_BUTTON if needed).
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

    // ─────────────────────────────────────────────────────────────────────────

    public teleopController(CommandJoystick joyL,
                            CommandJoystick joyR,
                            CommandXboxController xboxCommandJoystick) {
        joystickL    = joyL;
        joystickR    = joyR;
    }

    // =========================================================================
    // Main drive update — call every robot-periodic tick
    // =========================================================================
    public void drivePID() {

        // ── 1. Toggle hub-orbit on rising edge of button ───────────────────
        boolean orbitButtonNow = joystickL.button(ORBIT_TOGGLE_BUTTON).getAsBoolean();
        if (orbitButtonNow && !orbitButtonWasPressed) {
            hubOrbitEnabled = !hubOrbitEnabled;
        }
        orbitButtonWasPressed = orbitButtonNow;

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
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get().equals(Alliance.Red);
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

        // ── 6. Hub-orbit overlay ───────────────────────────────────────────
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
     */
    private void applyHubOrbit(boolean isRed) {

        // ── a. Current pose from YAGSL ────────────────────────────────────
        Pose2d pose       = RobotContainer.drivebase.getPose();
        Translation2d robotPos = pose.getTranslation();

        // ── b. Alliance hub centre ────────────────────────────────────────
        Translation2d hub = isRed
                ? RobotContainer.HUB_CENTER_RED
                : RobotContainer.HUB_CENTER_BLUE;

        // ── c. Hub→robot vector ───────────────────────────────────────────
        Translation2d toRobot = robotPos.minus(hub);
        double distance = toRobot.getNorm();

        if (distance < 0.01) return; // safety guard

        Translation2d unitAway = toRobot.div(distance); // unit vec pointing away from hub

        // ── d. Distance correction velocity ──────────────────────────────
        //
        // If the robot is too close, add a correction velocity pushing it
        // outward (away from hub). The further inside the limit, the stronger.
        // This runs ON TOP of whatever the driver is doing, so tangential
        // (left/right around the hub) movement is fully preserved.
        //
        // error > 0  →  robot is inside the keep-distance, push out      
        // error <= 0 →  robot is far enough, no correction needed

        // Strip ALL radial driver input — driver can only move tangentially.
        // The distance is controlled entirely by the snap correction below.
        double radialV = xV * unitAway.getX() + yV * unitAway.getY();
        xV -= radialV * unitAway.getX();
        yV -= radialV * unitAway.getY();

        // Snap correction: P-controller drives robot to exactly HUB_KEEP_DISTANCE_M.
        // Positive error = too far away → move toward hub (negative unitAway).
        // Negative error = too close    → move away from hub (positive unitAway).
        // distanceError > 0 = too far  → need to move TOWARD hub = subtract unitAway
        // distanceError < 0 = too close → need to move AWAY   = add unitAway
        double distanceError = distance - HUB_KEEP_DISTANCE_M;
        double snapSpeed = MathUtil.clamp(
                HUB_SNAP_GAIN * distanceError,
                -HUB_SNAP_MAX_SPEED,
                 HUB_SNAP_MAX_SPEED
        );
        // Negate: positive error should pull inward (opposite of unitAway)
        xV -= snapSpeed * unitAway.getX();
        yV -= snapSpeed * unitAway.getY();

        // ── e. Heading P-controller — front of robot faces hub ────────────
        //
        // Desired heading = angle pointing FROM robot TOWARD hub
        //                 = angle of (hub - robot) = angle of -unitAway

        Rotation2d desiredHeading = new Rotation2d(
                Math.atan2(-unitAway.getY(), -unitAway.getX())
        );

        double headingError = MathUtil.angleModulus(
                desiredHeading.minus(pose.getRotation()).getRadians()
        );

        rV = MathUtil.clamp(
                HEADING_P * headingError,
                -HEADING_MAX_RAD_S,
                 HEADING_MAX_RAD_S
        );
    }
}