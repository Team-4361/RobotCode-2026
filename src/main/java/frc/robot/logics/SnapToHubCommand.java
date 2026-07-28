package frc.robot.logics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * Drives the robot to HUB_KEEP_DISTANCE_M from the alliance hub and squares
 * it up to face the hub, then finishes.
 *
 * Shares its geometry (hub vector, distance error, heading error) with the
 * teleop hub-orbit overlay via HubAlignment, so that math exists in exactly
 * one place. Within a single scheduler pass, execute() and isFinished() are
 * called back-to-back on the same tick — this now computes the snapshot
 * once in execute() and reuses it in isFinished() instead of re-reading the
 * pose and redoing the atan2/angleModulus work a second time.
 */
public class SnapToHubCommand extends Command {

    // ── Tuning ────────────────────────────────────────────────────────────────

    /** Target distance from hub centre (metres). */
    private static final double HUB_KEEP_DISTANCE_M  = 2.502154;

    /** P-gain for radial snap (m/s per metre of error). */
    private static final double HUB_SNAP_GAIN        = 4.0;

    /** Max speed toward/away from hub (m/s). */
    private static final double HUB_SNAP_MAX_SPEED   = 2.5;

    /** P-gain for heading (rad/s per radian of error). */
    private static final double HEADING_P            = 5.0;

    /** Max angular speed from heading controller (rad/s). */
    private static final double HEADING_MAX_RAD_S    = 4.0;

    /**
     * Tolerance to consider "done":
     *   distance error < DISTANCE_TOLERANCE_M  AND
     *   heading error  < HEADING_TOLERANCE_RAD
     */
    private static final double DISTANCE_TOLERANCE_M   = 0.05; // 5 cm
    private static final double HEADING_TOLERANCE_RAD  = Math.toRadians(3); // 3°

    // ─────────────────────────────────────────────────────────────────────────

    /** Snapshot from the most recent execute() call, reused by isFinished(). */
    private HubAlignment.Snapshot lastSnapshot = null;

    public SnapToHubCommand() {
        // Require drivebase so this blocks any concurrent PathPlanner command
        addRequirements(RobotContainer.drivebase);
    }

    @Override
    public void initialize() {
        lastSnapshot = null;
    }

    @Override
    public void execute() {
        HubAlignment.Snapshot snap = HubAlignment.compute(
                RobotContainer.drivebase.getPose(),
                HubAlignment.allianceHub(HubAlignment.isRedAlliance()),
                HUB_KEEP_DISTANCE_M
        );
        lastSnapshot = snap;

        if (snap == null) {
            // Degenerate: robot is on top of the hub — hold still rather than
            // command garbage off a zero-length vector.
            RobotContainer.drivebase.drive(0, 0, 0, true);
            return;
        }

        // distanceError > 0 → too far  → move toward hub (subtract unitAway)
        // distanceError < 0 → too close → move away    (add unitAway)
        double snapSpeed = HubAlignment.radialSnapSpeed(snap.distanceError, HUB_SNAP_GAIN, HUB_SNAP_MAX_SPEED);
        double xV = -snapSpeed * snap.unitAway.getX();
        double yV = -snapSpeed * snap.unitAway.getY();

        double rV = HubAlignment.headingSnapSpeed(snap.headingError, HEADING_P, HEADING_MAX_RAD_S);

        RobotContainer.drivebase.drive(xV, yV, rV, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot cleanly when done (or interrupted by e.g. disable)
        RobotContainer.drivebase.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        if (lastSnapshot == null) return false;

        double distanceError = Math.abs(lastSnapshot.distanceError);
        double headingError  = Math.abs(lastSnapshot.headingError);

        return distanceError < DISTANCE_TOLERANCE_M
            && headingError  < HEADING_TOLERANCE_RAD;
    }
}